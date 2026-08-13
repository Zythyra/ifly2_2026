#include <ucar_controller/base_driver.h>
#include <Eigen/Eigen>
#include <cstdint>
#include <deque>
#include <std_msgs/Bool.h>

namespace ucarController
{
namespace
{
// ==================== BASE-DIAG V2 ====================
// Diagnostics only. These variables do not participate in control decisions.
bool g_diag_enable = true;
bool g_diag_verbose = false;

double g_diag_lock_wait_warn_ms = 2.0;
double g_diag_cmd_gap_warn_ms = 90.0;
double g_diag_write_warn_ms = 3.0;
double g_diag_tx_gap_warn_ms = 75.0;
int g_diag_repeat_warn_streak = 2;    // 2 extra repeats => same RX sent 3 TX cycles total
int g_diag_repeat_error_streak = 3;   // 3 extra repeats => same RX sent 4 TX cycles total
int g_diag_skip_warn_count = 2;       // warn only when >=2 RX commands are skipped at once

double g_diag_nonzero_linear_eps = 0.01;
double g_diag_nonzero_angular_eps = 0.02;

uint64_t g_cmd_rx_seq = 0;
uint64_t g_last_tx_selected_seq = 0;
uint64_t g_repeat_tx_count = 0;
uint64_t g_repeat_streak = 0;
uint64_t g_max_repeat_streak = 0;
uint64_t g_skipped_rx_total = 0;
uint64_t g_skip_event_count = 0;
uint64_t g_short_write_count = 0;
uint64_t g_nonzero_timeout_count = 0;
uint64_t g_stale_before_write_count = 0;

bool g_last_tx_was_timeout = false;
double g_last_rx_lock_wait_ms = 0.0;
double g_last_rx_interval_ms = 0.0;
ros::WallTime g_last_cmd_callback_wall_time;
ros::WallTime g_last_tx_wall_time;

ros::Publisher g_applied_cmd_pub;
ros::Publisher g_tx_diag_pub;

// ==================== BASE-FAILSAFE V4.2 ====================
// Software-only fail-safe for the observed RK3588 -> MCU stale/delayed command
// fault.  NORMAL control is unchanged.  V4 estimates the effective command ->
// odometry delay from recent trajectories instead of requiring odom to freeze.
enum FailsafeState
{
  FS_NORMAL = 0,
  FS_EMERGENCY_BRAKE = 1,
  FS_HOLD_STOP = 2
};

struct FailsafeSample
{
  ros::WallTime wall_time;  // history retention / diagnostics
  ros::Time tx_stamp;       // time this command was selected for serial TX
  ros::Time odom_stamp;     // timestamp of the measured odom snapshot
  double cmd_x;
  double cmd_y;
  double cmd_w;
  double odom_x;
  double odom_y;
  double odom_w;
};

bool g_fs_enable = true;
double g_fs_zero_rate_hz = 100.0;
double g_fs_odom_max_age_ms = 120.0;

// Channel A: strong persistent angular sign mismatch.
double g_fs_reverse_cmd_w_min = 1.0;
double g_fs_reverse_odom_w_min = 0.8;
int g_fs_reverse_confirm_cycles = 5;  // roughly 250 ms at 20 Hz

// Channel B: rolling trajectory delay estimator.
double g_fs_history_keep_ms = 1400.0;
double g_fs_delay_eval_window_ms = 220.0;
double g_fs_delay_min_ms = 0.0;
double g_fs_delay_max_ms = 650.0;
double g_fs_delay_step_ms = 50.0;
double g_fs_delay_normal_max_ms = 200.0;
double g_fs_delay_fault_min_ms = 250.0;
double g_fs_delay_match_tolerance_ms = 32.0;
int g_fs_delay_min_eval_odom_samples = 5;
int g_fs_delay_confirm_odom_frames = 2;

// Delay is not identifiable on a nearly constant command.  Require excitation
// in at least one control dimension over the evaluation window.
double g_fs_delay_cmd_span_linear_min = 0.25;
double g_fs_delay_cmd_span_angular_min = 0.90;

// Ignore trivial near-stop mismatch and normal small tracking error.
double g_fs_detection_min_linear = 0.15;
double g_fs_detection_min_angular = 0.50;
double g_fs_current_mismatch_linear = 0.25;
double g_fs_current_mismatch_angular = 0.80;

// Per-sample normalized trajectory fit score:
// score = sqrt((linear_error/scale_linear)^2 +
//              (angular_error/scale_angular)^2)
double g_fs_delay_score_linear_scale = 0.22;
double g_fs_delay_score_angular_scale = 0.55;

// A large delay must fit the measured trajectory well AND substantially better
// than every candidate in the normal-delay region.
double g_fs_delay_fault_score_max = 2.20;
double g_fs_delay_vs_normal_ratio_max = 0.85;
double g_fs_delay_improvement_min = 0.25;

// Channel C: FAST high-confidence abnormal-delay trigger.
// Unlike the regular delay detector, this channel needs only one FRESH odom
// frame, but it also requires a large current cmd/odom mismatch.
// ==================== BASE-FAILSAFE V4.2 ====================
// Diagnostic-output switch.
// false disables only BASE-DIAG logs and pure diagnostic topics.
// It NEVER disables any failsafe detector or emergency braking.
bool g_base_diag_enable = true;

bool g_fs_fast_delay_enable = true;
double g_fs_fast_delay_min_ms = 300.0;
double g_fs_fast_delay_score_max = 2.20;
double g_fs_fast_delay_ratio_max = 0.82;
double g_fs_fast_delay_improvement_min = 0.30;
double g_fs_fast_mismatch_linear = 0.30;
double g_fs_fast_mismatch_angular = 1.00;

// Brake/recovery state machine.
double g_fs_min_brake_ms = 200.0;
double g_fs_hold_ms = 300.0;
double g_fs_stop_linear = 0.05;
double g_fs_stop_angular = 0.12;
int g_fs_stop_confirm_odom_frames = 3;
double g_fs_recovery_grace_ms = 500.0;

FailsafeState g_fs_state = FS_NORMAL;
uint64_t g_fs_trigger_count = 0;
int g_fs_reverse_count = 0;
int g_fs_delay_confirm_count = 0;
int g_fs_stop_confirm_count = 0;
ros::WallTime g_fs_trigger_wall;
ros::WallTime g_fs_hold_start_wall;
ros::WallTime g_fs_last_recover_wall;
ros::Time g_fs_last_stop_check_odom_stamp;
ros::Time g_fs_last_detection_odom_stamp;
std::deque<FailsafeSample> g_fs_history;
ros::Publisher g_fs_active_pub;

static inline double fsLinearDiff(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return sqrt(dx * dx + dy * dy);
}

static inline double fsDelaySampleScore(double cmd_x, double cmd_y, double cmd_w,
                                        double odom_x, double odom_y, double odom_w)
{
  const double lin_scale = g_fs_delay_score_linear_scale > 1e-6
                               ? g_fs_delay_score_linear_scale
                               : 1e-6;
  const double ang_scale = g_fs_delay_score_angular_scale > 1e-6
                               ? g_fs_delay_score_angular_scale
                               : 1e-6;
  const double lin = fsLinearDiff(cmd_x, cmd_y, odom_x, odom_y) / lin_scale;
  const double ang = fabs(cmd_w - odom_w) / ang_scale;
  return sqrt(lin * lin + ang * ang);
}

static inline void resetFailsafeDetectionState()
{
  g_fs_reverse_count = 0;
  g_fs_delay_confirm_count = 0;
  g_fs_last_detection_odom_stamp = ros::Time();
}

static inline void publishFailsafeState(bool active)
{
  std_msgs::Bool msg;
  msg.data = active;
  g_fs_active_pub.publish(msg);
}
} // namespace
baseBringup::baseBringup() :x_(0), y_(0), th_(0)
{
  ros::NodeHandle pravite_nh("~");
  pravite_nh.param("provide_odom_tf", provide_odom_tf_,true);
  pravite_nh.param("vel_topic", vel_topic_,std::string("/cmd_vel"));///smooth_cmd_vel
  
  pravite_nh.param("joy_topic",  joy_topic_, std::string("/joy"));
  pravite_nh.param("odom_topic", odom_topic_,std::string("/odom"));
  pravite_nh.param("battery_topic", battery_topic_,std::string("/battery_state"));
  //serial
  pravite_nh.param("port", port_, std::string("/dev/base_serial_port"));
  pravite_nh.param("baud", baud_, 115200);                
  pravite_nh.param("serial_timeout", serial_timeout_, 50);//ms
  pravite_nh.param("rate", rate_, 20);                    //hz
  pravite_nh.param("duration", duration_, 0.01);
  pravite_nh.param("cmd_timeout", cmd_dt_threshold_, 0.2);
  
  pravite_nh.param("base_frame", base_frame_, std::string("base_footprint"));
  pravite_nh.param("odom_frame", odom_frame_, std::string("odom"));

  pravite_nh.param("encode_resolution", encode_resolution_, 270);  //   
  pravite_nh.param("wheel_radius", wheel_radius_, 0.04657);  //   m
  pravite_nh.param("period", period_, 50.0); //ms
  pravite_nh.param("base_shape_a", base_shape_a_, 0.2169);  //   m
  pravite_nh.param("base_shape_b", base_shape_b_, 0.0);  //   m

  pravite_nh.param("linear_speed_max",   linear_speed_max_, 3.0);  //   m/s
  pravite_nh.param("angular_speed_max", angular_speed_max_, 3.14);// rad/s
  pravite_nh.setParam("linear_speed_max" ,linear_speed_max_);
  pravite_nh.setParam("angular_speed_max",angular_speed_max_);

  pravite_nh.param("Mileage_file_name", Mileage_file_name_, std::string("car_Mileage_info.txt"));//
  Mileage_file_name_        = ros::package::getPath("ucar_controller") + std::string("/log_info/") + Mileage_file_name_;
  Mileage_backup_file_name_ = Mileage_file_name_ + ".bp";
  pravite_nh.param("debug_log", debug_log_, true);//  true rosinfo 等等  打印log数据


  // ==================== BASE-DIAG V2 parameters ====================
  pravite_nh.param("diagnostic_enable", g_diag_enable, true);
  pravite_nh.param("diagnostic_verbose", g_diag_verbose, false);
  pravite_nh.param("diag_lock_wait_warn_ms", g_diag_lock_wait_warn_ms, 2.0);
  pravite_nh.param("diag_cmd_gap_warn_ms", g_diag_cmd_gap_warn_ms, 90.0);
  pravite_nh.param("diag_write_warn_ms", g_diag_write_warn_ms, 3.0);
  pravite_nh.param("diag_tx_gap_warn_ms", g_diag_tx_gap_warn_ms, 75.0);
  pravite_nh.param("diag_repeat_warn_streak", g_diag_repeat_warn_streak, 2);
  pravite_nh.param("diag_repeat_error_streak", g_diag_repeat_error_streak, 3);
  pravite_nh.param("diag_skip_warn_count", g_diag_skip_warn_count, 2);
  pravite_nh.param("diag_nonzero_linear_eps", g_diag_nonzero_linear_eps, 0.01);
  pravite_nh.param("diag_nonzero_angular_eps", g_diag_nonzero_angular_eps, 0.02);

  // ==================== BASE-FAILSAFE V4.2 parameters ====================
  pravite_nh.param("failsafe_enable", g_fs_enable, true);
  pravite_nh.param("failsafe_zero_rate_hz", g_fs_zero_rate_hz, 100.0);
  pravite_nh.param("failsafe_odom_max_age_ms", g_fs_odom_max_age_ms, 120.0);

  pravite_nh.param("failsafe_reverse_cmd_w_min", g_fs_reverse_cmd_w_min, 1.0);
  pravite_nh.param("failsafe_reverse_odom_w_min", g_fs_reverse_odom_w_min, 0.8);
  pravite_nh.param("failsafe_reverse_confirm_cycles", g_fs_reverse_confirm_cycles, 5);

  pravite_nh.param("failsafe_history_keep_ms", g_fs_history_keep_ms, 1400.0);
  pravite_nh.param("failsafe_delay_eval_window_ms", g_fs_delay_eval_window_ms, 220.0);
  pravite_nh.param("failsafe_delay_min_ms", g_fs_delay_min_ms, 0.0);
  pravite_nh.param("failsafe_delay_max_ms", g_fs_delay_max_ms, 650.0);
  pravite_nh.param("failsafe_delay_step_ms", g_fs_delay_step_ms, 50.0);
  pravite_nh.param("failsafe_delay_normal_max_ms", g_fs_delay_normal_max_ms, 200.0);
  pravite_nh.param("failsafe_delay_fault_min_ms", g_fs_delay_fault_min_ms, 250.0);
  pravite_nh.param("failsafe_delay_match_tolerance_ms", g_fs_delay_match_tolerance_ms, 32.0);
  pravite_nh.param("failsafe_delay_min_eval_odom_samples", g_fs_delay_min_eval_odom_samples, 5);
  pravite_nh.param("failsafe_delay_confirm_odom_frames", g_fs_delay_confirm_odom_frames, 2);

  pravite_nh.param("failsafe_delay_cmd_span_linear_min", g_fs_delay_cmd_span_linear_min, 0.25);
  pravite_nh.param("failsafe_delay_cmd_span_angular_min", g_fs_delay_cmd_span_angular_min, 0.90);

  pravite_nh.param("failsafe_detection_min_linear", g_fs_detection_min_linear, 0.15);
  pravite_nh.param("failsafe_detection_min_angular", g_fs_detection_min_angular, 0.50);
  pravite_nh.param("failsafe_current_mismatch_linear", g_fs_current_mismatch_linear, 0.25);
  pravite_nh.param("failsafe_current_mismatch_angular", g_fs_current_mismatch_angular, 0.80);

  pravite_nh.param("failsafe_delay_score_linear_scale", g_fs_delay_score_linear_scale, 0.22);
  pravite_nh.param("failsafe_delay_score_angular_scale", g_fs_delay_score_angular_scale, 0.55);
  pravite_nh.param("failsafe_delay_fault_score_max", g_fs_delay_fault_score_max, 2.20);
  pravite_nh.param("failsafe_delay_vs_normal_ratio_max", g_fs_delay_vs_normal_ratio_max, 0.85);
  pravite_nh.param("failsafe_delay_improvement_min", g_fs_delay_improvement_min, 0.25);

  // Competition/debug diagnostic output switch.
  pravite_nh.param("base_diag_enable", g_base_diag_enable, true);

  // FAST_DELAY: high-confidence single-fresh-odom trigger.
  pravite_nh.param("failsafe_fast_delay_enable", g_fs_fast_delay_enable, true);
  pravite_nh.param("failsafe_fast_delay_min_ms", g_fs_fast_delay_min_ms, 300.0);
  pravite_nh.param("failsafe_fast_delay_score_max", g_fs_fast_delay_score_max, 2.20);
  pravite_nh.param("failsafe_fast_delay_ratio_max", g_fs_fast_delay_ratio_max, 0.82);
  pravite_nh.param("failsafe_fast_delay_improvement_min", g_fs_fast_delay_improvement_min, 0.30);
  pravite_nh.param("failsafe_fast_mismatch_linear", g_fs_fast_mismatch_linear, 0.30);
  pravite_nh.param("failsafe_fast_mismatch_angular", g_fs_fast_mismatch_angular, 1.00);

  pravite_nh.param("failsafe_min_brake_ms", g_fs_min_brake_ms, 200.0);
  pravite_nh.param("failsafe_hold_ms", g_fs_hold_ms, 300.0);
  pravite_nh.param("failsafe_stop_linear", g_fs_stop_linear, 0.05);
  pravite_nh.param("failsafe_stop_angular", g_fs_stop_angular, 0.12);
  pravite_nh.param("failsafe_stop_confirm_odom_frames", g_fs_stop_confirm_odom_frames, 3);
  pravite_nh.param("failsafe_recovery_grace_ms", g_fs_recovery_grace_ms, 500.0);

  g_cmd_rx_seq = 0;
  g_last_tx_selected_seq = 0;
  g_repeat_tx_count = 0;
  g_repeat_streak = 0;
  g_max_repeat_streak = 0;
  g_skipped_rx_total = 0;
  g_skip_event_count = 0;
  g_short_write_count = 0;
  g_nonzero_timeout_count = 0;
  g_stale_before_write_count = 0;
  g_last_tx_was_timeout = false;
  g_last_rx_lock_wait_ms = 0.0;
  g_last_rx_interval_ms = 0.0;
  g_last_cmd_callback_wall_time = ros::WallTime();
  g_last_tx_wall_time = ros::WallTime();
  g_fs_state = FS_NORMAL;
  g_fs_trigger_count = 0;
  g_fs_reverse_count = 0;
  g_fs_delay_confirm_count = 0;
  g_fs_stop_confirm_count = 0;
  g_fs_trigger_wall = ros::WallTime();
  g_fs_hold_start_wall = ros::WallTime();
  g_fs_last_recover_wall = ros::WallTime();
  g_fs_last_stop_check_odom_stamp = ros::Time();
  g_fs_last_detection_odom_stamp = ros::Time();
  g_fs_history.clear();

  pravite_nh.param("imu_topic", imu_topic_, std::string("/imu"));
  pravite_nh.param("imu_frame", imu_frame_id_, std::string("imu")); 
  pravite_nh.param("mag_pose_2d_topic", mag_pose_2d_topic_, std::string("/mag_pose_2d"));
  read_first_ = false;
  imu_frist_sn_ = false;
  controll_type_ = MOTOR_MODE_CMD; // 1:vel_mode 0:joy_node
  linear_gain_   = 0.3;
  twist_gain_    = 0.7;
  linear_speed_min_  = 0;
  angular_speed_min_ = 0;
  current_battery_percent_ = -1;
  led_mode_type_   = 0;
  led_frequency_   = 0;
  led_red_value_   = 0;
  led_green_value_ = 0;
  led_blue_value_  = 0;

  getMileage();

  odom_pub_    = nh_.advertise<nav_msgs::Odometry>(odom_topic_.c_str(),10);
  battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>(battery_topic_.c_str(),10);
  vel_sub_     = nh_.subscribe<geometry_msgs::Twist>(vel_topic_.c_str(), 1, &baseBringup::velCallback,this);
  joy_sub_     = nh_.subscribe<sensor_msgs::Joy>(joy_topic_.c_str(),     1, &baseBringup::joyCallback,this);
  imu_pub_     = nh_.advertise<sensor_msgs::Imu>(imu_topic_.c_str(), 10);
  mag_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(mag_pose_2d_topic_.c_str(), 10);


  // Actual body command selected by writeLoop immediately before packet conversion.
  g_applied_cmd_pub = nh_.advertise<geometry_msgs::Twist>("/base_driver/applied_cmd_vel", 20);
  // Numeric per-TX diagnostics. See writeLoop field definition below.
  g_tx_diag_pub = nh_.advertise<std_msgs::Float64MultiArray>("/base_driver/tx_diag", 50);
  g_fs_active_pub = nh_.advertise<std_msgs::Bool>("/base_driver/failsafe_active", 10, true);
  publishFailsafeState(false);

  stop_move_server_   = nh_.advertiseService("stop_move", &baseBringup::stopMoveCB, this);
  set_max_vel_server_ = nh_.advertiseService("set_max_vel", &baseBringup::setMaxVelCB, this);
  get_max_vel_server_ = nh_.advertiseService("get_max_vel", &baseBringup::getMaxVelCB, this);
  get_battery_state_server_ = nh_.advertiseService("get_battery_state", &baseBringup::getBatteryStateCB, this);
  set_led_server_     = nh_.advertiseService("set_led_light", &baseBringup::setLEDCallBack, this);
  
  try
  {
    serial_.setPort(port_); 
    serial_.setBaudrate(baud_); 
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);//default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_); 
    serial_.setTimeout(time_out); 
    serial_.open(); 
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port "); 
    exit(0); 
  }
  if(serial_.isOpen()) 
  { 
    ROS_INFO_STREAM("Serial Port initialized"); 
  }else{ 
    ROS_ERROR_STREAM("Unable to initial Serial port "); 
    exit(0);
  } 
  current_time_ = ros::Time::now();
  last_time_    = ros::Time::now();
  setSerial();
  openSerial();
  writeThread_   = new boost::thread(boost::bind(&baseBringup::writeLoop,   this));
  processThread_ = new boost::thread(boost::bind(&baseBringup::processLoop, this));

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ROS_INFO("ucarController Ready!");
  ros::waitForShutdown();
}

void baseBringup::setSerial()
{
  try
  {
    serial_.setPort(port_); 
    serial_.setBaudrate(baud_); 
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);//default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_); 
    serial_.setTimeout(time_out); 
    // serial_.open(); 
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("AIcarController setSerial failed, try again!");
    ROS_ERROR("AIcarController setSerial: %s", e.what());
    setSerial();
  }
  catch (...)
  {
    ROS_ERROR("AIcarController setSerial failed with unknow reason, try again!");
    setSerial();
  }
  ros::Duration(cmd_dt_threshold_).sleep();
}

void baseBringup::openSerial()
{
  bool first_open = true;
	while(ros::ok())
  {
    try
    {
      if(serial_.isOpen()==1)
      {
        ROS_INFO("AIcarController serial port open success.\n");
        return;
      }
      else
      {
        if (first_open){
          ROS_INFO("AIcarController openSerial: start open serial port\n");
        } 
        serial_.open();
      }
    }
    catch(const std::exception& e)
    {
      if (first_open)
      {
        ROS_ERROR("AIcarController openSerial: %s", e.what());
        ROS_ERROR("AIcarController openSerial: unable to open port, keep trying\n");
        // std::cerr << e.what() << '\n';
      }
    }
    catch(...)
    {
      if (first_open)
      {
        ROS_ERROR("AIcarController openSerial: unable to open port with unknow reason, keep trying\n");
      }
    }
    first_open = false;
    ros::Duration(cmd_dt_threshold_).sleep();
  }
}//openSerial

void baseBringup::writeLoop()
{
  ROS_INFO("baseBringup::writeLoop: start");
  led_timer = 0;
  ros::Rate loop_rate(rate_);

  while(ros::ok())
  {
    try
    {
      boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_);
      const int cur_controll_type = controll_type_;
      lock.unlock();

      double linear_x = 0.0;
      double linear_y = 0.0;
      double angular_z = 0.0;

      // Cached command before timeout substitution. Used only to decide whether a
      // timeout was dangerous (non-zero motion command) or harmless (already zero).
      double cached_cmd_x = 0.0;
      double cached_cmd_y = 0.0;
      double cached_cmd_w = 0.0;

      uint64_t tx_selected_seq = 0;
      uint64_t rx_seq_at_write = 0;
      double cmd_age_ms = -1.0;
      bool cmd_timed_out = false;
      bool cmd_mode = false;

      switch (cur_controll_type)
      {
        case MOTOR_MODE_JOY:
        {
          lock.lock();
          linear_x  = joy_linear_x_;
          linear_y  = joy_linear_y_;
          angular_z = joy_angular_z_;
          lock.unlock();
          break;
        }

        case MOTOR_MODE_CMD:
        {
          cmd_mode = true;
          lock.lock();

          tx_selected_seq = g_cmd_rx_seq;
          cached_cmd_x = cmd_linear_x_;
          cached_cmd_y = cmd_linear_y_;
          cached_cmd_w = cmd_angular_z_;

          if (tx_selected_seq != 0)
          {
            const double dt = (ros::Time::now() - last_cmd_time_).toSec();
            cmd_age_ms = dt * 1000.0;

            if (dt > cmd_dt_threshold_)
            {
              linear_x  = 0.0;
              linear_y  = 0.0;
              angular_z = 0.0;
              cmd_timed_out = true;
            }
            else
            {
              linear_x  = cached_cmd_x;
              linear_y  = cached_cmd_y;
              angular_z = cached_cmd_w;
            }
          }
          else
          {
            // No /cmd_vel has ever been received. Preserve original safe behavior:
            // last_cmd_time_ is effectively old, so output zero.
            linear_x = 0.0;
            linear_y = 0.0;
            angular_z = 0.0;
            cmd_timed_out = true;
          }

          lock.unlock();
          break;
        }

        case MOTOR_MODE_MOVE:
        {
          lock.lock();
          linear_x  = move_linear_x_;
          linear_y  = move_linear_y_;
          angular_z = move_angular_z_;
          lock.unlock();
          break;
        }

        default:
          ROS_ERROR("base_driver-writeLoop: controll_type_ error!");
          break;
      }

      // ==================== Original limits ====================
      if(linear_x > linear_speed_max_)
        linear_x = linear_speed_max_;
      else if(linear_x < -linear_speed_max_)
        linear_x = -linear_speed_max_;

      if(linear_y > linear_speed_max_)
        linear_y = linear_speed_max_;
      else if(linear_y < -linear_speed_max_)
        linear_y = -linear_speed_max_;

      if(angular_z > angular_speed_max_)
        angular_z = angular_speed_max_;
      else if(angular_z < -angular_speed_max_)
        angular_z = -angular_speed_max_;

      // ==================== BASE-FAILSAFE V4.2 ====================
      // Snapshot the newest wheel-odometry body velocity reported by the MCU.
      nav_msgs::Odometry fs_odom;
      lock.lock();
      fs_odom = current_odom_;
      lock.unlock();

      const ros::WallTime fs_now = ros::WallTime::now();
      const ros::Time fs_ros_now = ros::Time::now();
      const double fs_odom_x = fs_odom.twist.twist.linear.x;
      const double fs_odom_y = fs_odom.twist.twist.linear.y;
      const double fs_odom_w = fs_odom.twist.twist.angular.z;

      double fs_odom_age_ms = -1.0;
      bool fs_odom_valid = !fs_odom.header.stamp.isZero();
      if (fs_odom_valid)
      {
        fs_odom_age_ms = (fs_ros_now - fs_odom.header.stamp).toSec() * 1000.0;
        if (fs_odom_age_ms < 0.0 || fs_odom_age_ms > g_fs_odom_max_age_ms)
          fs_odom_valid = false;
      }

      const double fs_grace_age_ms =
          (g_fs_last_recover_wall.toSec() > 0.0)
              ? (fs_now - g_fs_last_recover_wall).toSec() * 1000.0
              : 1e9;
      const bool fs_in_recovery_grace = fs_grace_age_ms < g_fs_recovery_grace_ms;

      const char* fs_trigger_reason = NULL;
      bool fs_just_recovered_this_tx = false;

      // V4 diagnostics.
      double fs_eval_span_ms = -1.0;
      double fs_cmd_span_linear = -1.0;
      double fs_cmd_span_angular = -1.0;
      double fs_best_delay_ms = -1.0;
      double fs_best_delay_score = -1.0;
      double fs_best_normal_delay_ms = -1.0;
      double fs_best_normal_score = -1.0;
      double fs_delay_improvement = -1.0;
      int fs_eval_sample_count = 0;
      bool fs_delay_fault_candidate = false;
      bool fs_fast_delay_candidate = false;
      double fs_current_mismatch_linear = -1.0;
      double fs_current_mismatch_angular = -1.0;

      // Keep a command/odom history in NORMAL state.  tx_stamp represents the
      // actual command selected by writeLoop, i.e. immediately before serial TX.
      if (g_fs_enable && g_fs_state == FS_NORMAL && cmd_mode && !cmd_timed_out &&
          tx_selected_seq != 0 && fs_odom_valid)
      {
        FailsafeSample fs_sample;
        fs_sample.wall_time = fs_now;
        fs_sample.tx_stamp = fs_ros_now;
        fs_sample.odom_stamp = fs_odom.header.stamp;
        fs_sample.cmd_x = linear_x;
        fs_sample.cmd_y = linear_y;
        fs_sample.cmd_w = angular_z;
        fs_sample.odom_x = fs_odom_x;
        fs_sample.odom_y = fs_odom_y;
        fs_sample.odom_w = fs_odom_w;
        g_fs_history.push_back(fs_sample);

        while (!g_fs_history.empty() &&
               (fs_now - g_fs_history.front().wall_time).toSec() * 1000.0 > g_fs_history_keep_ms)
          g_fs_history.pop_front();
      }
      else if (g_fs_state == FS_NORMAL)
      {
        g_fs_history.clear();
        resetFailsafeDetectionState();
      }

      if (g_fs_enable && g_fs_state == FS_NORMAL && cmd_mode && !cmd_timed_out &&
          tx_selected_seq != 0 && fs_odom_valid && !fs_in_recovery_grace)
      {
        // -------------------------------------------------------------------
        // Channel A: persistent high-confidence angular sign disagreement.
        // -------------------------------------------------------------------
        const bool fs_reverse_condition =
            fabs(angular_z) >= g_fs_reverse_cmd_w_min &&
            fabs(fs_odom_w) >= g_fs_reverse_odom_w_min &&
            (angular_z * fs_odom_w) < 0.0;

        if (fs_reverse_condition)
          ++g_fs_reverse_count;
        else
          g_fs_reverse_count = 0;

        if (g_fs_reverse_count >= g_fs_reverse_confirm_cycles)
          fs_trigger_reason = "ANGULAR_REVERSE_MISMATCH";

        // -------------------------------------------------------------------
        // Channel B: rolling trajectory delay estimation.
        //
        // For each candidate delay D, compare recent odom(t) with the command
        // actually selected for TX near (t-D).  A stale/delayed actuator fault
        // should produce a much smaller trajectory error for a large D than for
        // every normal D (<= g_fs_delay_normal_max_ms).
        // -------------------------------------------------------------------
        if (g_fs_history.size() >= static_cast<size_t>(g_fs_delay_min_eval_odom_samples))
        {
          // Command excitation over the recent evaluation interval.
          std::deque<FailsafeSample>::const_iterator eval_begin = g_fs_history.end();
          for (std::deque<FailsafeSample>::const_iterator it = g_fs_history.begin();
               it != g_fs_history.end(); ++it)
          {
            const double age_ms = (fs_now - it->wall_time).toSec() * 1000.0;
            if (age_ms <= g_fs_delay_eval_window_ms)
            {
              eval_begin = it;
              break;
            }
          }

          if (eval_begin != g_fs_history.end())
          {
            fs_eval_span_ms = (fs_now - eval_begin->wall_time).toSec() * 1000.0;
            fs_cmd_span_linear = 0.0;
            fs_cmd_span_angular = 0.0;

            for (std::deque<FailsafeSample>::const_iterator a = eval_begin;
                 a != g_fs_history.end(); ++a)
            {
              std::deque<FailsafeSample>::const_iterator b = a;
              ++b;
              for (; b != g_fs_history.end(); ++b)
              {
                const double dl = fsLinearDiff(a->cmd_x, a->cmd_y, b->cmd_x, b->cmd_y);
                const double dw = fabs(a->cmd_w - b->cmd_w);
                if (dl > fs_cmd_span_linear) fs_cmd_span_linear = dl;
                if (dw > fs_cmd_span_angular) fs_cmd_span_angular = dw;
              }
            }

            const bool cmd_excited =
                fs_cmd_span_linear >= g_fs_delay_cmd_span_linear_min ||
                fs_cmd_span_angular >= g_fs_delay_cmd_span_angular_min;

            const double current_cmd_linear = sqrt(linear_x * linear_x + linear_y * linear_y);
            const double current_odom_linear = sqrt(fs_odom_x * fs_odom_x + fs_odom_y * fs_odom_y);
            const bool meaningful_motion =
                current_cmd_linear >= g_fs_detection_min_linear ||
                current_odom_linear >= g_fs_detection_min_linear ||
                fabs(angular_z) >= g_fs_detection_min_angular ||
                fabs(fs_odom_w) >= g_fs_detection_min_angular;

            fs_current_mismatch_linear =
                fsLinearDiff(linear_x, linear_y, fs_odom_x, fs_odom_y);
            fs_current_mismatch_angular = fabs(angular_z - fs_odom_w);
            const bool current_mismatch =
                fs_current_mismatch_linear >= g_fs_current_mismatch_linear ||
                fs_current_mismatch_angular >= g_fs_current_mismatch_angular;

            if (cmd_excited && meaningful_motion && current_mismatch &&
                fs_eval_span_ms >= g_fs_delay_eval_window_ms * 0.75)
            {
              double best_score = 1e9;
              double best_delay = -1.0;
              int best_count = 0;
              double best_normal_score = 1e9;
              double best_normal_delay = -1.0;

              const double delay_step = g_fs_delay_step_ms > 1.0 ? g_fs_delay_step_ms : 50.0;
              for (double delay_ms = g_fs_delay_min_ms;
                   delay_ms <= g_fs_delay_max_ms + 1e-6;
                   delay_ms += delay_step)
              {
                double score_sum = 0.0;
                int score_count = 0;
                ros::Time last_eval_odom_stamp;

                for (std::deque<FailsafeSample>::const_iterator s = eval_begin;
                     s != g_fs_history.end(); ++s)
                {
                  if (s->odom_stamp.isZero())
                    continue;

                  // writeLoop can observe the same 20 Hz odom sample more than
                  // once; count each physical odom frame only once.
                  if (!last_eval_odom_stamp.isZero() && s->odom_stamp == last_eval_odom_stamp)
                    continue;
                  last_eval_odom_stamp = s->odom_stamp;

                  const ros::Time target_tx =
                      s->odom_stamp - ros::Duration(delay_ms / 1000.0);

                  double nearest_dt_ms = 1e9;
                  std::deque<FailsafeSample>::const_iterator nearest = g_fs_history.end();
                  for (std::deque<FailsafeSample>::const_iterator c = g_fs_history.begin();
                       c != g_fs_history.end(); ++c)
                  {
                    const double dt_ms = fabs((c->tx_stamp - target_tx).toSec()) * 1000.0;
                    if (dt_ms < nearest_dt_ms)
                    {
                      nearest_dt_ms = dt_ms;
                      nearest = c;
                    }
                  }

                  if (nearest == g_fs_history.end() ||
                      nearest_dt_ms > g_fs_delay_match_tolerance_ms)
                    continue;

                  score_sum += fsDelaySampleScore(nearest->cmd_x,
                                                  nearest->cmd_y,
                                                  nearest->cmd_w,
                                                  s->odom_x,
                                                  s->odom_y,
                                                  s->odom_w);
                  ++score_count;
                }

                if (score_count >= g_fs_delay_min_eval_odom_samples)
                {
                  const double avg_score = score_sum / static_cast<double>(score_count);
                  if (avg_score < best_score)
                  {
                    best_score = avg_score;
                    best_delay = delay_ms;
                    best_count = score_count;
                  }

                  if (delay_ms <= g_fs_delay_normal_max_ms && avg_score < best_normal_score)
                  {
                    best_normal_score = avg_score;
                    best_normal_delay = delay_ms;
                  }
                }
              }

              if (best_score < 1e8)
              {
                fs_best_delay_ms = best_delay;
                fs_best_delay_score = best_score;
                fs_eval_sample_count = best_count;
              }
              if (best_normal_score < 1e8)
              {
                fs_best_normal_delay_ms = best_normal_delay;
                fs_best_normal_score = best_normal_score;
              }

              if (best_score < 1e8 && best_normal_score < 1e8)
              {
                fs_delay_improvement = best_normal_score - best_score;
                fs_delay_fault_candidate =
                    best_delay >= g_fs_delay_fault_min_ms &&
                    best_score <= g_fs_delay_fault_score_max &&
                    best_score <= best_normal_score * g_fs_delay_vs_normal_ratio_max &&
                    fs_delay_improvement >= g_fs_delay_improvement_min;

                // FAST_DELAY is stricter about instantaneous mismatch, but does
                // not wait for a second odom confirmation.
                fs_fast_delay_candidate =
                    g_fs_fast_delay_enable &&
                    best_delay >= g_fs_fast_delay_min_ms &&
                    best_score <= g_fs_fast_delay_score_max &&
                    best_score <= best_normal_score * g_fs_fast_delay_ratio_max &&
                    fs_delay_improvement >= g_fs_fast_delay_improvement_min &&
                    (fs_current_mismatch_linear >= g_fs_fast_mismatch_linear ||
                     fs_current_mismatch_angular >= g_fs_fast_mismatch_angular);
              }
            }
          }
        }

        // Delay fault confirmation counts only NEW odom frames.
        bool fs_new_detection_odom = false;
        if (fs_odom.header.stamp != g_fs_last_detection_odom_stamp)
        {
          fs_new_detection_odom = true;
          g_fs_last_detection_odom_stamp = fs_odom.header.stamp;

          if (fs_delay_fault_candidate)
            ++g_fs_delay_confirm_count;
          else
            g_fs_delay_confirm_count = 0;
        }

        // High-confidence path: one fresh odom frame is enough.
        if (fs_trigger_reason == NULL &&
            fs_new_detection_odom &&
            fs_fast_delay_candidate)
          fs_trigger_reason = "FAST_ABNORMAL_COMMAND_TO_ODOM_DELAY";

        // Medium-confidence path: retain two-frame confirmation.
        if (fs_trigger_reason == NULL &&
            g_fs_delay_confirm_count >= g_fs_delay_confirm_odom_frames)
          fs_trigger_reason = "ABNORMAL_COMMAND_TO_ODOM_DELAY";
      }
      else if (g_fs_state == FS_NORMAL)
      {
        g_fs_reverse_count = 0;
        g_fs_delay_confirm_count = 0;
        g_fs_last_detection_odom_stamp = ros::Time();
      }

      if (fs_trigger_reason != NULL && g_fs_state == FS_NORMAL)
      {
        g_fs_state = FS_EMERGENCY_BRAKE;
        g_fs_trigger_wall = fs_now;
        g_fs_hold_start_wall = ros::WallTime();
        g_fs_stop_confirm_count = 0;
        g_fs_last_stop_check_odom_stamp = ros::Time();
        ++g_fs_trigger_count;
        publishFailsafeState(true);

        ROS_ERROR("[BASE-FAILSAFE][TRIGGER] reason=%s count=%llu cmd=(%.3f,%.3f,%.3f) odom=(%.3f,%.3f,%.3f) odom_age_ms=%.2f reverse_count=%d delay_count=%d fast_delay=%d best_delay_ms=%.1f best_score=%.3f normal_delay_ms=%.1f normal_score=%.3f improvement=%.3f mismatch=(%.3f,%.3f) eval_n=%d cmd_span=(%.3f,%.3f)",
                  fs_trigger_reason,
                  static_cast<unsigned long long>(g_fs_trigger_count),
                  linear_x, linear_y, angular_z,
                  fs_odom_x, fs_odom_y, fs_odom_w,
                  fs_odom_age_ms,
                  g_fs_reverse_count,
                  g_fs_delay_confirm_count,
                  fs_fast_delay_candidate ? 1 : 0,
                  fs_best_delay_ms,
                  fs_best_delay_score,
                  fs_best_normal_delay_ms,
                  fs_best_normal_score,
                  fs_delay_improvement,
                  fs_current_mismatch_linear,
                  fs_current_mismatch_angular,
                  fs_eval_sample_count,
                  fs_cmd_span_linear,
                  fs_cmd_span_angular);

        // The triggering TX cycle is already forced to zero.
        linear_x = 0.0;
        linear_y = 0.0;
        angular_z = 0.0;
        g_fs_history.clear();
      }

      // Once active, base_driver owns output and ignores all motion commands.
      if (g_fs_state != FS_NORMAL)
      {
        linear_x = 0.0;
        linear_y = 0.0;
        angular_z = 0.0;

        // Count STOP confirmations only on NEW odometry frames.  Emergency TX
        // runs at 100 Hz while odometry is normally around 20 Hz.
        if (fs_odom_valid && fs_odom.header.stamp != g_fs_last_stop_check_odom_stamp)
        {
          g_fs_last_stop_check_odom_stamp = fs_odom.header.stamp;
          const double odom_linear_speed = sqrt(fs_odom_x * fs_odom_x + fs_odom_y * fs_odom_y);
          const bool odom_stopped =
              odom_linear_speed <= g_fs_stop_linear &&
              fabs(fs_odom_w) <= g_fs_stop_angular;

          if (odom_stopped)
            ++g_fs_stop_confirm_count;
          else
            g_fs_stop_confirm_count = 0;
        }

        if (g_fs_state == FS_EMERGENCY_BRAKE)
        {
          const double brake_ms = (fs_now - g_fs_trigger_wall).toSec() * 1000.0;
          if (brake_ms >= g_fs_min_brake_ms &&
              g_fs_stop_confirm_count >= g_fs_stop_confirm_odom_frames)
          {
            g_fs_state = FS_HOLD_STOP;
            g_fs_hold_start_wall = fs_now;
            ROS_WARN("[BASE-FAILSAFE][STOP_CONFIRMED] brake_ms=%.1f odom=(%.3f,%.3f,%.3f) confirm_frames=%d; entering HOLD_STOP",
                     brake_ms,
                     fs_odom_x, fs_odom_y, fs_odom_w,
                     g_fs_stop_confirm_count);
          }
        }
        else if (g_fs_state == FS_HOLD_STOP)
        {
          const double hold_ms = (fs_now - g_fs_hold_start_wall).toSec() * 1000.0;
          if (hold_ms >= g_fs_hold_ms)
          {
            g_fs_state = FS_NORMAL;
            g_fs_last_recover_wall = fs_now;
            g_fs_stop_confirm_count = 0;
            g_fs_history.clear();
            resetFailsafeDetectionState();

            // Prevent the normal 20 Hz loop from immediately "catching up"
            // after the temporary 100 Hz emergency loop.
            g_last_tx_selected_seq = 0;
            g_repeat_streak = 0;
            g_last_tx_wall_time = ros::WallTime();
            fs_just_recovered_this_tx = true;

            publishFailsafeState(false);
            ROS_WARN("[BASE-FAILSAFE][RECOVER] hold_ms=%.1f; returning to NORMAL with %.1fms detection grace",
                     hold_ms,
                     g_fs_recovery_grace_ms);
          }
        }
      }

      const bool failsafe_active_this_tx = (g_fs_state != FS_NORMAL);

      const double vw1 = linear_x - linear_y - angular_z * (base_shape_a_ + base_shape_b_);
      const double vw2 = linear_x + linear_y + angular_z * (base_shape_a_ + base_shape_b_);
      const double vw3 = linear_x - linear_y + angular_z * (base_shape_a_ + base_shape_b_);
      const double vw4 = linear_x + linear_y - angular_z * (base_shape_a_ + base_shape_b_);

      lock.lock();
      pack_write_.write_tmp[0] = 0x63;
      pack_write_.write_tmp[1] = 0x75;
      pack_write_.pack.ver     = 0;
      pack_write_.pack.len     = 11;
      pack_write_.pack.data.pluse_w1 = -period_/1000.0*vw1*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w2 =  period_/1000.0*vw2*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w3 = -period_/1000.0*vw4*(encode_resolution_/(2.0*Pi*wheel_radius_));
      pack_write_.pack.data.pluse_w4 =  period_/1000.0*vw3*(encode_resolution_/(2.0*Pi*wheel_radius_));

      const int cur_led_mode = led_mode_type_;
      ++led_timer;
      lock.unlock();

      switch (cur_led_mode)
      {
        case ucar_controller::SetLEDMode::Request::MODE_NORMAL:
        {
          lock.lock();
          pack_write_.pack.red_value   = (int)led_red_value_;
          pack_write_.pack.green_value = (int)led_green_value_;
          pack_write_.pack.blue_value  = (int)led_blue_value_;
          lock.unlock();
          break;
        }

        case ucar_controller::SetLEDMode::Request::MODE_BLINK:
        {
          const double t = (double)led_timer/(double)rate_;
          lock.lock();
          const double f = led_frequency_;
          const int blink = (int)(2.0*t*f)%2;
          pack_write_.pack.red_value   = led_red_value_   * blink;
          pack_write_.pack.green_value = led_green_value_ * blink;
          pack_write_.pack.blue_value  = led_blue_value_  * blink;
          lock.unlock();
          break;
        }

        case ucar_controller::SetLEDMode::Request::MODE_BREATH:
        {
          const double t = ros::Time::now().toSec();
          lock.lock();
          const double w = 2 * Pi * led_frequency_;
          pack_write_.pack.red_value   = 0.5 * (led_red_value_   + led_red_value_   * sin(w * t));
          pack_write_.pack.green_value = 0.5 * (led_green_value_ + led_green_value_ * sin(w * t));
          pack_write_.pack.blue_value  = 0.5 * (led_blue_value_  + led_blue_value_  * sin(w * t));
          lock.unlock();
          break;
        }

        default:
        {
          lock.lock();
          pack_write_.pack.red_value   = (int)led_red_value_;
          pack_write_.pack.green_value = (int)led_green_value_;
          pack_write_.pack.blue_value  = (int)led_blue_value_;
          lock.unlock();
          break;
        }
      }

      // ==================== Critical TX observation point ====================
      // Keep serial.write under the original Control_mutex_ on purpose. V2 is a
      // diagnostic build, not a behavioral fix.
      lock.lock();

      rx_seq_at_write = g_cmd_rx_seq;
      const bool newer_rx_pending =
          cmd_mode && tx_selected_seq != 0 && rx_seq_at_write > tx_selected_seq;
      const uint64_t seq_lag = newer_rx_pending ? (rx_seq_at_write - tx_selected_seq) : 0;

      const double rx_lock_wait_ms_snapshot = g_last_rx_lock_wait_ms;
      const double rx_interval_ms_snapshot = g_last_rx_interval_ms;

      // Repeat streak: number of EXTRA consecutive transmissions using same RX.
      // TX 100,100,100 => streak values 0,1,2.
      uint64_t repeat_streak_this_tx = 0;
      bool repeated_same_seq = false;
      if (cmd_mode && tx_selected_seq != 0 && !cmd_timed_out && !failsafe_active_this_tx)
      {
        repeated_same_seq = (tx_selected_seq == g_last_tx_selected_seq);
        if (repeated_same_seq)
        {
          ++g_repeat_tx_count;
          ++g_repeat_streak;
        }
        else
        {
          g_repeat_streak = 0;
        }
        repeat_streak_this_tx = g_repeat_streak;
        if (g_repeat_streak > g_max_repeat_streak)
          g_max_repeat_streak = g_repeat_streak;
      }
      else
      {
        g_repeat_streak = 0;
      }

      // Commands skipped between two distinct selected RX sequence numbers.
      uint64_t skipped_this_tx = 0;
      if (cmd_mode && tx_selected_seq != 0 && g_last_tx_selected_seq != 0 &&
          tx_selected_seq > g_last_tx_selected_seq + 1)
      {
        skipped_this_tx = tx_selected_seq - g_last_tx_selected_seq - 1;
        g_skipped_rx_total += skipped_this_tx;
        ++g_skip_event_count;
      }

      const bool cached_cmd_nonzero =
          (sqrt(cached_cmd_x * cached_cmd_x + cached_cmd_y * cached_cmd_y) > g_diag_nonzero_linear_eps) ||
          (fabs(cached_cmd_w) > g_diag_nonzero_angular_eps);

      const bool timeout_edge =
          cmd_mode && tx_selected_seq != 0 && cmd_timed_out && !g_last_tx_was_timeout;
      const bool nonzero_timeout_edge = timeout_edge && cached_cmd_nonzero;

      setWriteCS(WRITE_MSG_LONGTH);

      const ros::WallTime write_start = ros::WallTime::now();
      double tx_gap_ms = 0.0;
      if (g_last_tx_wall_time.toSec() > 0.0)
        tx_gap_ms = (write_start - g_last_tx_wall_time).toSec() * 1000.0;

      const size_t pack_write_s = serial_.write(pack_write_.write_tmp, WRITE_MSG_LONGTH);
      const ros::WallTime write_end = ros::WallTime::now();
      const double write_ms = (write_end - write_start).toSec() * 1000.0;

      if (pack_write_s != WRITE_MSG_LONGTH)
        ++g_short_write_count;
      if (nonzero_timeout_edge)
        ++g_nonzero_timeout_count;
      if (newer_rx_pending)
        ++g_stale_before_write_count;

      if (cmd_mode)
      {
        g_last_tx_selected_seq = tx_selected_seq;
        g_last_tx_was_timeout = cmd_timed_out;
      }
      else
      {
        g_last_tx_was_timeout = false;
      }
      g_last_tx_wall_time = write_end;

      lock.unlock();

      if (g_diag_enable)
      {
        geometry_msgs::Twist applied_cmd;
        applied_cmd.linear.x = linear_x;
        applied_cmd.linear.y = linear_y;
        applied_cmd.angular.z = angular_z;
        if (g_base_diag_enable) g_applied_cmd_pub.publish(applied_cmd);

        // /base_driver/tx_diag fields:
        // [0]  rx_seq_at_write
        // [1]  tx_selected_seq
        // [2]  cmd_age_ms (-1 before first RX)
        // [3]  serial_write_ms
        // [4]  serial_write_bytes
        // [5]  repeat_streak (extra repeats; 0,1,2,...)
        // [6]  command_timeout
        // [7]  tx_gap_ms
        // [8]  last_rx_lock_wait_ms
        // [9]  last_rx_interval_ms
        // [10] newer_rx_pending
        // [11] seq_lag
        // [12] skipped_this_tx
        // [13] skipped_rx_total
        // [14] repeat_tx_total
        // [15] max_repeat_streak
        // [16] cached_cmd_nonzero
        // [17] nonzero_timeout_total
        // [18] failsafe_state: 0 NORMAL, 1 BRAKE, 2 HOLD
        // [19] failsafe_reverse_count
        // [20] abnormal-delay confirmation count
        // [21] failsafe_trigger_total
        // [22] odom_age_ms
        // [23] measured_odom_vx
        // [24] measured_odom_vy
        // [25] measured_odom_wz
        // [26..29] reserved compatibility slots
        // [30] delay_eval_window_span_ms
        // [31] command_linear_span_mps
        // [32] command_angular_span_radps
        // [33] best_delay_ms
        // [34] best_delay_score
        // [35] best_normal_delay_ms
        // [36] best_normal_delay_score
        // [37] delay_fault_candidate
        // [38] delay_eval_odom_sample_count
        // [39] current_linear_mismatch_mps
        // [40] current_angular_mismatch_radps
        // [41] normal_score_minus_best_score
        // [42] fast_delay_candidate
        std_msgs::Float64MultiArray diag_msg;
        diag_msg.data.resize(43);
        diag_msg.data[0]  = static_cast<double>(rx_seq_at_write);
        diag_msg.data[1]  = static_cast<double>(tx_selected_seq);
        diag_msg.data[2]  = cmd_age_ms;
        diag_msg.data[3]  = write_ms;
        diag_msg.data[4]  = static_cast<double>(pack_write_s);
        diag_msg.data[5]  = static_cast<double>(repeat_streak_this_tx);
        diag_msg.data[6]  = cmd_timed_out ? 1.0 : 0.0;
        diag_msg.data[7]  = tx_gap_ms;
        diag_msg.data[8]  = rx_lock_wait_ms_snapshot;
        diag_msg.data[9]  = rx_interval_ms_snapshot;
        diag_msg.data[10] = newer_rx_pending ? 1.0 : 0.0;
        diag_msg.data[11] = static_cast<double>(seq_lag);
        diag_msg.data[12] = static_cast<double>(skipped_this_tx);
        diag_msg.data[13] = static_cast<double>(g_skipped_rx_total);
        diag_msg.data[14] = static_cast<double>(g_repeat_tx_count);
        diag_msg.data[15] = static_cast<double>(g_max_repeat_streak);
        diag_msg.data[16] = cached_cmd_nonzero ? 1.0 : 0.0;
        diag_msg.data[17] = static_cast<double>(g_nonzero_timeout_count);
        diag_msg.data[18] = static_cast<double>(g_fs_state);
        diag_msg.data[19] = static_cast<double>(g_fs_reverse_count);
        diag_msg.data[20] = static_cast<double>(g_fs_delay_confirm_count);
        diag_msg.data[21] = static_cast<double>(g_fs_trigger_count);
        diag_msg.data[22] = fs_odom_age_ms;
        diag_msg.data[23] = fs_odom_x;
        diag_msg.data[24] = fs_odom_y;
        diag_msg.data[25] = fs_odom_w;
        diag_msg.data[26] = 0.0;
        diag_msg.data[27] = -1.0;
        diag_msg.data[28] = -1.0;
        diag_msg.data[29] = -1.0;
        diag_msg.data[30] = fs_eval_span_ms;
        diag_msg.data[31] = fs_cmd_span_linear;
        diag_msg.data[32] = fs_cmd_span_angular;
        diag_msg.data[33] = fs_best_delay_ms;
        diag_msg.data[34] = fs_best_delay_score;
        diag_msg.data[35] = fs_best_normal_delay_ms;
        diag_msg.data[36] = fs_best_normal_score;
        diag_msg.data[37] = fs_delay_fault_candidate ? 1.0 : 0.0;
        diag_msg.data[38] = static_cast<double>(fs_eval_sample_count);
        diag_msg.data[39] = fs_current_mismatch_linear;
        diag_msg.data[40] = fs_current_mismatch_angular;
        diag_msg.data[41] = fs_delay_improvement;
        diag_msg.data[42] = fs_fast_delay_candidate ? 1.0 : 0.0;
        if (g_base_diag_enable) g_tx_diag_pub.publish(diag_msg);

        if (pack_write_s != WRITE_MSG_LONGTH)
        {
          if (g_base_diag_enable) ROS_ERROR("[BASE-DIAG][SHORT_WRITE] selected_rx=%llu bytes=%lu/%d write_ms=%.3f age_ms=%.3f",
                    static_cast<unsigned long long>(tx_selected_seq),
                    static_cast<unsigned long>(pack_write_s),
                    WRITE_MSG_LONGTH,
                    write_ms,
                    cmd_age_ms);
        }

        if (write_ms > g_diag_write_warn_ms)
        {
          if (g_base_diag_enable) ROS_WARN("[BASE-DIAG][SLOW_WRITE] selected_rx=%llu write_ms=%.3f threshold_ms=%.3f bytes=%lu/%d",
                   static_cast<unsigned long long>(tx_selected_seq),
                   write_ms,
                   g_diag_write_warn_ms,
                   static_cast<unsigned long>(pack_write_s),
                   WRITE_MSG_LONGTH);
        }

        // Do NOT warn for the first one-cycle repeat. Normal 20Hz phase jitter
        // produces these frequently. Warn only at the configured streak.
        if (repeat_streak_this_tx == static_cast<uint64_t>(g_diag_repeat_warn_streak))
        {
          if (g_base_diag_enable) ROS_WARN("[BASE-DIAG][REPEAT_STREAK] rx=%llu repeat_streak=%llu age_ms=%.3f cmd=(%.3f,%.3f,%.3f) tx_gap_ms=%.3f",
                   static_cast<unsigned long long>(tx_selected_seq),
                   static_cast<unsigned long long>(repeat_streak_this_tx),
                   cmd_age_ms,
                   linear_x, linear_y, angular_z,
                   tx_gap_ms);
        }
        else if (repeat_streak_this_tx == static_cast<uint64_t>(g_diag_repeat_error_streak))
        {
          if (g_base_diag_enable) ROS_ERROR("[BASE-DIAG][REPEAT_STUCK] rx=%llu repeat_streak=%llu age_ms=%.3f cmd=(%.3f,%.3f,%.3f) tx_gap_ms=%.3f",
                    static_cast<unsigned long long>(tx_selected_seq),
                    static_cast<unsigned long long>(repeat_streak_this_tx),
                    cmd_age_ms,
                    linear_x, linear_y, angular_z,
                    tx_gap_ms);
        }

        if (newer_rx_pending)
        {
          if (g_base_diag_enable) ROS_WARN("[BASE-DIAG][STALE_BEFORE_WRITE] selected_rx=%llu latest_rx=%llu lag=%llu cmd=(%.3f,%.3f,%.3f)",
                   static_cast<unsigned long long>(tx_selected_seq),
                   static_cast<unsigned long long>(rx_seq_at_write),
                   static_cast<unsigned long long>(seq_lag),
                   linear_x, linear_y, angular_z);
        }

        if (nonzero_timeout_edge)
        {
          if (g_base_diag_enable) ROS_WARN("[BASE-DIAG][NONZERO_TIMEOUT] rx=%llu age_ms=%.3f timeout_ms=%.3f cached_cmd=(%.3f,%.3f,%.3f)",
                   static_cast<unsigned long long>(tx_selected_seq),
                   cmd_age_ms,
                   cmd_dt_threshold_ * 1000.0,
                   cached_cmd_x, cached_cmd_y, cached_cmd_w);
        }

        if (tx_gap_ms > g_diag_tx_gap_warn_ms && tx_selected_seq != 0)
        {
          if (g_base_diag_enable) ROS_WARN("[BASE-DIAG][TX_GAP] tx_gap_ms=%.3f threshold_ms=%.3f selected_rx=%llu",
                   tx_gap_ms,
                   g_diag_tx_gap_warn_ms,
                   static_cast<unsigned long long>(tx_selected_seq));
        }

        // Single skipped RX is expected when a one-cycle repeat is followed by
        // catching up. Only warn on bursts of >= diag_skip_warn_count.
        if (skipped_this_tx >= static_cast<uint64_t>(g_diag_skip_warn_count))
        {
          if (g_base_diag_enable) ROS_WARN("[BASE-DIAG][SKIP_BURST] selected_rx=%llu skipped_now=%llu skipped_total=%llu",
                   static_cast<unsigned long long>(tx_selected_seq),
                   static_cast<unsigned long long>(skipped_this_tx),
                   static_cast<unsigned long long>(g_skipped_rx_total));
        }

        if (g_diag_verbose)
        {
          if (g_base_diag_enable) ROS_INFO("[BASE-DIAG][TX] latest_rx=%llu selected_rx=%llu age_ms=%.3f repeat_streak=%llu skipped=%llu stale=%d lag=%llu timeout=%d write=%lu/%d write_ms=%.3f gap_ms=%.3f",
                   static_cast<unsigned long long>(rx_seq_at_write),
                   static_cast<unsigned long long>(tx_selected_seq),
                   cmd_age_ms,
                   static_cast<unsigned long long>(repeat_streak_this_tx),
                   static_cast<unsigned long long>(skipped_this_tx),
                   newer_rx_pending ? 1 : 0,
                   static_cast<unsigned long long>(seq_lag),
                   cmd_timed_out ? 1 : 0,
                   static_cast<unsigned long>(pack_write_s),
                   WRITE_MSG_LONGTH,
                   write_ms,
                   tx_gap_ms);
        }

        if (tx_selected_seq == 0)
        {
          if (g_base_diag_enable) ROS_INFO_THROTTLE(1.0,
                            "[BASE-DIAG][SUMMARY] latest_rx=0 selected_rx=0 age_ms=NA repeat_total=%llu repeat_streak=0 max_repeat=%llu skipped_total=%llu skip_events=%llu stale=%llu short_write=%llu nonzero_timeout=%llu write_ms=%.2f tx_gap_ms=%.2f",
                            static_cast<unsigned long long>(g_repeat_tx_count),
                            static_cast<unsigned long long>(g_max_repeat_streak),
                            static_cast<unsigned long long>(g_skipped_rx_total),
                            static_cast<unsigned long long>(g_skip_event_count),
                            static_cast<unsigned long long>(g_stale_before_write_count),
                            static_cast<unsigned long long>(g_short_write_count),
                            static_cast<unsigned long long>(g_nonzero_timeout_count),
                            write_ms,
                            tx_gap_ms);
        }
        else
        {
          if (g_base_diag_enable) ROS_INFO_THROTTLE(1.0,
                            "[BASE-DIAG][SUMMARY] latest_rx=%llu selected_rx=%llu age_ms=%.2f repeat_total=%llu repeat_streak=%llu max_repeat=%llu skipped_total=%llu skip_events=%llu stale=%llu short_write=%llu nonzero_timeout=%llu write_ms=%.2f tx_gap_ms=%.2f rx_lock_wait_ms=%.3f rx_interval_ms=%.2f fs_state=%d fs_triggers=%llu fs_rev=%d fs_delay=%d delay_fault=%d fast_delay=%d eval_ms=%.1f cmd_span=(%.2f,%.2f) best_delay_ms=%.1f best_score=%.2f normal_delay_ms=%.1f normal_score=%.2f improve=%.2f mismatch=(%.2f,%.2f) eval_n=%d odom_age_ms=%.1f",
                            static_cast<unsigned long long>(rx_seq_at_write),
                            static_cast<unsigned long long>(tx_selected_seq),
                            cmd_age_ms,
                            static_cast<unsigned long long>(g_repeat_tx_count),
                            static_cast<unsigned long long>(repeat_streak_this_tx),
                            static_cast<unsigned long long>(g_max_repeat_streak),
                            static_cast<unsigned long long>(g_skipped_rx_total),
                            static_cast<unsigned long long>(g_skip_event_count),
                            static_cast<unsigned long long>(g_stale_before_write_count),
                            static_cast<unsigned long long>(g_short_write_count),
                            static_cast<unsigned long long>(g_nonzero_timeout_count),
                            write_ms,
                            tx_gap_ms,
                            rx_lock_wait_ms_snapshot,
                            rx_interval_ms_snapshot,
                            static_cast<int>(g_fs_state),
                            static_cast<unsigned long long>(g_fs_trigger_count),
                            g_fs_reverse_count,
                            g_fs_delay_confirm_count,
                            fs_delay_fault_candidate ? 1 : 0,
                            fs_fast_delay_candidate ? 1 : 0,
                            fs_eval_span_ms,
                            fs_cmd_span_linear,
                            fs_cmd_span_angular,
                            fs_best_delay_ms,
                            fs_best_delay_score,
                            fs_best_normal_delay_ms,
                            fs_best_normal_score,
                            fs_delay_improvement,
                            fs_current_mismatch_linear,
                            fs_current_mismatch_angular,
                            fs_eval_sample_count,
                            fs_odom_age_ms);
        }
      }

      if(debug_log_)
      {
        cout << "write buf:" << endl;
        for (size_t i = 0; i < WRITE_MSG_LONGTH; i++)
          cout << std::hex << (int)pack_write_.write_tmp[i] << " ";
        cout << std::dec << endl;
      }

      if (g_fs_state != FS_NORMAL && g_fs_zero_rate_hz > 0.0)
      {
        ros::WallDuration(1.0 / g_fs_zero_rate_hz).sleep();
      }
      else if (fs_just_recovered_this_tx)
      {
        // Re-establish a clean normal-rate phase after the 100 Hz emergency
        // loop instead of letting the old ros::Rate object "catch up" instantly.
        ros::WallDuration(1.0 / static_cast<double>(rate_)).sleep();
      }
      else
      {
        loop_rate.sleep();
      }
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("AIcarController writeLoop: %s\n", e.what());
      ROS_ERROR("AIcarController writeLoop error, waitfor reopen serial port\n");
      setSerial();
      openSerial();
    }
    catch(...)
    {
      ROS_ERROR("AIcarController writeLoop error, waitfor reopen serial port\n");
      setSerial();
      openSerial();
    }
  }
}

baseBringup::~baseBringup()
{
  if( serial_.isOpen() )   
    serial_.close();
}

bool baseBringup::getBatteryStateCB(ucar_controller::GetBatteryInfo::Request &req,
                                    ucar_controller::GetBatteryInfo::Response &res)
{
  if (current_battery_percent_ == -1)  //初始值为 -1. 即没有获取到电量
  {
    res.battery_state.power_supply_status     = 0; // UNKNOWN
    res.battery_state.power_supply_health     = 0; // UNKNOWN
    res.battery_state.power_supply_technology = 0; // UNKNOWN
    boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
    res.battery_state.percentage = current_battery_percent_;
    lock.unlock();
    return true;
  }
  else
  {
    res.battery_state.power_supply_status     = 2; // 放电中 即正在运行
    res.battery_state.power_supply_health     = 1; // 良好
    res.battery_state.power_supply_technology = 0; // UNKNOWN
    res.battery_state.present    = true;           // 存在电池
    boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
    res.battery_state.percentage = current_battery_percent_; // 电池电量百分比
    lock.unlock(); 
    return true;
  }
}

bool baseBringup::setLEDCallBack(ucar_controller::SetLEDMode::Request &req, 
                                 ucar_controller::SetLEDMode::Response &res)
{
  if (!read_first_)
  {
    res.success = false;
    res.message = "Can't connect base's MCU."; 
    return true;
  }
  else
  {
    boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
    led_mode_type_   = req.mode_type;
    led_frequency_   = req.frequency;
    led_red_value_   = req.red_value;
    led_green_value_ = req.green_value;
    led_blue_value_  = req.blue_value;
    led_t_0 = ros::Time::now().toSec();
    led_timer = 0;
    lock.unlock();
    res.success = true;
    res.message = "Set LED success.";
    return true;
  }
}


bool baseBringup::getMaxVelCB(ucar_controller::GetMaxVel::Request &req, ucar_controller::GetMaxVel::Response &res)
{
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_);
  res.max_linear_velocity  = linear_speed_max_ ;
  res.max_angular_velocity = angular_speed_max_;
  lock.unlock();
  return true;
}
bool baseBringup::setMaxVelCB(ucar_controller::SetMaxVel::Request &req, ucar_controller::SetMaxVel::Response &res)
{
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  linear_speed_max_  = req.max_linear_velocity ;
  angular_speed_max_ = req.max_angular_velocity;
  ros::NodeHandle pravite_nh("~");
  pravite_nh.setParam("linear_speed_max",linear_speed_max_);
  pravite_nh.setParam("angular_speed_max",angular_speed_max_);
  lock.unlock();
  if (linear_speed_max_ == req.max_linear_velocity && angular_speed_max_ == req.max_angular_velocity)
  {
    res.success = true;
    res.message = "Max vel set successfully.";
    return true;
  }
  res.success = false;
  res.message = "Max vel set faild.";
  return true;
}

bool baseBringup::stopMoveCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  //判断当前控制类型
  if (controll_type_ != MOTOR_MODE_MOVE){
    res.success = false;
    res.message = "Not in MOTOR_MODE_MOVE.";
    lock.unlock();
    return true;
  }
  else{
    move_linear_x_  = 0;
    move_linear_y_  = 0;
    move_angular_z_ = 0;
    res.success = true;
    res.message = "Move stopped ";
    controll_type_ = MOTOR_MODE_CMD;
    lock.unlock();
    return true;
  }
}

void baseBringup::processLoop()
{
  ROS_INFO("baseBringup::processLoop: start");
  uint8_t check_head_last[1]    = {0xFF};
  uint8_t check_head_current[1] = {0xFF};
  while(ros::ok()){
    if (!serial_.isOpen())
    {
      ROS_ERROR("serial unopen");
    }
    try
    {
      int head_type = 0;
      while(ros::ok()) 
      {                        
        size_t head_s = serial_.read(check_head_current,1);
        if (check_head_last[0] == 0x63 && check_head_current[0] == 0x76)
        {
          boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
          pack_read_.read_msg.head[0] = check_head_last[0];
          pack_read_.read_msg.head[1] = check_head_current[0];
          check_head_last[0] = 0xFF;
          lock.unlock();
          head_type = 1; // base
          break;
        }
        else if (check_head_last[0] == 0xfc && (check_head_current[0] == 0x40 || check_head_current[0] == 0x41 || head_type == TYPE_INSGPS || 
                                                check_head_current[0] == TYPE_GROUND || check_head_current[0] == 0x50))
        {
          boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_);
          if      (check_head_current[0] == 0x40)
          {
            imu_frame_.frame.header.header_start = 0xfc;
            imu_frame_.frame.header.data_type    = 0x40;
            head_type = 0x40;
          }
          else if (check_head_current[0] == 0x41)
          {
            ahrs_frame_.frame.header.header_start = 0xfc;
            ahrs_frame_.frame.header.data_type    = 0x41;
            head_type = 0x41;
          }
          else if (check_head_current[0] == TYPE_GROUND){
            head_type = TYPE_GROUND;
          }
          else if (check_head_current[0] == 0x50)
          {
            head_type = 0x50;
          }
          check_head_last[0] = 0xFF;
          lock.unlock();
          if(debug_log_){
            cout << "head_type: " << head_type << endl;
          }
          break;
        }
        check_head_last[0] = check_head_current[0];
      }
      if (head_type == 1)
      {
        size_t res = serial_.read(pack_read_.read_msg.read_msg,READ_DATA_LONGTH);
        if(debug_log_){
          cout << "serial_read: " <<endl;
          for (size_t i = 0; i < READ_MSG_LONGTH; i++)
          {
            cout << std::hex << (int)pack_read_.read_tmp[i] << " ";
          }
          cout << std::dec << endl;
        }
        if(!checkCS(READ_MSG_LONGTH))
        {
          ROS_WARN("check cs error");
        }
        else
        {
          processBattery();
          processOdometry();
        }
        if(!read_first_)
        {
          read_first_ = true;
        }
      }
      else if (head_type == 0x40 || head_type == 0x41|| head_type == TYPE_GROUND || head_type == 0x50 || head_type == TYPE_INSGPS)
      {
        processIMU(head_type);
      }
      else
      {
        if(debug_log_)
        {
          ROS_DEBUG("head_type ERROR.");
        }
      }
    }// try end
    catch(const std::exception& e)
    {
      ROS_ERROR("AIcarController readLoop: %s\n", e.what());
      ROS_ERROR("AIcarController readLoop error, try to reopen serial port\n");
      setSerial();
      openSerial();
    }
    catch(...)
    {
      ROS_ERROR("AIcarController readLoop error, try to reopen serial port\n");
      setSerial();
      openSerial();
    }
  }
}

void baseBringup::processIMU(uint8_t head_type)
{
  uint8_t check_len[1] = {0xff};
  size_t len_s = serial_.read(check_len, 1);
  if (debug_log_){
    std::cout << "check_len: "<< std::dec << (int)check_len[0]  << std::endl;
  }
  if (head_type == TYPE_IMU && check_len[0] != IMU_LEN)
  {
    ROS_WARN("head_len error (imu)");
    return;
  }else if (head_type == TYPE_AHRS && check_len[0] != AHRS_LEN)
  {
    ROS_WARN("head_len error (ahrs)");
    return;
  }else if (head_type == TYPE_INSGPS && check_len[0] != INSGPS_LEN)
  {
    ROS_WARN("head_len error (insgps)");
    return;
  }
  else if (head_type == TYPE_GROUND || head_type == 0x50) // 无效数据，防止记录失败
  {
    uint8_t ground_sn[1];
    size_t ground_sn_s = serial_.read(ground_sn, 1);
    if (++read_sn_ != ground_sn[0])
    {
      if ( ground_sn[0] < read_sn_)
      {
        if(debug_log_){
          ROS_WARN("detected sn lost_1.");
        }
        sn_lost_ += 256 - (int)(read_sn_ - ground_sn[0]);
        read_sn_ = ground_sn[0];
      }
      else
      {
        if(debug_log_){
          ROS_WARN("detected sn lost_2.");
        }
        sn_lost_ += (int)(ground_sn[0] - read_sn_);
        read_sn_ = ground_sn[0];
      }
    }
    uint8_t ground_ignore[500];
    size_t ground_ignore_s = serial_.read(ground_ignore, (check_len[0]+4));
    return;
  }
  //read head sn 
  uint8_t check_sn[1] = {0xff};
  size_t sn_s = serial_.read(check_sn, 1);
  uint8_t head_crc8[1] = {0xff};
  size_t crc8_s = serial_.read(head_crc8, 1);
  uint8_t head_crc16_H[1] = {0xff};
  uint8_t head_crc16_L[1] = {0xff};
  size_t crc16_H_s = serial_.read(head_crc16_H, 1);
  size_t crc16_L_s = serial_.read(head_crc16_L, 1);
  if (debug_log_){
    std::cout << "check_sn: "     << std::hex << (int)check_sn[0]     << std::dec << std::endl;
    std::cout << "head_crc8: "    << std::hex << (int)head_crc8[0]    << std::dec << std::endl;
    std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl;
    std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl;
  }
  // put header & check crc8 & count sn lost
  if (head_type == TYPE_IMU)
  {
    imu_frame_.frame.header.data_size      = check_len[0];
    imu_frame_.frame.header.serial_num     = check_sn[0];
    imu_frame_.frame.header.header_crc8    = head_crc8[0];
    imu_frame_.frame.header.header_crc16_h = head_crc16_H[0];
    imu_frame_.frame.header.header_crc16_l = head_crc16_L[0];
    uint8_t CRC8 = CRC8_Table(imu_frame_.read_buf.frame_header, 4);
    if (CRC8 != imu_frame_.frame.header.header_crc8)
    {
      ROS_WARN("header_crc8 error");
      return;
    }
    if(!imu_frist_sn_){
      read_sn_  = imu_frame_.frame.header.serial_num - 1;
      imu_frist_sn_ = true;
    }
    //check sn 
    baseBringup::checkSN(TYPE_IMU);
  }
  else if (head_type == TYPE_AHRS)
  {
    ahrs_frame_.frame.header.data_size      = check_len[0];
    ahrs_frame_.frame.header.serial_num     = check_sn[0];
    ahrs_frame_.frame.header.header_crc8    = head_crc8[0];
    ahrs_frame_.frame.header.header_crc16_h = head_crc16_H[0];
    ahrs_frame_.frame.header.header_crc16_l = head_crc16_L[0];
    uint8_t CRC8 = CRC8_Table(ahrs_frame_.read_buf.frame_header, 4);
    if (CRC8 != ahrs_frame_.frame.header.header_crc8)
    {
      ROS_WARN("header_crc8 error");
      return;
    }
    if(!imu_frist_sn_){
      read_sn_  = ahrs_frame_.frame.header.serial_num - 1;
      imu_frist_sn_ = true;
    }
    //check sn 
    baseBringup::checkSN(TYPE_AHRS);
  }
  else if (head_type == TYPE_INSGPS)
  {
    insgps_frame_.frame.header.header_start   = 0xfc;
    insgps_frame_.frame.header.data_type      = TYPE_INSGPS;
    insgps_frame_.frame.header.data_size      = check_len[0];
    insgps_frame_.frame.header.serial_num     = check_sn[0];
    insgps_frame_.frame.header.header_crc8    = head_crc8[0];
    insgps_frame_.frame.header.header_crc16_h = head_crc16_H[0];
    insgps_frame_.frame.header.header_crc16_l = head_crc16_L[0];
    uint8_t CRC8 = CRC8_Table(insgps_frame_.read_buf.frame_header, 4);
    if (CRC8 != insgps_frame_.frame.header.header_crc8)
    {
      ROS_WARN("header_crc8 error");
      return;
    }
    else if(debug_log_)
    {
      std::cout << "header_crc8 matched." << std::endl;
    }
    
    baseBringup::checkSN(TYPE_INSGPS);
  }
  if (head_type == TYPE_IMU)
  {
    uint16_t head_crc16_l = imu_frame_.frame.header.header_crc16_l;
    uint16_t head_crc16_h = imu_frame_.frame.header.header_crc16_h;
    uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
    size_t data_s = serial_.read(imu_frame_.read_buf.read_msg, (IMU_LEN + 1)); //48+1
    uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN);
    if (debug_log_){          
      std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
      std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
      std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
      std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
      bool if_right = ((int)head_crc16 == (int)CRC16);
      std::cout << "if_right: " << if_right << std::endl;
    }
    
    if (head_crc16 != CRC16)
    {
      ROS_WARN("check crc16 faild(imu).");
      return;
    }
    else if(imu_frame_.frame.frame_end != FRAME_END)
    {
      ROS_WARN("check frame end.");
      return;
    }
    
  }
  else if (head_type == TYPE_AHRS)
  {
    uint16_t head_crc16_l = ahrs_frame_.frame.header.header_crc16_l;
    uint16_t head_crc16_h = ahrs_frame_.frame.header.header_crc16_h;
    uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
    size_t data_s = serial_.read(ahrs_frame_.read_buf.read_msg, (AHRS_LEN + 1)); //48+1
    uint16_t CRC16 = CRC16_Table(ahrs_frame_.frame.data.data_buff, AHRS_LEN);
    if (debug_log_){          
      std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
      std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
      std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
      std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
      bool if_right = ((int)head_crc16 == (int)CRC16);
      std::cout << "if_right: " << if_right << std::endl;
    }
    
    if (head_crc16 != CRC16)
    {
      ROS_WARN("check crc16 faild(ahrs).");
      return;
    }
    else if(ahrs_frame_.frame.frame_end != FRAME_END)
    {
      ROS_WARN("check frame end.");
      return;
    }
  }
  else if (head_type == TYPE_INSGPS)
  {
    uint16_t head_crc16 = insgps_frame_.frame.header.header_crc16_l + ((uint16_t)insgps_frame_.frame.header.header_crc16_h << 8);
    size_t data_s = serial_.read(insgps_frame_.read_buf.read_msg, (INSGPS_LEN + 1)); //48+1
    uint16_t CRC16 = CRC16_Table(insgps_frame_.frame.data.data_buff, INSGPS_LEN);
    if (head_crc16 != CRC16)
    {
      ROS_WARN("check crc16 faild(insgps).");
      return;
    }
    else if(insgps_frame_.frame.frame_end != FRAME_END)
    {
      ROS_WARN("check frame end.");
      return;
    }
    
  }

  // publish magyaw topic
  if (head_type == TYPE_AHRS)
  {
    // publish imu topic
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = imu_frame_id_.c_str();
    Eigen::Quaterniond q_ahrs(ahrs_frame_.frame.data.data_pack.Qw,
                              ahrs_frame_.frame.data.data_pack.Qx,
                              ahrs_frame_.frame.data.data_pack.Qy,
                              ahrs_frame_.frame.data.data_pack.Qz);
    Eigen::Quaterniond q_r =                          
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_rr =                          
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_xiao_rr =
        Eigen::AngleAxisd( 3.14159/2, Eigen::Vector3d::UnitZ()) * 
        Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
        Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitX());
      
    Eigen::Quaterniond q_out =  q_r * q_ahrs * q_rr;
    imu_data.orientation.w = q_out.w();
    imu_data.orientation.x = q_out.x();
    imu_data.orientation.y = q_out.y();
    imu_data.orientation.z = q_out.z();
    imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
    imu_data.angular_velocity.y = -ahrs_frame_.frame.data.data_pack.PitchSpeed;
    imu_data.angular_velocity.z = -ahrs_frame_.frame.data.data_pack.HeadingSpeed;
    imu_data.linear_acceleration.x = -imu_frame_.frame.data.data_pack.accelerometer_x;
    imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
    imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;

    imu_pub_.publish(imu_data);

    Eigen::Quaterniond rpy_q(imu_data.orientation.w,
                              imu_data.orientation.x,
                              imu_data.orientation.y,
                              imu_data.orientation.z);
    geometry_msgs::Pose2D pose_2d;
    double magx, magy, magz, roll, pitch;
    magx  = -imu_frame_.frame.data.data_pack.magnetometer_x;
    magy  = imu_frame_.frame.data.data_pack.magnetometer_y;
    magz  = imu_frame_.frame.data.data_pack.magnetometer_z;
    Eigen::Vector3d EulerAngle = rpy_q.matrix().eulerAngles(2, 1, 0);
    roll  = EulerAngle[2];
    pitch = EulerAngle[1];

    double magyaw;
    magCalculateYaw(roll, pitch, magyaw, magx, magy, magz);
    pose_2d.theta = magyaw;
    mag_pose_pub_.publish(pose_2d);
  }
}

void baseBringup::magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz)
{
  double temp1 = magy * cos(roll) + magz * sin(roll);
  double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
  magyaw = atan2(-temp1, temp2);
  if(magyaw < 0)
  {
    magyaw = magyaw + 2 * PI;
  }
  // return magyaw;
}

void baseBringup::checkSN(int type)
{
  switch (type)
  {
  case TYPE_IMU:
    if (++read_sn_ != imu_frame_.frame.header.serial_num)
    {
      if ( imu_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num);
        if(debug_log_){
          ROS_WARN("detected sn lost_3.");
        }
      }
      else
      {
        sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_);
        if(debug_log_){
          ROS_WARN("detected sn lost_4.");
        }
      }
    }
    read_sn_ = imu_frame_.frame.header.serial_num;
    break;

  case TYPE_AHRS:
    if (++read_sn_ != ahrs_frame_.frame.header.serial_num)
    {
      if ( ahrs_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - ahrs_frame_.frame.header.serial_num);
        if(debug_log_){
          ROS_WARN("detected sn lost_5.");
        }
      }
      else
      {
        sn_lost_ += (int)(ahrs_frame_.frame.header.serial_num - read_sn_);
        if(debug_log_){
          ROS_WARN("detected sn lost_6.");
        }
      }
    }
    read_sn_ = ahrs_frame_.frame.header.serial_num;
    break;

  case TYPE_INSGPS:
    if (++read_sn_ != insgps_frame_.frame.header.serial_num)
    {
      if ( insgps_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - insgps_frame_.frame.header.serial_num);
        if(debug_log_){
          ROS_WARN("detected sn lost_7.");
        }
      }
      else
      {
        sn_lost_ += (int)(insgps_frame_.frame.header.serial_num - read_sn_);
        if(debug_log_){
          ROS_WARN("detected sn lost_8.");
        }
      }
    }
    read_sn_ = insgps_frame_.frame.header.serial_num;
    break;

  default:
    break;
  }
}

void baseBringup::setWriteCS(int len)
{
	uint8_t ck = 0x00;
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
	for (size_t i = 0; i < len - 1; i++)
	{
		ck += pack_write_.write_tmp[i];
	}
	pack_write_.write_tmp[len - 1] = ck;
  lock.unlock();
}

void baseBringup::joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
  if(debug_log_){
    std::cout << "joyCallback" << std::endl;
  }
  //mode_switch 
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  if (msg->buttons[0]==1)//turn off joy mode
  {
    controll_type_ = MOTOR_MODE_CMD;
    std::cout << "controll_type_ turn to MOTOR_MODE_CMD:" << controll_type_ << std::endl;

  }
  else if (msg->buttons[1] == 1)//turn on joy mode
  {
    controll_type_ = MOTOR_MODE_JOY;
    std::cout << "controll_type_ turn to MOTOR_MODE_JOY:" << controll_type_ << std::endl;
  }
  
  //set speed 
  if (msg->axes[6] == 1)        //twist_speed down
  {
    if(debug_log_){
      std::cout << "twist_speed down" << std::endl;
    }
    twist_gain_ -= 0.1;

  }else if(msg->axes[6] == -1)  //twist_speed up
  {
    if(debug_log_){
      std::cout << "twist_speed up" << std::endl;
    }
    twist_gain_ += 0.1;
  }
  if (msg->axes[7] == -1)      //linear_speed down
  {
    if(debug_log_){
      std::cout << "linear_speed down" << std::endl;
    }
    linear_gain_ -= 0.1;
  }else if(msg->axes[7] == 1)  //linear_speed up
  {
    if(debug_log_){
      std::cout << "linear_speed up" << std::endl;
    }
    linear_gain_ += 0.1;
  }
  //write speed
  double linear_x  = linear_gain_ * msg->axes[1];  // linear_gain_ = 0.3
  double linear_y  = linear_gain_ * msg->axes[0];  // twist_gain_  = 0.2  
  double angular_z = twist_gain_  * msg->axes[2];  // msg->axes[]  = [-1,1]

  if (debug_log_){
    cout << "linear_x=" << linear_x << "linear_y=" << linear_y << "angular_z=" << angular_z << endl;
  }  
  joy_linear_x_  =  linear_x;
  joy_linear_y_  =  linear_y;
  joy_angular_z_ =  angular_z;
  lock.unlock();
  return;
}

void baseBringup::velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  const ros::WallTime callback_enter = ros::WallTime::now();

  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_);

  const ros::WallTime lock_acquired = ros::WallTime::now();
  const double lock_wait_ms =
      (lock_acquired - callback_enter).toSec() * 1000.0;

  if (controll_type_ != MOTOR_MODE_CMD)
  {
    lock.unlock();
    return;
  }

  if(debug_log_)
    std::cout << "vel mode" << std::endl;

  double rx_interval_ms = 0.0;
  if (g_last_cmd_callback_wall_time.toSec() > 0.0)
  {
    rx_interval_ms =
        (lock_acquired - g_last_cmd_callback_wall_time).toSec() * 1000.0;
  }

  ++g_cmd_rx_seq;
  const uint64_t this_rx_seq = g_cmd_rx_seq;

  cmd_linear_x_  = msg->linear.x;
  cmd_linear_y_  = msg->linear.y;
  cmd_angular_z_ = msg->angular.z;
  last_cmd_time_ = ros::Time::now();

  g_last_cmd_callback_wall_time = lock_acquired;
  g_last_rx_lock_wait_ms = lock_wait_ms;
  g_last_rx_interval_ms = rx_interval_ms;

  const double rx_x = cmd_linear_x_;
  const double rx_y = cmd_linear_y_;
  const double rx_w = cmd_angular_z_;

  lock.unlock();

  if (g_diag_enable)
  {
    if (lock_wait_ms > g_diag_lock_wait_warn_ms)
    {
      if (g_base_diag_enable) ROS_WARN("[BASE-DIAG][RX_LOCK_WAIT] rx=%llu wait_ms=%.3f threshold_ms=%.3f cmd=(%.3f,%.3f,%.3f)",
               static_cast<unsigned long long>(this_rx_seq),
               lock_wait_ms,
               g_diag_lock_wait_warn_ms,
               rx_x, rx_y, rx_w);
    }

    // Ignore interval==0 (first command). Normal 70-77 ms jitter is no longer warned.
    if (rx_interval_ms > g_diag_cmd_gap_warn_ms)
    {
      if (g_base_diag_enable) ROS_WARN("[BASE-DIAG][RX_GAP] interval_ms=%.3f threshold_ms=%.3f rx=%llu cmd=(%.3f,%.3f,%.3f)",
               rx_interval_ms,
               g_diag_cmd_gap_warn_ms,
               static_cast<unsigned long long>(this_rx_seq),
               rx_x, rx_y, rx_w);
    }

    if (g_diag_verbose)
    {
      if (g_base_diag_enable) ROS_INFO("[BASE-DIAG][RX] rx=%llu cmd=(%.3f,%.3f,%.3f) interval_ms=%.3f lock_wait_ms=%.3f",
               static_cast<unsigned long long>(this_rx_seq),
               rx_x, rx_y, rx_w,
               rx_interval_ms,
               lock_wait_ms);
    }
  }
}

bool baseBringup::checkCS(int len){
  uint8_t ck = 0;
	for (size_t i = 0; i < len - 1; i++)
	{
		ck += pack_read_.read_tmp[i];
	}
  return pack_read_.read_tmp[len - 1] == ck;
  // return true;
}

void baseBringup::processBattery()
{
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  current_battery_percent_ = (int)pack_read_.pack.battery_percent;
  lock.unlock();
  sensor_msgs::BatteryState battery_msg;
  battery_msg.power_supply_status     = 2; // 放电中 即正在运行
  battery_msg.power_supply_health     = 1; // 良好
  battery_msg.power_supply_technology = 0; // UNKNOWN
  battery_msg.present    = true;           // 存在电池
  battery_msg.percentage = current_battery_percent_; // 电池电量百分比
  battery_pub_.publish(battery_msg);
}

void baseBringup::processOdometry(){
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  current_time_ = ros::Time::now();
  double dt = (current_time_ - last_time_).toSec();
  last_time_ = current_time_;
  lock.unlock();
  double vw1,vw2,vw3,vw4;
  vw1 =-pack_read_.pack.data.pluse_w1 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);
  vw2 = pack_read_.pack.data.pluse_w2 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);
  vw4 =-pack_read_.pack.data.pluse_w3 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);
  vw3 = pack_read_.pack.data.pluse_w4 * 2 * Pi * wheel_radius_ / (encode_resolution_ * period_/1000.0);

  double Vx,Vy,Vth;
  Vx  = ( vw1+vw2+vw3+vw4)/4;
  //cout << "VX="<< Vx <<endl;
  Vy  = 0.975*(-vw1+vw2-vw3+vw4)/4;
  Vth = (-vw1+vw2+vw3-vw4)/(4*(base_shape_a_+base_shape_b_));

  double delta_x = (Vx * cos(th_) - Vy * sin(th_)) * dt;
  double delta_y = (Vx * sin(th_) + Vy * cos(th_)) * dt;
  double delta_th = Vth * dt;
  lock.lock();
  x_  += delta_x;
  y_  += delta_y;
  th_ += delta_th;
  lock.unlock();
  nav_msgs::Odometry odom_tmp;
  odom_tmp.header.stamp = ros::Time::now();
  odom_tmp.header.frame_id = odom_frame_.c_str();
  odom_tmp.child_frame_id  = base_frame_.c_str();
  odom_tmp.pose.pose.position.x = x_;
  odom_tmp.pose.pose.position.y = y_;
  odom_tmp.pose.pose.position.z = 0.0;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
  odom_tmp.pose.pose.orientation = odom_quat;
  if (!Vx||!Vy||!Vth){
    odom_tmp.pose.covariance = ODOM_POSE_COVARIANCE;
  }else{
    odom_tmp.pose.covariance = ODOM_POSE_COVARIANCE2;
  }
  odom_tmp.twist.twist.linear.x  = Vx;
  odom_tmp.twist.twist.linear.y  = Vy;
  odom_tmp.twist.twist.linear.z  = 0.0;
  odom_tmp.twist.twist.angular.x = 0.0;
  odom_tmp.twist.twist.angular.y = 0.0;
  odom_tmp.twist.twist.angular.z = Vth;
  if (!Vx||!Vy||!Vth){
    odom_tmp.twist.covariance = ODOM_TWIST_COVARIANCE;
  }else{
    odom_tmp.twist.covariance = ODOM_TWIST_COVARIANCE2;
  }
  odom_pub_.publish(odom_tmp);
  
  lock.lock();
  current_odom_ = odom_tmp;
  lock.unlock();

  if (ros::param::has("publish_odom_tf"))
  {
    ros::param::get("publish_odom_tf", provide_odom_tf_);
  }
  
  if(provide_odom_tf_)
  {
    geometry_msgs::TransformStamped odom_trans;     /* first, we'll publish the transform over tf */
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = odom_frame_.c_str();
    odom_trans.child_frame_id  = base_frame_.c_str();
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);    /* send the transform */
  }
  updateMileage(odom_tmp.twist.twist.linear.x,odom_tmp.twist.twist.linear.y,dt);
}

bool baseBringup::getMileage(){
  std::fstream fin;
  std::fstream fin_b;
  std::string str_in;
  std::string str_in_b;
  std::stringstream ss;
  ros::NodeHandle pravite_nh("~");
  
	fin.open(Mileage_file_name_.c_str()); //Mileage_backup_file_name_
  fin_b.open(Mileage_backup_file_name_.c_str());
  if (fin.fail() && fin_b.fail())
  {
    ROS_ERROR("open Mileage files error, will creat a new file! \n");
    Mileage_sum_ = 0.0;
    pravite_nh.setParam("Mileage_sum",Mileage_sum_);
    return false;
  }
  if (!fin.fail()){
    fin >> str_in;
    fin.close();
  }
  if (!fin_b.fail()){
      fin_b >> str_in_b;
      fin_b.close();
  }
  if (str_in != "")
  {
    ss << str_in;
    ss >> Mileage_sum_;
    pravite_nh.setParam("Mileage_sum",Mileage_sum_);
    ss.clear();
  }
  else if (str_in_b != "")
  {
    ss << str_in_b;
    ss >> Mileage_sum_;
    ss.clear();
    pravite_nh.setParam("Mileage_sum",Mileage_sum_);
  }
  else
  {
    ROS_ERROR("Mileage_files empty. \n");
    Mileage_sum_ = 0.0;
    pravite_nh.setParam("Mileage_sum",Mileage_sum_);
  }
  return true;
}

bool baseBringup::updateMileage(double vx, double vy, double dt){
  double speed  = sqrt(vx*vx + vy*vy);
  boost::unique_lock<boost::recursive_mutex> lock(Control_mutex_); 
  Mileage_sum_ += speed * dt;
  lock.unlock();
  double d_Mileage = abs(Mileage_sum_ - Mileage_last_);
  if (d_Mileage < 0.1)
  {
    return true;
  }
  FILE* fout = std::fopen(Mileage_file_name_.c_str(), "w");
  if (fout)
	{
		std::fprintf(fout,"%lf\n",Mileage_sum_);
		std::fclose(fout);
	}
  FILE* fout_b = std::fopen(Mileage_backup_file_name_.c_str(), "w");
  if (fout_b)
	{
		std::fprintf(fout_b,"%lf\n",Mileage_sum_);
		std::fclose(fout_b);
	}
  ros::NodeHandle pravite_nh("~");
  pravite_nh.setParam("Mileage_sum",Mileage_sum_);
  Mileage_last_ = Mileage_sum_;
  return true;
}

void baseBringup::callHandle()
{
  serial_.open();
}

}//namespace ucarController

int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_bringup");
  ucarController::baseBringup bp;

  return 0;
}
