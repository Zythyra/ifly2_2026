#include <ros/ros.h>
#include <qr_01/qr_code.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>

#include <algorithm>
#include <clocale>
#include <cstring>
#include <cerrno>
#include <sys/stat.h>
#include <set>
#include <signal.h>
#include <sstream>
#include <string>
#include <vector>

cv::VideoCapture cap;
cv::VideoWriter video_out;
zbar::ImageScanner scanner;

bool camera_active = false;
bool debug_video_enabled = false;
bool qr_save_failed_frame = true;
int camera_device = 0;
int camera_width = 640;
int camera_height = 480;
int flush_frames = 3;
std::string debug_video_path =
    "/home/ucar/ucar_ws_copy/src/qr_01/qr_debug/qr_debug.avi";
const std::string failed_frame_dir =
    "/home/ucar/ucar_ws_copy/src/qr_01/qr_debug";

bool observation_active = false;
// 当前观察视角是否成功解出过任何二维码。
// 失败截图只用于诊断“视觉层完全没有解码成功”的情况；
// 即使第二轮再次解出第一轮已经见过的旧二维码，也不再误保存为失败图。
bool observation_detected_any_qr = false;
int observation_index = 0;
cv::Mat last_observation_frame;

// 整个扫码会话（包含第一轮和第二轮）已经由视觉层成功解出的二维码内容。
// 注意：这里只用于失败截图判定，不影响二维码内容返回给 race.cpp。
std::set<std::string> session_seen_qr_codes;

bool directoryExists(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool ensureFailedFrameDirectory() {
    if (directoryExists(failed_frame_dir)) {
        return true;
    }

    if (mkdir(failed_frame_dir.c_str(), 0755) == 0 || errno == EEXIST) {
        return true;
    }

    ROS_WARN(
        "[QR Server] 无法创建失败截图目录：%s，原因：%s",
        failed_frame_dir.c_str(),
        strerror(errno)
    );
    return false;
}

void saveCurrentObservationIfFailed() {
    if (
        !qr_save_failed_frame
        || !observation_active
        || observation_detected_any_qr
        || last_observation_frame.empty()
    ) {
        return;
    }

    if (!ensureFailedFrameDirectory()) {
        return;
    }

    std::ostringstream filename;
    filename
        << failed_frame_dir
        << "/qr_failed_observation_"
        << observation_index
        << "_"
        << ros::WallTime::now().toNSec()
        << ".jpg";

    const std::string path = filename.str();
    if (cv::imwrite(path, last_observation_frame)) {
        ROS_WARN(
            "[QR Server] 当前观察视角完全未解出二维码，已保存失败画面：%s",
            path.c_str()
        );
    } else {
        ROS_WARN(
            "[QR Server] 当前观察视角完全未解出二维码，但失败画面保存失败：%s",
            path.c_str()
        );
    }
}

void startNewObservation() {
    saveCurrentObservationIfFailed();

    observation_active = true;
    observation_detected_any_qr = false;
    ++observation_index;
    last_observation_frame.release();
}

void finishCurrentObservation() {
    saveCurrentObservationIfFailed();
    observation_active = false;
    observation_detected_any_qr = false;
    last_observation_frame.release();
}

void releaseCamera() {
    if (cap.isOpened()) {
        cap.release();
    }
    if (video_out.isOpened()) {
        video_out.release();
    }
    camera_active = false;
}

void sigintHandler(int sig) {
    (void)sig;
    releaseCamera();
    ros::shutdown();
}

bool openCamera(std::string& error_text) {
    if (camera_active && cap.isOpened()) {
        return true;
    }

    releaseCamera();

    if (!cap.open(camera_device, cv::CAP_V4L2) || !cap.isOpened()) {
        error_text = "ERROR: CAMERA_OPEN_FAILED";
        ROS_ERROR("[QR Server] 摄像头打开失败，device=%d", camera_device);
        return false;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, camera_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height);

    camera_active = true;
    ROS_INFO(
        "[QR Server] 摄像头已开启：device=%d，%dx%d",
        camera_device,
        camera_width,
        camera_height
    );

    if (debug_video_enabled && !video_out.isOpened()) {
        video_out.open(
            debug_video_path,
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            10.0,
            cv::Size(camera_width, camera_height)
        );
        if (!video_out.isOpened()) {
            ROS_WARN(
                "[QR Server] 调试视频打开失败：%s；继续执行二维码识别",
                debug_video_path.c_str()
            );
        } else {
            ROS_INFO("[QR Server] 调试视频已开启：%s", debug_video_path.c_str());
        }
    }

    return true;
}

bool flushCameraBuffer(std::string& error_text) {
    if (!camera_active || !cap.isOpened()) {
        error_text = "ERROR: CAMERA_CLOSED";
        return false;
    }

    const int count = std::max(0, flush_frames);
    for (int i = 0; i < count; ++i) {
        if (!cap.grab()) {
            error_text = "ERROR: CAMERA_GRAB_FAILED";
            ROS_WARN("[QR Server] 清理缓存时 grab 失败");
            return false;
        }
    }

    ROS_INFO("[QR Server] 已清理 %d 帧摄像头缓存", count);
    return true;
}


std::set<std::string> scanQrWithZbar(
    const cv::Mat& gray,
    cv::Mat& draw_frame,
    bool source_is_mirrored
) {
    std::set<std::string> results;

    if (gray.empty()) {
        return results;
    }

    zbar::Image zbar_image(
        gray.cols,
        gray.rows,
        "Y800",
        gray.data,
        gray.cols * gray.rows
    );

    scanner.scan(zbar_image);

    for (
        zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
        symbol != zbar_image.symbol_end();
        ++symbol
    ) {
        const std::string data = symbol->get_data();
        if (!data.empty()) {
            results.insert(data);
        }

        std::vector<cv::Point> points;
        for (int i = 0; i < symbol->get_location_size(); ++i) {
            int x = symbol->get_location_x(i);
            const int y = symbol->get_location_y(i);

            // 镜像图上的坐标转换回原始摄像头画面坐标，
            // 保证调试视频/画框始终基于原始帧显示。
            if (source_is_mirrored) {
                x = draw_frame.cols - 1 - x;
            }

            points.emplace_back(x, y);
        }

        if (points.size() >= 4) {
            for (std::size_t i = 0; i < points.size(); ++i) {
                cv::line(
                    draw_frame,
                    points[i],
                    points[(i + 1) % points.size()],
                    cv::Scalar(0, 255, 0),
                    3
                );
            }
        }
    }

    // 防止 zbar::Image 析构时继续持有 OpenCV Mat 的数据指针。
    zbar_image.set_data(nullptr, 0);
    return results;
}

bool qr_detect_cb(
    qr_01::qr_code::Request& req,
    qr_01::qr_code::Response& resp
) {
    resp.result.clear();

    // -1：开启摄像头。扫码阶段只需调用一次，后续观察点保持相机开启。
    if (req.command == -1) {
        const bool starting_new_session =
            !camera_active || !cap.isOpened();

        if (starting_new_session) {
            session_seen_qr_codes.clear();
            observation_active = false;
            observation_detected_any_qr = false;
            observation_index = 0;
            last_observation_frame.release();
            ROS_INFO("[QR Server] 开始新的二维码扫码会话，已清空视觉去重记录");
        }

        std::string error_text;
        if (!openCamera(error_text)) {
            resp.result = error_text;
        }
        return true;
    }

    // -2：整个扫码阶段结束后释放摄像头。
    if (req.command == -2) {
        finishCurrentObservation();
        if (camera_active || cap.isOpened()) {
            releaseCamera();
            ROS_INFO("[QR Server] 摄像头已释放");
        }
        session_seen_qr_codes.clear();
        return true;
    }

    // -3：到达新的观察点后丢弃移动过程中积压的旧帧。
    // 同时结束上一观察点；若上一观察点整个期间都没有解出“本次会话中
    // 此前未见过的新二维码”，则在 qr_save_failed_frame=true 时保存最后一帧。
    if (req.command == -3) {
        startNewObservation();
        std::string error_text;
        if (!flushCameraBuffer(error_text)) {
            resp.result = error_text;
        }
        return true;
    }

    // 识别流程。
    if (!camera_active || !cap.isOpened()) {
        ROS_ERROR("[QR Server] 摄像头未打开");
        resp.result = "ERROR: CAMERA_CLOSED";
        return true;
    }

    cv::Mat frame;
    cap >> frame;
    if (frame.empty()) {
        ROS_WARN("[QR Server] 获取到空帧");
        resp.result = "ERROR: FRAME_EMPTY";
        return true;
    }

    // 失败截图始终保存摄像头真正采集到的原始帧。
    // 这样后续分析时不会再被水平镜像干扰。
    if (qr_save_failed_frame && observation_active) {
        last_observation_frame = frame.clone();
    }

    // 第一优先级：直接使用摄像头原始帧进行 ZBar 解码。
    cv::Mat raw_gray;
    cv::cvtColor(frame, raw_gray, cv::COLOR_BGR2GRAY);

    std::set<std::string> frame_results =
        scanQrWithZbar(raw_gray, frame, false);

    // 只有原始帧完全没有解出二维码时，才保留旧版本的水平镜像作为 fallback。
    // 这样正常二维码不增加第二次扫描开销，同时兼容过去镜像路径可能更容易
    // 识别的特殊场景。
    if (frame_results.empty()) {
        cv::Mat mirrored_frame;
        cv::Mat mirrored_gray;

        cv::flip(frame, mirrored_frame, 1);
        cv::cvtColor(
            mirrored_frame,
            mirrored_gray,
            cv::COLOR_BGR2GRAY
        );

        frame_results =
            scanQrWithZbar(mirrored_gray, frame, true);

        if (!frame_results.empty()) {
            ROS_DEBUG(
                "[QR Server] 原始帧未解码，水平镜像 fallback 成功识别 %zu 个二维码",
                frame_results.size()
            );
        }
    }

    // 同一帧允许一次返回多个二维码，使用换行符分隔。
    if (!frame_results.empty()) {
        // 只要这一视角成功解出过任何二维码，就说明视觉解码本身成功，
        // 不应保存“失败截图”。是否为本次会话新二维码仍单独统计，仅用于调试。
        observation_detected_any_qr = true;

        std::size_t new_qr_count = 0;
        for (const std::string& value : frame_results) {
            const auto insert_result = session_seen_qr_codes.insert(value);
            if (insert_result.second) {
                ++new_qr_count;
            }
        }

        // 无论是不是重复二维码，都继续原样返回给 race.cpp。
        // 因此这里只改变失败截图判定，不改变原来的扫码/去重/HTTP逻辑。
        std::ostringstream output;
        bool first = true;
        for (const std::string& value : frame_results) {
            if (!first) {
                output << '\n';
            }
            first = false;
            output << value;
        }
        resp.result = output.str();

        ROS_DEBUG(
            "[QR Server] 当前帧识别到 %zu 个二维码，其中 %zu 个为本次会话新二维码",
            frame_results.size(),
            new_qr_count
        );
    }

    if (debug_video_enabled && video_out.isOpened()) {
        video_out.write(frame);
    }

    return true;
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(
        argc,
        argv,
        "qr_server_node",
        ros::init_options::NoSigintHandler
    );
    signal(SIGINT, sigintHandler);

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("camera_device", camera_device, 0);
    private_nh.param("camera_width", camera_width, 640);
    private_nh.param("camera_height", camera_height, 480);
    private_nh.param("flush_frames", flush_frames, 3);
    private_nh.param("debug_video", debug_video_enabled, false);
    private_nh.param("qr_save_failed_frame", qr_save_failed_frame, true);
    private_nh.param<std::string>(
        "debug_video_path",
        debug_video_path,
        "/home/ucar/ucar_ws_copy/src/qr_01/qr_debug/qr_debug.avi"
    );

    // 只启用二维码识别，避免扫描其他条码类型。
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    ros::ServiceServer service = nh.advertiseService(
        "qr_detect",
        qr_detect_cb
    );

    ROS_INFO("二维码服务端启动完毕，等待调用...");
    ROS_INFO(
        "参数：device=%d，%dx%d，flush_frames=%d，debug_video=%s，qr_save_failed_frame=%s",
        camera_device,
        camera_width,
        camera_height,
        flush_frames,
        debug_video_enabled ? "true" : "false",
        qr_save_failed_frame ? "true" : "false"
    );
    ROS_INFO(
        "二维码解码策略：原始帧 ZBar 优先；仅原始帧失败时启用水平镜像 ZBar fallback"
    );

    ros::spin();
    finishCurrentObservation();
    releaseCamera();
    return 0;
}