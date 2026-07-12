#！/home/ucar/miniforge3/envs/opencv_env/bin/env python3

import os
import smach
import smach_ros
import rospy
import actionlib
import threading
import time
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

voiceflag = False
voice_event = threading.Event()

# 语音回调函数，用于处理语音信号
def VoiceCallback(data):
    global voiceflag
    if data.data == "start":
        rospy.loginfo('I get the start sign')
        voiceflag = True
        voice_event.set()  # 设置事件，通知主线程
    else:
        rospy.loginfo('Waiting voice sign')
        voiceflag = False
        voice_event.set()  # 设置事件，通知主线程        
        

# 语音状态
class Wait4Awake(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigating', 'wait'])
        self.start_time = None  # 用于记录开始等待的时间
    
    def execute(self, userdata):
        global voice_event, voiceflag
        rospy.Subscriber("voiceAwake", String, VoiceCallback)
        rospy.loginfo("Waiting for voice wake-up...")

        # 记录开始等待的时间
        self.start_time = time.time()
        
        # 等待最多 5 秒，或者通过回调更新 voiceflag
        while not voice_event.is_set():
            # 如果已经等待超过 30 秒，强制启动
            if time.time() - self.start_time >= 30:
                rospy.loginfo("30 seconds passed, force starting navigation.")
                voiceflag = True  # 强制设置为 True，表示接收到有效指令
                voice_event.set()  # 通知主线程
                break

            # 继续等待
            voice_event.wait(timeout=0.1)  # 每次等待 100ms，避免占用过多的 CPU

        # 根据 voiceflag 的值决定状态转换
        if voiceflag:
            rospy.loginfo("Voice command received or forced start, starting navigation.")
            return 'navigating'
        else:
            rospy.loginfo("No valid voice command received, continuing to wait.")
            return 'wait'

        
# 导航状态（给予目标点，导航到特定位置）
class Navigate2Target(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived', 'waiting'], input_keys=['navpoints'], output_keys=['navpoints'])
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    def getTarget(self, navpoints):
        param_name = f"/point{int(navpoints)}/x_y_yaw"
        xy_yaw = rospy.get_param(param_name)
        rospy.loginfo(f"the target is at {xy_yaw}")
        return xy_yaw
    
    def execute(self, userdata):
        # 使用 userdata.navpoints 访问传递的输入数据
        rospy.loginfo("Navigating to: %s", userdata.navpoints)
        navpoints = userdata.navpoints
        target = self.getTarget(navpoints)
        self.client.wait_for_server()
        # 设置目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target[0]
        goal.target_pose.pose.position.y = target[1]
        quat = quaternion_from_euler(0, 0, target[2])
        goal.target_pose.pose.orientation = Quaternion(*quat)
        # 发送目标并等待结果
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_result():
            rospy.loginfo("Navigation succeeded!")
            userdata.navpoints = navpoints + 1
            return 'arrived'
        else:
            rospy.logwarn("Navigation failed!")
            return 'waiting'


# 根据当前导航到的位置决定接下来要进行的操作
class Decision(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['erweima', 'scan', 'park', 'simulation', 'judgeColor', 'visualTask'], 
                             input_keys=['navpoints'], output_keys=['navpoints'])
        self.workstream = {1: 'erweima', 2: 'scan', 3: 'park', 4: 'park',
                            5: 'park', 6: 'simulation', 7: 'judgeColor', 8: 'visualTask'}
    
    def execute(self, userdata):
        current_nav = userdata.navpoints - 1
        return self.workstream[current_nav]


# 调用二维码识别节点，返回目标
class Erweima(smach.State):
    def __init__(self, outcomes=..., input_keys=..., output_keys=..., io_keys=...):
        pass


# 在该点扫描四周，并识别出目标物品的位置
class Scan(smach.State):
    pass


# 在目标物品前的白框内停下，并播报我已balabala
class Park(smach.State):
    pass


# 停下来，在电脑端开始仿真任务，接受电脑给的消息，进入下一个任务
class Simulation(smach.State):
    pass


# 判断在左侧的灯的颜色，指出接下来的导航点
class JudgeColor(smach.State):
    pass  


# 已经进入了视觉巡线的位置，给出是左巡线还是右巡线
class VisualTask(smach.State):
    pass



def main():
    rospy.init_node('first_try')
    sm = smach.StateMachine(outcomes=['end'])
    sm.userdata.navpoints = 1

    # add_thread = threading.Thread(target = thread_detect)
    # add_thread.start()

    with sm:        
        smach.StateMachine.add('Wait', Wait4Awake(), transitions = {'navigating':'Nav', 'wait':'Wait' })
        smach.StateMachine.add('Nav', Navigate2Target(), transitions = {'arrived':'Decision', 'waiting':'End'})
        smach.StateMachine.add('Decision', Decision(), transitions = {'erweima':'Ewm', 'scan':'Scan', 'park':'Park', 'simulation':'Simulation',
                                                                      'judgeColor':'Judge', 'visualTask':'VisualTask'})
        # smach.StateMachine.add('NAV', Navigate(), transitions={'arrived':'ARUCO', 'navigating':'NAV','end':'VOISUCC'})
        # smach.StateMachine.add('ARUCO', Aruco(), transitions={'NAV2C': 'NAV2C','nav2D1':'NAV', 'nav2D2':'NAV','nav2D3':'NAV','Aruco':'ARUCO','end':'end'})
        # smach.StateMachine.add('VOISUCC',voiceSuccess(),transitions={'end':'end'})
        # smach.StateMachine.add('NAV2C',Navigate2C(),transitions={'navigating':'NAV2C','arrived':'NAV2PIC','end':'end'})
        # smach.StateMachine.add('NAV2PIC',Navigate2Pic(),transitions={'navigating':'NAV2PIC','arrived':'NAV','end':'end'})


        sis = smach_ros.IntrospectionServer('FIRST_TRY', sm, '/SM_ROOT')
        sis.start()
        
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()
        