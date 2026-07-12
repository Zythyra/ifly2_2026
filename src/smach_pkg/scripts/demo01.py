import rospy
import smach
import smach_ros
import actionlib
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

# 用于实现多个路径点的导航

class NavigateState(smach.State):
    def __init__(self, waypoint, yaw):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.waypoint = waypoint
        self.yaw = yaw
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo(f"Navigating to waypoint: {self.waypoint}")
        
        # 等待 ActionServer 启动
        self.client.wait_for_server()

        # 设置目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.waypoint[0]
        goal.target_pose.pose.position.y = self.waypoint[1]
        quat = quaternion_from_euler(0, 0, self.yaw)
        goal.target_pose.pose.orientation = Quaternion(*quat)
        # 发送目标并等待结果
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_result():
            rospy.loginfo("Navigation succeeded!")
            return 'succeeded'
        else:
            rospy.logwarn("Navigation failed!")
            return 'failed'


# === 定义具体的导航目标点 ===
class NavigateToPointA(NavigateState):
    def __init__(self):
        super().__init__((5.242443714141846, -0.1689101254940033), 0.6)  # 目标坐标 (x=2.0, y=3.0)

class NavigateToPointB(NavigateState):
    def __init__(self):
        super().__init__((3.983996200561523, -0.4224375259876251), 0.6)  # 目标坐标 (x=5.0, y=-1.0)

class NavigateToPointC(NavigateState):
    def __init__(self):
        super().__init__((6.813447456359863, -0.5503661561012268), 0.6)  # 目标坐标 (x=-2.5, y=4.5)


# === 创建状态机 ===
def main():
    rospy.init_node('smach_navigation')

    # 创建状态机
    sm = smach.StateMachine(outcomes=['done'])
    
    with sm:
        smach.StateMachine.add('NAV_A', NavigateToPointA(), 
                               transitions={'succeeded': 'NAV_B', 'failed': 'done'})
        smach.StateMachine.add('NAV_B', NavigateToPointB(), 
                               transitions={'succeeded': 'NAV_C', 'failed': 'done'})
        smach.StateMachine.add('NAV_C', NavigateToPointC(), 
                               transitions={'succeeded': 'done', 'failed': 'done'})

    # 运行状态机
    outcome = sm.execute()
    rospy.loginfo(f"State Machine finished with outcome: {outcome}")


if __name__ == '__main__':
    main()
