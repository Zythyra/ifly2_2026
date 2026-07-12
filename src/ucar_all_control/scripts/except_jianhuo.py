#比赛中一个非常有意思的一个点：当小车在仿真任务区到达终点是可能会进行语音播报，而一旦进行语音播报就视为仿真任务失败
#给我的第一感觉是，小车无法区分仿真环境与现实环境，而导致小车在仿真环境中以为已到达终点，然后直接进行语音播报
#这是否会与相同的发布话题有关，或许得对相同的话题进行remap from to 修改话题名称或者加上namespace进行区别
#但是这种失误能否用更简单的方法在程序中进行解决呢？比如利用代码逻辑什么的，这个还需要思考和实践

#本代码由于语音播报所需的匹配项较多，为了让语音播报看起来更加简洁，利用回调函数来解决这个问题，相当于是两个进程，主程序的i（目标点的执行）对语音播报没有影响
#当订阅者订阅到了视觉识别的内容时会自己进行播报，此时我们只需要主程序中延迟与语音播报相同时间的间隔，最好比语音播报稍长的间隔，即可实现定点播报



#由于发布者我一共只发布了一次，故摄像头一旦被打开后可能就会关掉，得考虑循环发布的问题，但是摄像头一直开着会导致非常的卡顿
import rospy
import math
import numpy as np
import actionlib
import os
from geometry_msgs.msg import Twist, Point, Quaternion, Pose,PoseArray
from std_msgs.msg import Int8,String,Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionResult
from cv_bridge import CvBridge
import cv2
from actionlib_msgs import GoalStatus
# from std_msgs.msg import Int32 麦克风话题数据的消息类型

class RobotController:

    #其实我们书写代码时都有一个习惯，因为是用msg接收消息，但是接收完之后若还要取的他具体的值只能再调用回调函数，但是回调函数是不能单独被调用的，他是订阅者订阅消息时用于处理消息的
    #故我们要将回调函数中接收到的数据利用本地变量传下来，方便作为标识变量或者其他功能直接被我们的逻辑代码所使用
    def __init__(self):
        # self.cmd_pub = None
        self.awake_flag = 0#麦克风唤醒标志位
        # self.stop_flag = 0#暂停标志位 为1是停下，为0时继续
        self.current_pose = Pose()
        self.lasergoal = MoveBaseGoal()#雷达目标点的对象
        self.sound_file = "" #声源通道的载体
        self.bridge = CvBridge()
        self.goal_list = []
        self.point = Pose()#这个与当时创建String（）的消息载体用于发布是一样的 #点的位姿，保存了四元素和xyz的位置
        self.index = 0#用于后续的总控目标点的匹配执行
        self.detect_for = Int8()#用于组织视觉识别发布者的消息数据（标志位）

        #用于计算睡眠的时间
        self.rate = rospy.Rate(5)

        #以下三个变量用于存储从视觉中识别到的物资，并进行组合生成最终的物资标签
        #还有一种思路就是因为视觉本来就会进行代码的识别，可以先让视觉进行储存变量，最后把组合的变量在完成视觉巡线的时候发给我，并把停车指令发给我，或者直接对底盘发送用于停车（干脆）
        #视觉处理完之后会把消息发给我，到时候订阅者会直接订阅并发布，无需再进行变量的储存
        # self.mix_obj = ""
        # self.ob1 = ""
        # self.ob2 = ""

        #用于红绿灯路口识别
        self.color = ""

        #以下两行用于雷达目标点的获取使用
        self.laser_dot = PoseArray()
        self.laser_dot_list = []
        #下面这行是用于存储获取雷达扫描的目标点的数目的
        self.laser_num = 0

        #用于组织消息（标志位），用于发布
        #这部分还得进行商量，看看到时候该写多少个接口!!!
        self.msg_qr_code = Int8()#用于组织二维码的发布消息
        self.msg_object = Int8()#用于组织拣货区的发布消息
        self.msg_traficlight = Int8()#用于组织红绿灯的识别消息

        #用于组织一个用于旋转的消息载体
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 1.0

        # self.wuping_price = [[4,2,5],[2,5,2],[5,10,3]]#这是一个存放物资单价的列表
        # self.fruit = 0
        # self.vegetable = 1
        # self.sweet = 2
        #下述代码的实现已经利用字典进行了匹配，可以不用再定义一系列的变量进行传值判断了
        # self.erweima_wuzi01 = ""
        # self.erweima_wuzi02 = ""
        # self.lukou_yi_flag = 0 #为0时表示不可通过，为1表示可以通过
        # self.lukou_er_flag = 0#为0时表示不可通过，为1表示可以通过
        # self.j = 0#循坏标志位

        #下面这版的参数用于其他的算法，主要用于用launch文件创建参数文件和把总控节点包含进去用于启动
        #最后再在python文件中进行导入到对应的自定义参数
        self.goal_num = 0
        self.goal_param = []
        self.goal_test_param = MoveBaseGoal()

        #发布者统一创建

        self.start_xx_pub = rospy.Publisher("xun_start", Int8, queue_size=1)#巡线开启的发布者
        self.start_yd_pub = rospy.Publisher("start_yd",Int8,queue_size=1)#开始雷达检测的发布者
        self.start_detect_pub = rospy.Publisher("start_detect",Int8,queue_size=1)#开始目标检测的发布者 为1表示检测二维码，为2表示检测拣货区识别版，为3表示识别红绿灯，为0表示关闭识别
        self.twist_pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)#用于发布消息进行定点旋转寻找所需的物资
        #订阅者统一创建
        # self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.mic_sub = rospy.Subscriber("/angle", Int32, self.mic_awake_cb,queue_size=1)#麦克风订阅者，用于接收麦克风标志位
        # self.laser_flag_sub = rospy.Subscriber("laser_scan", Int8, self.laser_flag_cb,queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb,queue_size=1)#用于订阅里程计消息用于定位或者获取当前的速度
        # self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback,queue_size=1)
        # self.cam_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback,queue_size=1)

        #下述终点巡线后结束在视觉段代码执行并完成，即在视觉中完成巡线后，视觉代码创建发布者发消息给底盘停止运动
        #并通过之前的"cv_msg"话题以String的消息格式对消息进行返回，完成语音播报
        # self.xunxian_sub = rospy.Subscriber("xun_xian", Int8, self.xun_sub_cb,queue_size=1)#巡线代码的订阅者，用于订阅视觉巡线结束后获取标志位，标志着视觉巡线的完成，然后进行终点的语音播报
        self.cv_sub = rospy.Subscriber("cv_msg",String,self.cv_msgs_callback,queue_size=1)#视觉识别的结果，用于输出对应的语音
        self.yd_sub = rospy.Subscriber("yd_msg",PoseArray,self.yd_msgCallback,queue_size=1)#雷达的回调函数订阅者，用于处理雷达信息
        self.gazebo_sub = rospy.Subscriber("gazebo_msg",String,self.gazebo_Callback,queue_size=1)#暂时写String()，后续还得看仿真环境与现实环境是怎么通信的
        self.crossing_sub = rospy.Subscriber("crossing",String,self.crossing_detect_cb,queue_size=1)

        # action通讯与move_base对接的接口的话题用的是"move_base"，而如果用话题通信，对接的接口的话题就是"move_base_simple/goal"
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # self.ac.wait_for_server() #这一行先写在后面，后期看看要不要改

        #这里针对最后的语音输出还是提出一种思路：进行动态的传参思路
        #下面的最后的播报是针对写死的语音，用于参数匹配
        #这是一个用于存储所有音频文件的字典

        #写参数也有两种思路，一种是把识别的物件存储下来后，写18个if else来进行最后的语音播报
        #另一种思路是把语音的键值对先写进字典里，生成一个键来进行匹配
        self.audio_dict = {
            #二维码识别播报
            "fruit": "mpg123 /home/your_user/xxx.mp3",
            "vegetable": "mpg123 /home/your_user/xxx.mp3",
            "sweet": "mpg123 /home/your_user/xxx.mp3",
            #现实取物播报
            "apple": "mpg123 /home/your_user/xxx.mp3",
            "banana": "mpg123 /home/your_user/xxx.mp3",
            "watermelon": "mpg123 /home/your_user/xxx.mp3",

            "chili": "mpg123 /home/your_user/xxx.mp3",
            "tomato": "mpg123 /home/your_user/xxx.mp3",
            "potato": "mpg123 /home/your_user/xxx.mp3",

            "milk": "mpg123 /home/your_user/xxx.mp3",
            "cake": "mpg123 /home/your_user/xxx.mp3",
            "coke": "mpg123 /home/your_user/xxx.mp3",
            # #虚拟仿真物品所在区域播报
            # "A": "mpg123 /home/your_user/xxx.mp3",
            # "B": "mpg123 /home/your_user/xxx.mp3",
            # "C": "mpg123 /home/your_user/xxx.mp3",
            #水果组终点播报
            "apple_apple":"mpg123 /home/your_user/xxx.mp3",
            "banana_banana": "mpg123 /home/your_user/xxx.mp3",
            "watermelon_watermelon": "mpg123 /home/your_user/xxx.mp3",
            "apple_banana": "mpg123 /home/your_user/xxx.mp3",
            "apple_watermelon": "mpg123 /home/your_user/xxx.mp3",
            "banana_watermelon": "mpg123 /home/your_user/xxx.mp3",
            #蔬菜组终点播报
            "chili_chili": "mpg123 /home/your_user/xxx.mp3",
            "tomato_tomato": "mpg123 /home/your_user/xxx.mp3",
            "potato_potato": "mpg123 /home/your_user/xxx.mp3",
            "chili_tomato": "mpg123 /home/your_user/xxx.mp3",
            "chili_potato": "mpg123 /home/your_user/xxx.mp3",
            "tomato_potato": "mpg123 /home/your_user/xxx.mp3",
            #甜品组终点播报
            "milk_milk": "mpg123 /home/your_user/xxx.mp3",
            "cake_cake": "mpg123 /home/your_user/xxx.mp3",
            "coke_coke": "mpg123 /home/your_user/xxx.mp3",
            "milk_cake": "mpg123 /home/your_user/xxx.mp3",
            "milk_coke": "mpg123 /home/your_user/xxx.mp3",
            "cake_coke": "mpg123 /home/your_user/xxx.mp3"

        }

    def mic_awake_cb(self, msg):
        self.awake_flag = msg.data
        rospy.loginfo("heard success")
        #这一步看似多输出了一步多余的，但是实际上可以检测msg.datacv_msgs_callback
        rospy.loginfo("I heard %d",self.awake_flag)

    # 视觉回调函数优化
    def cv_msgs_callback(self, msg):
        rospy.loginfo("I heard: %s", msg.data)
        # 播放音频（如果在字典中）
        if msg.data in self.audio_dict:
            os.system(self.audio_dict[msg.data])
    
    def yd_msgCallback(self, msg):
        # 处理雷达数据
        rospy.loginfo("Received laser data with %d points", len(msg.poses))
        self.laser_dot_num = len(msg.poses)
        for i in range(self.laser_dot_num):
            laser_dot_list = Pose()
            laser_dot.position.x = msg.poses[i].position.x
            laser_dot.position.y = msg.poses[i].position.y
            laser_dot.orientation.z = msg.poses[i].orientation.z
            laser_dot.orientation.w = msg.poses[i].orientation.w
            self.laser_dot_list.append(laser_dot)
    # 基于雷达的板前定位识别现在先给出一种思想
    # 先让小车旋转一周，雷达把板前三个点的坐标获取后，同时视觉部分也进行回调，得到三个识别版的标签，订阅后进行字典的组织，然后根据前面取到的二维码信息
    #进行if判断，用来决定该定位到哪块板前，并把这个物资记录下来，方便后续进行语音播报，视觉那边利用另外的判断机制进行后续终点的信息组合
    #不需要再在主程序进行传回了，避免过于麻烦以及信息错乱
    for 
    self.twist_pub.publish(self.twist)

    # 一句话概括：先进行旋转取点，进行摄像头点的一一对应匹配，然后根据判断跑到对应的板前
    # 需要考虑的问题：1.板与墙壁贴合雷达扫不到 2.雷达可能会误识别板子 3.摄像头传过来的数据可能有误
    # 好处：可以将旋转以及信息的接收独立开来，避免了因为节点spin不上而导致的问题
    



    def crossing_detect_cb(self,msg):
        self.color = msg.data
        # if color == "green":
        #     os.system("mpg123 /home/your_user/xxx.mp3")
        # elif color == "red":
        #     self.send_goal(self.goal_list[8])#这是另一块板前
        #     os.system("mpg123 /home/your_user/xxx.mp3")
        # else:
        #     rospy.loginfo("Waiting for msg")
        #


    #这个函数就是用在不同的发布者和对应的消息载体之间，进行反复的发布，确保对方的订阅者能收到消息
    def publishMessageAndSleep(publisher, msg, duration=0.2):
        for i in range(5):
            publisher.publish(msg)
            rospy.sleep(duration)

    # nav_msgs/Odometry是里程计消息，常用于里程计定位，其中的Pose包含了物体的坐标位置以及角度
    # 其中的Twist包含了物体的线速度和角速度

    #仿真区域的播报与视觉识别无关，还是分开为好
    def gazebo_Callback(self,msgs):
        if msgs.data == 'A':
            os.system("mpg123 /home/your_user/xxx.mp3")
        if msgs.data == 'B':
            os.system("mpg123 /home/your_user/xxx.mp3")
        if msgs.data == 'C':
            os.system("mpg123 /home/your_user/xxx.mp3")





    def odom_cb(self,msg):
        self.current_pose.position.x = msg.pose.pose.position.x
        self.current_pose.position.y = msg.pose.pose.position.y
        self.current_pose.position.z = msg.pose.pose.position.z
        self.current_pose.orientation.x = 0.0
        self.current_pose.orientation.y = 0.0
        self.current_pose.orientation.z = msg.pose.pose.orientation.z
        self.current_pose.orientation.w = msg.pose.pose.orientation.w

    def yd_msgCallback(self,msg):
        #len是否可以计算geometry_msgs/Point类型的列表长度
        self.laser_num = len(msg.poses)
        rospy.loginfo("I heard %d",len(msg.poses))
        for i in range(laser_num):
            self.laser_dot.poses.position.x = msg.poses[i].position.x
            self.laser_dot.poses.position.y = msg.poses[i].position.y
            self.laser_dot.poses.orientation.z = msg.poses[i].orientation.z
            self.laser_dot.poses.orientation.w = msg.poses[i].orientation.w
            self.laser_dot_list.append(laser_dot)
            # 当i == goal_num -1时，意味着参数已全部读入
            rospy.loginfo("I heard i")
        # i默认从0开始


    #不需要单独定义一个订阅者用于订阅到达终点，因为只要在视觉巡线代码中规定到达终点后给我们传个语音就行
    # def xun_sub_cb(self,msg):
    #     if msgs.data == 1 :
    #         rospy.loginfo("I have achieve!!!")
    #         #回调函数内部嵌套回调函数是否会出错？？？
    #         self.cv_msgs_callback()



    #考虑后期是否要将参数写入launch文件后再导入
    def get_param(self):
        self.goal_num = rospy.get_param("")
        #i默认从0开始
        for i in range(self.goal_num):
            #到时候还得考虑该坐标点参考的坐标系是哪个
            self.goal_test_param.target_pose.head.frame_id = "map"
            self.goal_test_param.target_pose.pose.position.x = rospy.get_param("")
            self.goal_test_param.target_pose.pose.position.y = rospy.get_param("")
            # self.goal_test_param.target_pose.pose.position.z = 0.0
            # self.goal_test_param.target_pose.pose.orientation.x = 0.0
            # self.goal_test_param.target_pose.pose.orientation.y = 0.0
            self.goal_test_param.target_pose.pose.orientation.z = rospy.get_param("")
            self.goal_test_param.target_pose.pose.orientation.w = rospy.get_param("")
            goal_param.append(goal_test_param)
            #当i == goal_num -1时，意味着参数已全部读入
            rospy.loginfo("I heard i")



    # 这种写法在于直接把目标点放在了主程序中，而没有单独的写一个launch文件进行启动节点的同时进行导入参数
    def init_goals(self):
        goals_data = [
            (0.285, 0.38, 0, 0, 0, 0.707, 0.707),
            (0.4, -0.4, 0, 0, 0, -0.707, 0.707),
            (0.94, 0.57, 0, 0, 0, 1, 0),
            (1.5, -0.45, 0, 0, 0, 1, 0),
            (2.4, 0.48, 0, 0, 0, 0.707, 0.707),
            (2.33, -0.32, 0, 0, 0, -0.707, 0.707),
            (2.8, 1.15, 0, 0, 0, 0.707, 0.707),
            (2.87, 0.0, 0, 0, 0, 0, 1),
            (3.66, 0.73, 0, 0, 0, 0.707, 0.707),
            (3.53, -0.2, 0, 0, 0, -0.707, 0.707),
            (4.5, 0.6, 0, 0, 0, 0.707, 0.707),
            (4.55, -0.28, 0, 0, 0, -0.707, 0.707)
        ]
        # 意为依次将x, y, z, ox, oy, oz, ow从goals_data中读出并进行依次组织目标点进行赋值
        for x, y, z, ox, oy, oz, ow in goals_data:
            pose = Pose()
            # pose.position = Point(x, y, z)
            # pose.orientation = Quaternion(ox, oy, oz, ow)
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = oz
            pose.orientation.w = ow
            #append表示在末尾添加，故先传入的下标是在最前面的
            #列表会自动生成下标，故先进去的是下标0，最后进去的元素下标就是总的元素个数-1
            self.goal_list.append(pose)

    #这里pose若不用就可以去掉了
    def send_goal(self, pose, timeout=30):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        # goal.target_pose.pose.position.x = goal_list[self.j].position.x
        # goal.target_pose.pose.position.y = goal_list[self.j].position.y
        # goal.target_pose.pose.position.z = 0
        # goal.target_pose.pose.orientation.x = 0
        # goal.target_pose.pose.orientation.y = 0
        # goal.target_pose.pose.orientation.z = goal_list[self.j].orientation.z
        # goal.target_pose.pose.orientation.w = goal_list[self.j].orientation.w
        self.ac.send_goal(goal)
        # self.j += 1

        #用于进行目标点发送的异常捕获
        # finished = self.ac.wait_for_result(rospy.Duration(timeout))
        # if not finished:
        #     rospy.logwarn("Goal timed out after %d seconds", timeout)
        #     self.ac.cancel_goal()
        # else:
        #     state = self.ac.get_state()
        #     rospy.loginfo("Goal finished with state: %d", state)

    #这是一个用于播放音频的函数，若要使用，还得修改os.system，因为我们用的播放音频是mp3格式
    # def play_audio(self, path):
    #     try:
    #         if os.path.exists(path):
    #             # 在Linux中执行该句指令
    #             os.system(f"aplay {path}")
    #         else:
    #             rospy.logwarn(f"Audio file not found: {path}")
    #     except Exception as e:
    #         rospy.logerr(f"Error playing audio: {e}")

    #雷达回调函数，暂时先不用
    # def process_laser_data(self):
    #     try:
    #         if self.laser_process != 1 or self.g_laser_scan is None:
    #             return
    #         # ... 省略内部处理代码 ...
    #         self.laser_process = 0
    #     except Exception as e:
    #         rospy.logerr(f"Laser data processing error: {e}")

    #不知道是啥玩意
    # def image_callback(self, msg):
    #     try:
    #         if not self.xun_flag:
    #             return
    #         # ... 图像处理及PID控制逻辑 ...
    #     except Exception as e:
    #         rospy.logerr(f"Image processing error: {e}")

    def main(self):
        rospy.loginfo("Waiting for awake_flag...")
        while not rospy.is_shutdown() and not self.awake_flag:
            #这一步进行睡眠等待时会把标志位给更新过来吗？
            rospy.sleep(0.1)

        # start_detect_pub.publish(msg_pub) 我们不采用分多次进行消息的组织，而采用将这个过程抽象为一个接口，我们每次发布消息时调用这个接口进行传参且同时进行反复发送确保消息能被传递到
        # self.publishMessageAndSleep(start_detect_pub, msg_pub)
        #或用直接书写的形式，如下代码所示，用于休眠0.2秒每次，一共休眠1秒
        #这一步的目的是为了让摄像头先启动并进行初始化，同时发布多次消息是为了确保摄像头能够准确的接收到
        #摄像头一旦启动便会一直运行，我们的start_detect_pub.publish发布者只是用于进行不同任务的识别
        for i in range(5):
            self.msg_qr_code.data = 1
            self.start_detect_pub.publish(msg_qr_code)
            rospy.sleep(rospy.Duration(0.2))


        #duration是否需要在__init__中定义呢？
        #下述两行代码用于只提供“小飞小飞”直接进行启动的情况，用于等待3s
        duration = rospy.Duration(3)
        rospy.sleep(duration)

        #等待导航服务端启动（即等待move_base的路径规划接口启动）
        self.ac.wait_for_server()
        rospy.loginfo("server start success!!!")
        # rate = rospy.Rate(1)
        # max_retry = 3

        #暂时把while和捕获异常去掉了，感觉没必要，且会导致程序后续出错
        #index用于表示第几个目标点

        # # 发布者统一创建
        #
        # self.start_xx_pub = rospy.Publisher("xun_start", Int8, queue_size=1)  # 巡线开启的发布者
        # self.start_yd_pub = rospy.Publisher("start_yd", Int8, queue_size=1)  # 开始雷达检测的发布者
        # self.start_detect_pub = rospy.Publisher("start_detect", Int8, queue_size=1)  # 开始目标检测的发布者 为1表示开始检测，为0表示尚未开始
        #
        # # 订阅者统一创建
        # # self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # self.mic_sub = rospy.Subscriber("awake_flag", Int8, self.mic_awake_cb, queue_size=1)  # 麦克风订阅者，用于接收麦克风标志位
        # # self.laser_flag_sub = rospy.Subscriber("laser_scan", Int8, self.laser_flag_cb,queue_size=1)
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)  # 用于订阅里程计消息用于定位或者获取当前的速度
        # # self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback,queue_size=1)
        # # self.cam_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback,queue_size=1)
        # self.xunxian_sub = rospy.Subscriber("xun_xian", Int8, self.xun_sub_cb,
        #                                     queue_size=1)  # 巡线代码的订阅者，用于订阅视觉巡线结束后获取标志位，标志着视觉巡线的完成，然后进行终点的语音播报
        # self.cv_sub = rospy.Subscriber("cv_msg", String, self.cv_msgs_callback, queue_size=1)  # 视觉识别的结果，用于输出对应的语音
        # self.yd_sub = rospy.Subscriber("yd_msg", PoseArray, self.yd_msgCallback, queue_size=1)  # 雷达的回调函数订阅者，用于处理雷达信息

        #这边需要考虑index要不要加self
        #python中的循环变量可以是本地的全局变量吗？？？这样的话就可以直接修改，在代码中控制循环流程了
        for index in range():
            #发送目标点的工作统一到了if判断的外面，让任务更加专一化，同时进行代码的简化
            self.send_goal(self.goal_list[index])#这边index的引用也要考虑有没有问题！！！
            rospy.loginfo("I send goal %d", index)  # 不知道这段代码应用for循环中的循环变量index有没有用，或者说有没有错？

            # 如果指定了时间，比如 ac.wait_for_result(rospy.Duration(10.0))，那么最多等 10 秒；
            # 如果不传入参数（如 ac.wait_for_result()），它会一直等，直到有结果为止（阻塞等待）
            #对于ac.wait_for_result()来说，它会等待 action 服务器执行完目标任务（goal）并返回结果，且正常执行则返回True并继续下去，若并没有到达目标点则返回False，一直阻塞，不会继续接下来的代码
            #所以我们在这里要设置一个大概的时间，这个时间即为我们一开始到第一个目标点的大概时间，若是提前完成可提前结束，就算10秒内不能完成。也能让程序继续下去
            #所以我们在这里是不能阻塞写死的

            ac.wait_for_result(rospy.Duration(10.0))

            #这段代码判断机器人是否成功到达目标点。如果状态是 SUCCEEDED，就说明任务完成，打印 "success"
            #这段代码相当于上述阻塞接口的一个二重保障了，因为接下来的主程序都是嵌套在到达了目标点后再继续下去的，这样就能让代码在可控范围内（安心的感觉（笑））
            if ac.get_state() == GoalStatus.SUCCEEDED:#这代代码准确来说并不是很严谨，直接使用actionlib_msgs/GoalStatus中的消息状态判断会更严谨
                #举例：如下述代码
                # from actionlib_msgs.msg import GoalStatus
                # if ac.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("arrive next goal success!!!")
                #index前都无需加入self。因为是循环变量
                if index == 0:
                    for i in range(5):
                        self.detect_for.data = 1#参数变量名还需要修改规范！！！
                        start_detect_pub.publish(detect_for)#进行二维码检测 检测完后接收到视觉端的返回值，进行语音播报
                        rospy.Duration(0.2)#到时候得考虑这个等待是否有必要？？？

                    #订阅者创建后就会一直运行，并且再改目标点进行语音播报,所以貌似主程序中并不需要给订阅者再组织什么命令了
                rospy.Duration(5.0)#暂时设置为5秒钟，实际应该考虑语音播报的时间已经延迟时间
                    #或许不需要进行停车标识了，感觉好像太繁琐，而且好像实现不了
                    # #stop_flag初值为0
                    # while not self.stop_flag:
                    #     pass
                    # self.stop_flag = 0


                #拣货区物资识别
                if index == 1:
                    #发送目标点已经在send_goal函数中进行了封装，故我们直接调用函数即可，且由于它不是回调函数，可以随时被调用
                    #我只需传入目标点进行初始化即可 目标点已经重新进行了逻辑的组织，无需再返回发送了
                    # self.send_goal(self.goal_list[1])
                    #发布者创建了也只是创建了一个接口，真正意义上若要使用则需组织消息并用发布者发布

                    #前面组织过了，且并没有重新置零，没有必要再次组织消息了
                    # msg_data = 1
                    # msg_pub.data = msg_data

                    self.publishMessageAndSleep(start_detect_pub, msg_pub)
                    start_time = rospy.Time.now()
                    while rospy.Time.now() - start_time < rospy.Duration(3.14):
                        ro_pub.publish(twist)
                        start_detect_pub.publish(msg)
                        rospy.spinOnce()  # rospy 中不需要这个，保留可兼容其他回调
                        if human_num == tool_kind:
                            find_flag = 1
                            break
                        loop_rate.sleep()
                    rospy.sleep(3.5)

                # if self.index == 2:
                #     if self.terrorist_count > 0:
                #         self.play_audio(self.sound_file)
                #         rospy.sleep(3)
                #
                # if self.index == 3:
                #     self.send_goal(self.goal_list[2])
                #
                # if self.index == 4:
                #     self.send_goal(self.goal_list[3])
                #     self.detect1_pub.publish(Int8(data=1))
                #     rospy.sleep(3.5)
                #
                # if self.index == 5:
                #     if self.aid_kit_flag > 0:
                #         self.play_audio("b1.wav")
                #         rospy.sleep(3)
                #
                # if self.index == 6:
                #     self.send_goal(self.goal_list[4])
                #     self.index += 1
                #
                # if self.index == 7:
                #     self.send_goal(self.goal_list[5])
                #     self.detect2_pub.publish(Int8(data=1))
                #     rospy.sleep(3.5)
                #     self.index += 1

                #完成视觉识别后，进行仿真区域的停泊，并设置时间让语音播报完成
                #是否有这么一个指令，他可以等待仿真时间结束后进行返回值，与返回值匹配后若正确再进行语音播报，而不是
                #其实根本不需要按照上面的思路去考虑，我们只需要让仿真段结束后再进行消息发布，我们接收到之后自然就会进行语音播报了，但是这样就需要我们在现实段需要有一个等待到仿真段结束后再进行后续目标点发布的操作功能
                #利用多个进程需要

                #这里我想到一种思路：可以在语音播报完成的同时对于某个全局变量赋值为1，然后主程序中检测到这个全部变量为1时，才继续下去，否则一直在while中循环
                #这里不必定义多个flag 直接定义一个用于停车判断的stop_flag即可，跳出循环后需要对其重新置零
                #这样连延迟可能都不要了

                if index == 8:
                    self.send_goal(self.goal_list[6])
                    index += 1


                    while not self.stop_flag:
                        pass
                    self.stop_flag = 0
                #
                # if self.index == 9:
                #     if self.laser_flag == 1:
                #         self.laser_process = 1
                #         for _ in range(50):
                #             if self.laser_process == 0:
                #                 break
                #             self.process_laser_data()
                #             rospy.sleep(0.1)
                #         self.send_goal(self.lasergoal.target_pose.pose)
                #     self.index += 1
                #
                # if self.index == 10:
                #     self.send_goal(self.goal_list[7])
                #     self.index += 1



                #识别路口红绿灯
                if index == 11:
                    # rospy.loginfo("I send goal %d", index)
                    # self.send_goal(self.goal_list[8])
                    self.detect_for.data = 1  # 参数变量名还需要修改规范！！！
                    start_detect_pub.publish(detect_for)  #进行路口红绿灯识别 检测完后接收到视觉端的返回值，进行语音播报
                    if self.color == "green":
                        os.system("mpg123 /home/your_user/xxx.mp3")
                    elif self.color == "red":
                        self.index += 1
                        self.send_goal(self.goal_list[self.index])  # 这是另一块板前 这个脚标肯定有问题，需要进行修改
                        ac.wait_for_result(rospy.Duration(3.0))#等待到达下一个目标点
                        if ac.get_state() == GoalStatus.SUCCEEDED:
                            #这边毕竟是同一个进程就没有必要再进行语音时间以及消息订阅时间的等待了吧？？？
                            # duration = rospy.Duration(2)
                            # rospy.sleep(duration)
                            os.system("mpg123 /home/your_user/xxx.mp3")
                    else:
                        rospy.loginfo("Waiting for msg")


                #进行后续巡线
                # 视觉巡线总控需要干的事情如下
                if index == 12:
                    # rospy.loginfo("I send goal %d", index)
                    # self.send_goal(self.goal_list[9])
                    #下述两行消息的组织貌似也有问题，需要进行修改！！！
                    msg_data = 1
                    msg.data = msg_data
                    for i in range(5):
                        start_xx_pub.publish(msg)#开始进行视觉巡线
                        #关于视觉巡线中是否需要关闭雷达，识别，虚拟仿真等的发布者，需要具体结合实际情况来看，若我们是那种发布一次对面就一直在运行的则需要进行发布关闭，若不是，则无需进行发布关闭
                        #但是，因为由于进行目标板识别时摄像头大概率是要一直开着的，故其实具体也不是很确定是否视觉识别部分是一直开着的
                        rospy.Duration(0.2)
                    self.index += 1

                #终点语音播报
                #对于终点语音播报似乎并不需要写代码，因为该段的语音播报与总程序并不是一个进程，这边也得和视觉的说一下同一个发布者发布消息最好多发几次，否则怕接收不到！！！
                if index == 13:
                    self.send_goal(self.goal_list[10])
                    self.index += 1

                if index == 14:
                    self.send_goal(self.goal_list[11])
                    self.xun_flag = 1
                    self.xun_pub.publish(Int8(data=1))
                    rospy.loginfo("Line following activated")
                    self.index += 1

                rospy.loginfo("Main sequence completed.")




        # 其他函数保留（如 mid, is_spike, send_all_goals 等）

if __name__ == '__main__':
    rospy.init_node("zongkong_p")
    controller = RobotController()
    controller.main()
    rospy.spin()
