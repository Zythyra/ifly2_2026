#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

class PnPPoseEstimator:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('pnp_pose_estimator', anonymous=True)
        
        # 创建发布者和订阅者
        self.bridge = CvBridge()
        self.pose_pub = rospy.Publisher('/object_pose', PoseStamped, queue_size=10)
        self.control_pub = rospy.Publisher('/pnp_control_params', Float32MultiArray, queue_size=10)
        self.image_sub = rospy.Subscriber('/ucar_camera/image_raw', Image, self.image_callback)

        # 定义3D物体点 (Y轴向下，单位：米)
        self.object_points = np.array([
            [0.0, 0.0, 0.0],  # 左上
            [0.5, 0.0, 0.0],  # 右上
            [0.5, 0.3, 0.0],  # 右下
            [0.0, 0.3, 0.0]   # 左下
        ], dtype=np.float32)

        # 相机内参 (示例值，实际需标定)
        self.camera_matrix = np.array([
            [612.7148, 0.0, 616.11004],
            [0.0, 611.02227, 346.95483],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

        # 畸变系数
        self.dist_coeffs = np.array([-0.262284, 0.048112, 0.000962, 0.003192, 0.0], dtype=np.float32)

        # 预计算去畸变映射
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix, self.dist_coeffs, None, 
            self.camera_matrix, (1280, 720), cv2.CV_32FC1)

    def image_callback(self, msg):
        try:
            # 1. 转换ROS图像为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 2. 去畸变处理
            undistorted_img = cv2.remap(cv_image, self.map1, self.map2, cv2.INTER_LINEAR)
            
            # 3. 角点检测 (这里使用硬编码坐标，实际应使用角点检测算法)
            corners = np.array([
                [614.38, 72.32],  # 左上
                [951.33, 69.36],  # 右上
                [951.33, 272.32], # 右下
                [622.27, 265.42]  # 左下
            ], dtype=np.float32)

            # 4. 亚像素级优化
            gray = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            )

            # 5. PnP位姿估计
            ret, rvec, tvec = cv2.solvePnP(
                self.object_points, corners,
                self.camera_matrix, None,
                flags=cv2.SOLVEPNP_EPNP
            )

            if not ret:
                rospy.logwarn("PnP求解失败")
                return

            # 6. 计算控制参数
            z_distance = float(tvec[2])  # Z轴距离 (米)
            yaw_angle = float(np.arctan2(rvec[1][0], rvec[0][0]))  # 偏航角 (弧度)
            
            # 创建控制参数消息
            control_msg = Float32MultiArray()
            control_msg.data = [z_distance, yaw_angle]
            self.control_pub.publish(control_msg)
            
            rospy.loginfo(f"控制参数发布 - 距离: {z_distance:.3f}m, 偏航角: {np.degrees(yaw_angle):.2f}°")

            # 7. 发布位姿消息
            R, _ = cv2.Rodrigues(rvec)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "camera"
            pose_msg.pose.position.x = float(tvec[0])
            pose_msg.pose.position.y = float(tvec[1])
            pose_msg.pose.position.z = float(tvec[2])
            q = self.rotation_matrix_to_quaternion(R)
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]
            self.pose_pub.publish(pose_msg)

            # 8. 可视化 (可选)
            # self.visualize_results(undistorted_img, corners, rvec, tvec)

        except Exception as e:
            rospy.logerr(f"图像处理错误: {str(e)}")

    def rotation_matrix_to_quaternion(self, R):
        """将旋转矩阵转换为四元数"""
        q = np.zeros(4)
        q[3] = np.sqrt(1.0 + R[0,0] + R[1,1] + R[2,2]) / 2.0
        q[0] = (R[2,1] - R[1,2]) / (4 * q[3])
        q[1] = (R[0,2] - R[2,0]) / (4 * q[3])
        q[2] = (R[1,0] - R[0,1]) / (4 * q[3])
        return q / np.linalg.norm(q)  # 归一化

    def visualize_results(self, img, corners, rvec, tvec):
        """可视化位姿估计结果"""
        # 绘制坐标系轴
        axis_points = np.float32([[0.1,0,0], [0,0.1,0], [0,0,-0.1]])
        imgpts, _ = cv2.projectPoints(
            axis_points, rvec, tvec,
            self.camera_matrix, None
        )
        imgpts = np.int32(imgpts).reshape(-1, 2)
        cv2.line(img, tuple(imgpts[0]), tuple(imgpts[1]), (0,0,255), 3)  # X轴 (红)
        cv2.line(img, tuple(imgpts[0]), tuple(imgpts[2]), (0,255,0), 3)  # Y轴 (绿)
        
        # 绘制角点连线
        cv2.polylines(img, [np.int32(corners)], True, (255,255,0), 2)
        
        # 显示图像
        cv2.imshow("PnP Result", img)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        estimator = PnPPoseEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()