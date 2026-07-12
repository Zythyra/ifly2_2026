# amcl的参数配置
## 1.初始化位姿
#### 参数列表：
  param name="initial_pose_x" value="100.0"/
  
  param name="initial_pose_y" value="0.0"/
  
  param name="initial_pose_a" value="0.0"/
  
  param name="initial_cov_xx" value="0.1"/
  
  param name="initial_cov_yy" value="0.1"/
  
  param name="initial_cov_aa" value="0.1"/
  
在amcl节点启动后，会默认初始位姿为上面参数设定的值，缺点是只能预设一次

#### 发布/initialpose
   修正初始位姿：如果在AMCL使用launch文件中的参数进行初始化后，您发现机器人的定位不准确，或者粒子云没有收敛到正确的位置，您可以通过Rviz的"2D Pose Estimate"工具（它会发布到 /initialpose 主题）或手动发布一个 /initialpose 消息来重新设定或修正机器人的位姿。这个新的位姿会覆盖AMCL当前的粒子分布。
    机器人丢失定位后重新定位：如果机器人在运行过程中因为某些原因（例如，被抬起、传感器暂时失效、进入特征稀疏区域等）丢失了定位，您需要发布一个新的 /initialpose 消息来帮助AMCL重新找到机器人在地图上的位置。
    在不同已知位置启动：如果您的机器人可能在地图上的多个不同已知位置启动，而您不想为每个位置都创建一个单独的launch文件，那么您可以在启动通用的AMCL launch文件（可能不包含精确的 initial_pose_* 参数，或者包含一个大致的默认值）后，再根据实际启动位置发布一个精确的 /initialpose 消息。
    动态调整初始位姿：在某些自动化脚本或更高级的系统中，可能会根据外部逻辑动态计算并发布初始位姿。
