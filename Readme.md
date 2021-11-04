# LGSVL_ROS2_C++
2021.9.16
Vehicle Dynamics Simulation with LGSVL_ROS2_C++

## 教程:  
Part1:https://zhuanlan.zhihu.com/p/392768756
Part2:https://zhuanlan.zhihu.com/p/393079410

结果记录：成功完成教程Part1的内容，没有问题。

## 2021.9.17
### 欠缺技能：
ROS2运用，CMake使用
下一步需要弄清楚的内容：
如何获得车辆数据，如何发送控制指令

### 2021.9.22
C++教程地址：https://light-city.club/sc/

### 2021.9.26 
计划分为两个部分，一部分为C++的控制算法实现，白天做；另一部分为ROS2的通信实现，晚上做。  
需要解决的内容具体如下：
1. C++部分首先目标是使用最简单的空旷平地的路径规划和横向LQR，纵向目标为简单PID，算法代码的结构应该符合基本的结构要求。
2. ROS2部分要搞清楚如何获取车辆状态和环境(地面)状态，使得控制算法能获取指定格式的数据；弄清楚如何将控制算法计算出来的控制量转化为车辆的输入，实现发布信息的通信。 
根据教程Part1的内容，已经实现车辆的控制，下一步需要设计算法控制输出。目前给的输出是定值。
是否可以用single lane地图设计一个双移线？

### 2021.10.27
经过多个测试，暂时还是回归ROS2直接自己写控制算法。现在已可以连通ROS2和LGSVL，需要搞明白msg的通信用法。第一步要搞清楚通信机制，搞定信息获取和发出的方法！  
在运行lgsvl_bridge 和 ros2 run autodriving main_msg后，输入ros2 topic list，可以看到有哪些订阅的消息，用ros2 topic echo xxx就可以看到内容。用rosbag录制的命令ros2 bag record xxx，最好现cd到建好的文件夹内录制。录制多个则为ros2 bag record -o subset xxx yyy。其中 -o参数是用来设置数据库文件名的，此时文件名叫subset。查看则为：ros2 bag play subset.  
ros2 topic echo xxx可以实时看到topic内消息！

### 2021.10.28
列举所需要的车辆状态参数
1. 车速Vx
2. 加速度ax,ay
3. 横摆角速度phi_p
4. 车轮转角delta

### 2021.11.3
修改了cpp代码！增加了imu的内容，成功build并运行，获得了imu数据！  
需要注意的是订阅的topic必须在lgsvl的sensor里面进行设置相同的topic！  
新的发现：lgsvl_msg,sensor_msg的内容都在/opt/ros/foxy/include内，可以看到各种源码定义！  

### 2021.11.4
再次修改cpp代码，发现需要进一步学习ros2的内容，研究传感器信号msgs。  
现在加入了对imu、canbus信号的读取，可以用RCLCPP_INFO()实时显示。
