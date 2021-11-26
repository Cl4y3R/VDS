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

### 2021.11.5
从lgsvl中导出opendrive格式的地图，研究一下这种格式的地图可否获取有效信息以得到规划的基本参数。  

### 2021.11.7
OpenDrive里，道路参考线（一般是道路的中心线）用来描述道路在俯视下的走向，是OpenDRIVE中每条道路的基本元素。那么在XML文件中，OpenDRIVE用<road>元素里的<planView>元素来表示道路参考线。参考线的几何形状用<planView>里的<geometry>元素来表示。注意：由于道路的原因，一般不可能用一条曲线（一个几何形状）来完全表述整个道路，所以在<planView>元素里，一般都包括几个<geometry>元素，它们一定是前后相连接（s值）无重叠的。  
至于每种几何形状的元素表示我们前面已经讲过了，直线是<line>，参数三次多项式是<paramPoly3>。下面是其他属性的介绍：  
-s 该段曲线起始位置的s坐标 （参考线坐标系）  
-x 该段曲线起始位置的x值 （惯性坐标系）  
-y 该段曲线起始位置的y值 （惯性坐标系）  
-hdg 该段曲线起始点的方向 （惯性航向角/偏航角 yaw）  
-length 该段曲线的长度 （参考线坐标系）  

**注意**：每条道路必须有且只有一条道路参考线。

### 2021.11.8
注意：create_wall_timer调用的回调函数不能包含argument

### 2021.11.16
今天发现publisher callback调用subscriber的内容时，停止循环发布了，需要解决这个问题。

### 2021.11.17
**重大消息！**发现之前没有成功的原因应该是没有用一个中间变量保存imu_msg，因此导致msg内容丢失无法传到  
pulisher去(有待检验确认)。现在已经可以成功的根据subscriber的msg传输到publisher进行发布，下一步  
是进行反馈控制！  
小问题待解决：lgsvl传出来的msg数值需要检查！  

### 2021.11.18
搞定了一些通讯msg，需要注意单位！！  

### 2021.11.19
对于ros2内用到的mgs结构，根据名称找到opt/ros/foxy/include内不同msg包的detail内xxx_struct.h，内有  
结构体定义，若有嵌套，即表示直接应用了嵌套的结构体定义。层层找下去直到找到变量类型。  
不要忘记添加msg包到CMakeLists.txt和package.xml里面！！！！  

### 2021.11.23
神奇的事情需要确认：gps传感器选择ignore map origin则会实时输出unity左手坐标系下的坐标。目前对比  
opendrive看，x=-pose.pose.position.y,y=pose.pose.position.x。咋回事呢？  

### 2021.11.24
下一步要做的事情：假设之前的x,y坐标获取的正确的话，就需要根据opendrive的文件生成reference line。  
再对reference line进行平滑处理，生成path。对于reference line生成，建议不考虑routing算法，建议  
针对某些地图进行离线处理保存起来，再实时读取。  
opendrive parampoly3 曲线计算：  
u local = a u + b u * p + c u * p^2 + d u * p^3
v local = a v + b v * p + c v * p^2 + d v * p^3  
思考问题：是否选择在平面上仿真，不使用opendrive的道路，自己设定一个reference line？可视化问题用  
rviz解决。这样的话可以免去路径生成的麻烦，快速进行控制算法的学习。但是需要评估平面上坐标的问题。  
新发现：lgsvl导出的autoware格式地图文件，point.csv保存了路径点！完成了路径点读取，下面需要进行  
参考线平滑并生成合适的路径点。**思考**：是否可以先用直接读取的路径点进行简单仿真？

### 2021.11.25
发现：四元数转欧拉角有ros2的tf2，下一步进行学习使用。  
若使用直接读取的路径点，现在还需要处理的变量如下：  
yaw, theta_ref, kappa_ref  
搞定之后可以进行横向控制，此时可以先只设计横向控制，纵向控制直接用键盘！

### 2021.11.26
成功使用tf2将quaternion转换为euler获得yaw。目前车辆数据方面已经完备，下面需要计算theta_ref和kappa_ref了。
