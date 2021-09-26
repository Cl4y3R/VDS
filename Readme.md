# LGSVL_ROS2_C++
2021.9.16  
Vehicle Dynamics Simulation with LGSVL_ROS2_C++  

教程:  
Part1:https://zhuanlan.zhihu.com/p/392768756  
Part2:https://zhuanlan.zhihu.com/p/393079410

结果记录：成功完成教程Part1的内容，没有问题。

2021.9.17  
欠缺技能：
ROS2运用，CMake使用
下一步需要弄清楚的内容：
如何获得车辆数据，如何发送控制指令

2021.9.22  
C++教程地址：https://light-city.club/sc/

2021.9.26  
计划分为两个部分，一部分为C++的控制算法实现，白天做；另一部分为ROS2的通信实现，晚上做。  
需要解决的内容具体如下：  
1.C++部分首先目标是使用最简单的空旷平地的路径规划和横向LQR，纵向目标为简单PID，算法代码的结构应该符合基本的结构要求。  
2.ROS2部分要搞清楚如何获取车辆状态和环境(地面)状态，使得控制算法能获取指定格式的数据；弄清楚如何将控制算法计算出来的控制量转化为车辆的输入，实现发布信息的通信。  
根据教程Part1的内容，已经实现车辆的控制，下一步需要设计算法控制输出。目前给的输出是定值。  
是否可以用single lane地图设计一个双移线？
