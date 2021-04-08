@[TOC](Cartographer总结(二）-代码框架及算法流程)
1.	本文从/scan传递的信息流帮助大家找到算法的入口，在找算法入口的同时可以帮助理解代码框架。在ROS环境中跑cartographer一般包含cartographer和cartographer_ros两个库，cartographer是算法库，用于实现3D和2D情况下的SLAM算法，cartographer_ros是对cartographer进行了一次打包，使得cartographer能够在ROS的环境下使用。cartographer代码框架详细总结有很多博客，本文将不再详细描述，可以参考[代码框架总结链接](https://blog.csdn.net/zouyu1746430162/article/details/73289745?utm_source=blogxgwz1)。下图是我对代码中几个重要的类做的总结，括号内是类名对应的头文件位置，可以一层一层找到对应类，比如node实例中包含了map_builder_bridge_和extrapolators_2个重要的类实例，map_builder_bridge_包含了map_builder_重要的类。lua文件和类都是对应的，一般1个{}对应1个类。
 ![代码框架](https://img-blog.csdnimg.cn/20210301114955108.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTI0NTk4OA==,size_16,color_FFFFFF,t_70#pic_center)

2.	源码中运用了很多C++的特性。比如继承、虚函数、lambda、将函数作为参数传递等，其中= delete、std::move()等都是为了使得代码更加科学严谨，效率更高、更难出bug。一个比较重要的特性是std::function（函数）当成变量使用，lambda还能对函数再一次打包后可以赋给std::function变量，注意在赋值时函数不执行，当std::function变量被使用时函数才执行。
3.	在大概理清各类的关系后，通过/scan的信号流便能找到算法的入口，接下来以2D为例介绍/scan的流向。
1)node(cartographer_ros/node.h)-HandleLaserScanMessage--类实例名(类所在头文件)-类函数：初始化完成后，订阅了/scan消息，该函数为ROS自动调用的函数，不对数据做任何处理
2)sensor_bridge(cartographer_ros/sensor_bridge.h)-HandleLaserScanMessage：将ROS格式数据转换成carto定义的数据格式。首先，根据角度和距离将一帧数据中所有点转换到以激光雷达中心为原点的坐标系中，以点云的形式表示，将帧的时间戳换成最后一束激光的时间，ROS一般以第一束激光作为帧的时间戳。
3)sensor_bridge(cartographer_ros/sensor_bridge.h)-HandleLaserScan：将一帧数据分成back_pack_2d.lua-> num_subdivisions_per_laser_scan个子点云
4)sensor_bridge(cartographer_ros/sensor_bridge.h)-HandleRangefinder:从/tf中读取激光雷达在机体（back_pack_2d.lua->tracking_frame）坐标系（有IMU时一般取IMU坐标系，只有Lidar时就取Lidar坐标系）的位姿（外参），将所有点转换到机体坐标系
5)trajectory_builder_(cartographer/mapping/internal/collated_trajectory_buider.h)-AddSensorData：将数据转换成继承于Data的模板类dispatchable，IMU、编码器等传感器数据都可以用该模板类表示。至此，数据流cartographer_ros进入到cartographer，该模板类像是将不同传感器数据流合流，用一个模板表示。
6)trajectory_builder_(cartographer/mapping/internal/collated_trajectory_buider.h)-AddData：所有传感器数据都会进入这个函数，不对数据作处理
7)sensor_collator_(cartographer/sensor/internal/collator.h)-AddSensorData：得到queue_key{trajectory_id,sensor_id}, trajectory_id指明第几条轨迹，sensor_id指明是什么类型的多少号传感器，若一种类型的传感器只有一个，则就是ROS消息中的frame_id，不对数据作处理。
8)queue(cartographer/sensor/intenal/ordered_muti_queue.h)-Add：添加数据到queues_中对应queue_key，不对数据作处理
9)queue(cartographer/sensor/intenal/ordered_muti_queue.h)-Dispatch：在一个死循环处理函数，直到提取完已经收到的各传感器数据，不对数据作处理。该函数体里会调用queues_->callback(),callback是一个函数变量，在构造trajectory_builder_时将lambda函数赋给它，而trajectory_builder_的成员函数HandleCollatedSensorData是lambda函数的函数体部分，故程序运行时，下一步会调用HandleCollatedSensorData。
10)trajectory_builder_(cartographer/mapping/internal/collated_trajectory_buider.h)-HandleCollatedSensorData：打印"rate:",不对数据作处理
11)data(cartographer/sensor/internal/dispatchable)-AddToTrajectoryBuilder：不对数据作任何处理
12)wrapped_trajectory_builder_(cartographer/mapping/internal/global_trajectory_builder.cc)-AddSensorData：至此数据流才算进入算法的主体。进入[官方链接](https://google-cartographer.readthedocs.io/en/latest/)中的算法流程图，如下图
![算法框架](https://img-blog.csdnimg.cn/20210301114938202.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTI0NTk4OA==,size_16,color_FFFFFF,t_70#pic_center)

13)local_trajectory_builder_( mapping/internal/2d/local_trajectory_buider_2d.h)-AddRangeData：去运动畸变（首先通过extrapolator预测每一束光束得到的点在世界坐标系位置并记录此刻Lidar中心在世界坐标系位姿，最后将所有点转换到最后一束激光打出去时刻对应Lidar坐标系中，相当于用PoseExtrapolator得到每束激光对应坐标系与最后一束光的相对位姿，要求PoseExtrapolator比Lidar高频且准确）、重力对齐、体素滤波(Voxel Filter)、拼接trajectory_builder_2d.lua->num_accumlated_range_data个3)中分出的子点云（我想这也是为什么要用最后一束光束作当前子点云时间戳的原因，子点云最后一束时间戳也是累积点云最后一束激光的时间戳，反之，若用子点云第一束光束时间戳，那么它并不是累积点云的第一束激光的时间戳）
14)local_trajectory_builder_( mapping/internal/2d/local_trajectory_buider_2d.h)-AddAccumulatedRangeData：自适应体素滤波(Adaptive Voxel Filter，在满足min_num_points情况下，使体素滤波尺寸尽可能大且不超过max_length)、Scan Matching(在ceres scan matching前可以通过暴力匹配得到更好的初值)、运动滤波(Motion Fiter,时间、距离和角度中有一个超出设定值便插入关键帧，否则Dropped)、插入关键帧更新active_submap(由2个submap组成)、得到IserttionResult
15)pose_graph_(mapping/internal/2d/pose_graph_2d.h)-AddNode：插入关键帧、计算约束、后端优化（Global SLAM）
16)wrapped_trajectory_builder_(cartographer/mapping/internal/global_trajectory_builder.cc)-local_slam_result_callback_：该函数也是被一个Lambda函数赋值，Lambda函数的函数体调用了map_builder_bridge_(cartographer_ros/map_builder_bridge.h)-OnLocalSlamResult,这样cartographer算法库得到的结果回传到cartographer_ros。

