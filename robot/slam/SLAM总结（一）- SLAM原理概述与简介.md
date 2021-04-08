@[TOC](SLAM总结（一）- SLAM原理概述与简介)

 1. SLAM（Simultaneous Localization and Mapping）：同时定位和建图，定位是定位机体在世界坐标系下的位姿（pose、transformation）。单传感器机体一般指相机光心、激光雷达scan中心、IMU中心、编码器两轮轴心，多传感器一般使用IMU中心，可以避免离心力的影响。位姿包括3个自由度的位置（translation）和3个自由度的姿态（rotation）。世界坐标系的原点和姿态可以由第一帧关键帧、已有的全局地图或路标点、GPS（全世界一样，真正意义的世界坐标）得到。建图是建立机器人所感知周围环境的地图，地图基本几何元素是点，点没有方向，只有3个自由度的位置。可以是稀疏点、稠密点、栅格地图、八叉树地图和拓扑地图等。地图的主要作用是定位、导航，导航可拆为导和航，导包括全局规划和局部规划，航即规划完后控制机器人运动。稀疏点一般只能用于定位；栅格地图、八叉树地图可用于定位、导航，稠密地图可以用于定位，处理后可转换成栅格地图或八叉树地图用于导航。
总之，常见的SLAM问题是估计由大量离散的6自由度的机体pose（定位）和3自由度点（建图）构成的n维变量。那么时间和运动范围共同造成n增大，从而造成CPU和内存开销变大。拓展一下，把“同时定位和建图”改成“定位或建图”，那么可以分为下面几种情况：
1）	机体pose和点完全未知：这种情况便属于SLAM问题
2）机体pose完全已知和点完全未知：这便是纯建图问题，比如在ORB-SLAM2跑的过程中把关键帧对应图片保存下来，跑完后进行一次全局优化，此时得到完全已知的pose，再利用已知pose和图片建图。
3)机体pose完全未知和点完全已知：这便是纯定位问题。
4）机体pose完全未知和点部分已知：这些点一般可以作为路标使用，认为它们是没有累积误差的，可以利用它们使得每次开机都在同一世界坐标系下和减小累积误差。
5）理论上有3*3=9中情况，其他情况不太常见就不一一例举

 2. SLAM框架如下图所示，整个系统由前端、后端和回环检测组成。
 ![SLAM框架](https://img-blog.csdnimg.cn/20210218173656943.png#pic_center)
1）前端：频率为传感器的帧率。数据关联（如特征点匹配、光流法等）、初始化、通过几何方法或很小范围优化快速得到当前帧的机体pose的较准确的初值（跟踪，Tracking）。当前帧一般只参照前一帧或前几帧，故随着关键帧增多累积误差会变大。
2）	后端：频率为关键帧率，关键帧需要在保证跟踪质量的同时尽量减少帧的数量，帧的时间间隔主要与机器人的运动线速度、角速度（速度越快更容易跟踪丢失）、视野（距离太近更容易跟踪丢失）和环境中的特征特征结构（特征点稀疏和拐角更容易丢失）有关。
功能是通过三角化得到新的点在世界坐标系下较准确的位置初值、剔除或合并一些旧的点、添加约束和优化较大范围的局部窗口中的关键帧pose和点位置。可以较长一段时间进行一次全局优化。
3）	回环检测：频率不确定，一般远远低于前两个，与回环数量有关，会设置最高频率使得不在短距离内检测闭环。通过数据关联找到之前到过的地方。发现回环后会先使用相似变换的方法（3D-3D）调整与闭环帧关联的关键帧的位姿，然后优化回环内的所有关键帧和点，最后再进行一次全局优化。相对于普通的全局优化，闭环检测后的全局优化更容易收敛。
4）	建图：一般地，通过前几步可以得到稀疏点地图。根据不同需求可以建立其他格式地图。
 4. SLAM分类按方法分类如下图，本人的主要研究的是基于关键帧的几何特征法，对多传感器和多几何特征的融合、增量式非线性优化兴趣比较大，以下内容主要讨论基于非线性优化的几何特征法。
 ![SLAM分类](https://img-blog.csdnimg.cn/20210218174142841.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTI0NTk4OA==,size_16,color_FFFFFF,t_70#pic_center)

 5. 前端：常用的视觉几何特征是点，但最近几年对线、面和物体的研究越来越多。一些人致力于将它们用一个统一的数学表达式表达出来，便于处理；一些人致力于给物体加上语义，结合生活实际赋予它们特性，比如人是经常动的、不能撞的；一些人致力于估计动态物体的pose和运动速度。这些是我们生活中常见的几何特征，加以利用势必提高算法的鲁棒性和精度，给物体赋予语义也会使得机器人变得更加智能。
 6. 后端：对于非线性优化，关键是目标函数（约束、残差项）、目标函数对变量的Jacobi矩阵和增量的求解方法。增量的求解一般使用LM方法。最后转化成了求解线性方程AX=b，求解方法包括CSparse、Cholesky、Preconditioned Conjugate Gradient(PCG)-CG改进、QR，求解之前还可以使用schur、plain、reording等方法对系数矩阵A进行处理，加快计算速度
 7. 拓展
1）动态SLAM：SLAM一般假设环境是静止的，那么当空间中存在动态物体时，势必打破这一假设。当动态物体占据传感器数据较少一部分时，数据关联、RANSAC、优化的鲁棒核滤去动态物体，因为相对于静态点来说，动态物体上的点是outlier（外点）。但动态物体占据传感器的大部分时，动态点更像是inlier（内点），以上方法无法将动态物体滤去。从而动态SLAM应运而生，顾名思义，也就是在动态环境中运行的SLAM算法。它除了估计静态点的3D位置外，还能估计动态点的3D位置和速度。其中，SLAM_MOT（Muti-object tracking）比较具有代表性，在自动驾驶汽车上应用较多，它除了完成SLAM任务外，还能跟踪环境中的动态物体。
2)语义SLAM：语义SLAM一般会与CV中的语义分割、物体识别等任务相结合，可以充分利用视觉特征中的语义信息，赋予环境中的物体语义，使得机器人能够更好地与环境交互，使得机器人更加智能。
3)Active SLAM：SLAM一般都是在人或者在导航算法地控制下运动建图，这样的话，机器人的运动并不一定有利于定位和建图。Active SLAM就是通过算法控制机器人按有利于定位和建图的轨迹运动。比如机器人主动（active）利用已有地图，探索未知区域、通过闭环等减小累积误差和提高鲁棒性。
 8. 作为技术篇的开篇之作，简单叙述一下自己的学习SLAM的经历以及一些主要的参考文献。转眼研究生生活已过一半，从工作到现在，尝试过开关电源、嵌入式开发、机器学习、图像处理、CV和NLP，最后选择了SLAM。从《视觉SLAM十四讲》入门，然后开啃ORB-SLAM2论文和源码。拿下ORB-SLAM2后，差不多一两个星期可以初步搞定一个开源算法，从多传感器融合的VINS-Mono和PL-VIO到lidar算法Cartographer。感叹一下，carto这源码是我看过最绕的代码，得像剥洋葱一层一层扒开，终于知道《C++ primer》是怎么用的了！ 得益于导师的放养，自己能够到公司实习，通过项目加深对上面几个算法的理解。感觉自己遇到了全天下最好的导师，一心为学生着想。经过一些实习面试，发现自己对知识掌握的深度还不太够，于是决定停下来好好总结一下再去探索新的算法。因此，决定重启CSDN，将四个算法放在一个系列一起总结对比一下，另外也会单独更详细结合源码解析各个算法。希望通过这种方式加深自己的理解，也能为SLAM的爱好者提供一些帮助，希望感兴趣的朋友点进来看一看，多多指正！以下是本系列（SLAM总结）一些主要参考文献
 
 1)ORB-SLAM2
[1]ORB-SLAM: Tracking and Mapping Recognizable Features（2014）
[2]ORB-SLAM: A Versatile and Accurate Monocular SLAM System（2015）
[3]ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras（2017）
[4]Fast角点：Faster and Better: A Machine Learning Approach to Corner Detection
[5]Brief描述子：Binary robust independent elementary features
[6]ORB特征：ORB_an efficient alternative to SIFT or SURF
[7]词袋：Bags of Binary Words for Fast Place Recognition in Image Sequences
词袋相似度计算（需采集数据集大量特征，《视觉SLAM十四讲》P309）
[8]Video google: A text retrieval approach to object matching in videos
[9]Understanding inverse document frequency: on theoretical arguments for idf
[10]图优化：g2o：A General Framework for Graph Optimization
[11]H分解出t、R：Motion and structure from motion in a piecewise planar environment
[12]E的SVD分解：Multiple View Geometry in Computer Vision
[13]RANSAC：Random sample consensus:a paradigm for model fitting with applications to image analysis and automated cartography
[14]闭环检测和重定位方法：Fast Relocalisation and Loop Closing in Keyframe-Based SLAM
[15]2D-2D三维重建（5点法）：An efficient solution to the five-point relative pose problem
[16]重定位时获得当前帧的初始位姿3D-2D方法（EPnP）： An accurate On solution to the PnP problem
闭环检测中相似变换、pose graph optimization：
[17]Scale Drift-Aware Large Scale Monocular SLAM
[18]closed-form solution of absolute orientation using unit quaternions
[19]3D-3D点（闭环检测用到）求解R、T: Least-Squares Fitting of Two 3-D Point Sets

2）VINS-Mono（多传感器）：与1中一样的论文就不再罗列
[1]VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator
[2]VIO：Monocular visual-inertial state estimation for mobile augmented reality
[3]重定位、闭环检测和地图融合：Relocalization, global optimization and map merging for monocular visual-inertial SLAM
[4]EuRoc相机模型及标定方法：Single View Point Omnidirectional Camera Calibration from Planar Grids
[5]初始化：Robust initialization of monocular visual-inertial estimation on aerial robots
[6]时间偏移标定：Online Temporal Calibration for Monocular Visual-Inertial Systems
[7]Harris角点：Good features to track
[8]KLT稀疏光流：An iterative image registration technique with an application to stereo vision
IMU预积分：
[9]Tightly-coupled monocular visualinertial fusion for autonomous flight of rotorcraft MAVs
[10]On-manifold preintegration for real-time visual–inertial odometry
[11]IMU preintegration on manifold for efficient visual-inertial maximum-a-posteriori estimation
[12]边缘化方法（schur）：Sliding window filter with application to planetary landing

3）PL-VIO（多传感器、点线特征融合）：与1中一样的论文就不再罗列
[1]	PL-VIO: Tightly-Coupled Monocular Visual–Inertial
Odometry Using Point and Line Features
[2]	LSD线特征：LSD: A Fast Line Segment Detector with a False Detection Control
[3]	LBD线特征描述子：An efficient and robust line segment matching approach based on LBD descriptor and pairwise geometric consistency

4）Cartographer
[1]	Real-Time Loop Closure in 2D LIDAR SLAM
[2]	ICP：Linear Least-Squares Optimization for Point-to-Plane ICP Surface Registration
[3]	闭环优化：Sparse pose adjustment for 2D mapping

5）多几何特征
多特征：
[1]	Unified Representation and Registration of Heterogeneous Sets of Geometric Primitives
[2]	StructVIO: Visual-Inertial Odometry With Structural Regularity of Man-Made Environments
[3]	Systematic Handling of Heterogeneous Geometric Primitives in Graph-SLAM Optimization
线特征：
[4]	The 3D Line Motion Matrix and Alignment of Line Reconstructions∗
[5]	Building a 3-D Line-Based Map Using Stereo SLAM
面特征：
[6]	GPO: Global Plane Optimization for Fast and Accurate Monocular SLAM Initialization
物体：
[7]	QuadricSLAM: Dual Quadrics From Object Detections as Landmarks in Object-Oriented SLAM
[8]	CubeSLAM: Monocular 3-D Object SLAM

6)增量式优化
[1]iSAM: Incremental Smoothing and Mapping
[2]iSAM2: Incremental smoothing and mapping using the Bayes tree
[3]ICE-BA: Incremental, Consistent and Efficient Bundle Adjustment for
Visual-Inertial SLAM
[4]g2o: A General Framework for Graph Optimization

7）相关书籍，主要是数学和图像处理相关
[1]视觉SLAM十四讲，高翔
[2]概率机器人
[3]计算机视觉：多视图几何
[4]机器人学中的状态估计
[5]数值分析与实验
[6]线性代数9讲，张宇
[7]线性系统理论数学基础
[8]机器学习，周志华
[9]图像处理，冈萨雷斯



