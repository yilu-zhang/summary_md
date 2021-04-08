@[TOC](SLAM总结（二）-数学基础之常用概率知识)
1.常用概率论基础概念：在高维情况下，即n很大时，由链式法则得到的联合概率p(x~1~,x~2~,...,x~n~)将非常难求解，但是在每个状态只与前一个状态有关的条件独立的假设下，链式法则中p(x~i~ | x~1~,x~2~,...,x~i-1~)变成p(x~i~ | x~i-1~）,求解将大大简化
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210321212652551.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTI0NTk4OA==,size_16,color_FFFFFF,t_70#pic_center)
2.SLAM问题可以看成一个概率问题，概率意味着不确定性，导致不确定性的原因主要是传感器测量和控制量不是非常精确和存在噪声干扰。在马尔可夫假设下（第n次状态常决定于第n-1次状态）获得条件独立性，那么SLAM问题可用贝叶斯图（有向图）表示，如下图所示。其中x表示机体pose，l表示路标点（landmark）的位置，z表示第j个路标点在第i个机体pose下的观测，u为相邻状态转换的控制量，一般指向电机发送的控制量或者由编码器测得的控制量（更准）。其中，x、l为待估计的状态，z、u为已知状态。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210321223151697.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTI0NTk4OA==,size_16,color_FFFFFF,t_70#pic_center)
根据图中信息，通过因式分解可将SLAM表示为联合概率P(X,L,U,Z) ,如下图所示。其中大写X、L、U、Z分别包含所有小写x、l、u、z变量，如X=[x~0~,x~1~,…,x~n~]^T^。第一项对应运动模型，第二项对应观测模型。一般假设它们都是高斯分布，运动模型的均值由运动方程f(x~i-1~,u~i~)得到，方差由控制量的误差决定。观测模型的均值由观测方程h(x~ji~,l~ji~)得到，方差由传感器的测量误差决定。ORB-SLAM2的单目情况只有视觉观测模型。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210321223606199.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTI0NTk4OA==,size_16,color_FFFFFF,t_70#pic_center)

从而，将SLAM问题转化为求解下图中方程，即非线性最小二乘问题。解方程有求解析解和数值解2种方法。该方程维数很高，很难求得解析解，故可用非线性优化（数值解方法）求解，后端部分再详细介绍非线性优化方法。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210321223718532.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTI0NTk4OA==,size_16,color_FFFFFF,t_70#pic_center)
3.随机噪声的理解：比如我们现在需要预测下图二维空间中的虚直线是怎样的，假如我们只能通过一个传感器，每次观测到虚直线上一个点的坐标(x~i~,y~i~)。如果我们一直停在一个地方观测下图第3象限点a，由于噪声的存在，会得到一堆围绕点a的点，我们也无法知道哪个点就是点a。一般噪声服从高斯分布，当观测量足够多的时候，我们会发现这些点正好服从高斯分布，观测越多越接近真实分布（使机器人静止来测量传感器的噪声是比较容易地，但是传感器在运动时地噪声和静止时地噪声可能不太一样）。但一般我们边移动边观测，如第一象限，理论上只要观测到2个点就能得到一条直线，如果我们观测到b、c两个点后，马上预测一条直线，由于噪声的存在，直线bc会偏离真实虚直线。但如果我们观测到很多点后，用最小二乘法去拟合出一条直线，这条直线就会比较接近真实虚直线。理论上，观测的点越多，拟合出来直线就越接近真实直线。观测噪声越小，拟合出来直线越接近真实直线。SLAM中，观测噪声与Lidar、相机等传感器的精度对应。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210322001316519.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTI0NTk4OA==,size_16,color_FFFFFF,t_70#pic_center)

4.随机采样一致性（RANSAC）：如果上图观测到的点存在如d、e一样与实际值相差很大的点（outlier），若直接使用最小二乘方法拟合，那么拟合出来的直线很可能受d、e影响，与实际值相差很大。为了排除outlier的影响，RANSAC被提出来了。以上图为例，我们能得到有n个点的点集S，方法如下，以下是一个循环，：
1）从S随机取出m个点集S~t~（观测）拟合出一条直线（模型）；（m一般取稍大于最小能确定直线（模型）的点数）
2）然后计算剩余的点与直线的距离，若距离小于等于阈值t,则归为inlier集S~i~，否则归为outlier集S~o~
3）若S~i~中点的数量大于阈值T，则使用S~t~和S~i~中所有的点拟合出一条直线，退出循环
4）若循环次数小于N,则跳转到步骤1），否则选取前N次循环中S~i~最大的一次，使用S~t~和S~i~拟合出一条直线退出循环（这里也可以设置，当前N次循环中S~i~点数小于一个阈值时直接退出，直接报错）

使用RANSC时，重要的是如何选取m、t、T、N等阈值，之后在SLAM中使用时会具体问题具体分析。
