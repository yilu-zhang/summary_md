@[TOC](Cartographer总结(三）-线程池（thread_pool）)
1.	线程池：程序运行时，创建一定数量的线程，此时所有线程都处于空闲状态。当有新的任务进来时，若线程池中有空闲线程，则从线程池中取出一个空闲的线程处理该任务，任务处理完后，该线程被放到线程池中；若线程池中无空闲线程，则将任务放入任务队列等待线程池中有线程空闲。[参考链接](https://www.cnblogs.com/yangang92/p/5485868.html)
2.	使用线程池原因：线程的建立与销毁存在开销，线程池中线程在程序运行过程中不会被销毁，从而减小这一开销；避免打开过多线程会给系统产生大量消耗。[参考链接](https://blog.csdn.net/qq_34771252/article/details/90319537) 
3.	Cartograher线程池：实现线程池功能的类在cartographer/common/thread_pool.h与task.h中。thread_pool类中重要变量：
1)任务队列(task_queue_)：每个线程的DoWork()线程空闲时都会通过反复读取该队列来获得任务，各线程通过互斥锁防止同时读取。
2)tasks_not_ready_：尚未ready的任务队列，其依赖还没准备好。
task类中重要变量：
1)WorkItem：std::function变量，用于保存任务，利用其在赋值时不执行，赋值完成后被调用才执行。常使用lambda函数赋值给它，lambda函数具有很强的灵活性，可以表达各种函数。其形式为：[函数对象参数] (操作符重载函数参数) mutable 或 exception 声明 -> 返回值类型 {函数体},参数既可以使用[函数对象参数]传递，也可以使用(操作符重载函数参数)，{函数体}可以包含多个函数。
线程池实现步骤如下：
1)初始化：构造map_builder(cartographer/mapping/map_builder.h)类实例时会构造thread_pool，thread_pool中会构建map_builder.lua->num_background_threads个线程。每个线程构造时开始运行thread_pool::DoWork()函数。
![DoWork（）函数体](https://img-blog.csdnimg.cn/20210306145641981.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTI0NTk4OA==,size_16,color_FFFFFF,t_70)

2)初始化成功后要解决就是如何将任务添加到task_queue_。Task按以下几个状态顺序执行：NEW（新建任务，还未schedule到线程池）、DISPATCHED（任务已经schedule到线程池）、DEPENDENCIES_COMPLETED（任务依赖已经执行完成）、RUNNING（任务执行中）、COMPLETED（任务完成）等多个状态。具体步骤如下：
3)新建task实例，状态默认为NEW，然后通过task->SetWorkItem设置任务（WorkItem）
4)（可选，有依赖的任务才需添加）task2->AddDependcy（task1）：tasl2依赖于task1，只有在task1执行完后，task2才能执行。
5)tread_pool->Schedule（*task）：将task赋给tasks_not_ready_并将task状态变为DISPATCHED，判断依赖是否完成，若完成则将状态置为DEPENDENCIES_COMPLETED ，然后task加入task_queue_并从tasks_not_ready_移除等待线程执行任务；若依赖未完成，则等待依赖的task执行完。
6)	在task->Exrcute()：执行任务时将状态设为RUNNING，执行完任务时将状态设为COMPLETED。每个任务执行完都会检查依赖其的任务并将该任务依赖数减1，当依赖它的任务的依赖数减到0时，该任务会被加入task_queue_并从tasks_not_ready_移除等待线程执行任务。[参考链接1，](https://www.cnblogs.com/heimazaifei/p/12435875.html) [参考链接2，](https://blog.csdn.net/windxf/article/details/109380520)[参考链接3](https://blog.csdn.net/xiaoma_bk/article/details/102233373)
cartographer/mapping/internal/constraints/constraint_builder_2d.cc代码中一个例子如下，其中包含3个task：scan_matcher_task、constraint_task、finish_node_task_，constraint_task依赖于scan_matcher_task，finish_node_task_依赖于constraint_task。
![thread pool例子](https://img-blog.csdnimg.cn/20210306160323546.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTI0NTk4OA==,size_16,color_FFFFFF,t_70)



