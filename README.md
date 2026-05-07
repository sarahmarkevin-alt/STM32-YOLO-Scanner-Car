# STM32-YOLO-Scanner-Car

基于STM32F103C8T6和树莓派5的智能跟踪打击小车（第十三届光电设计大赛项目）
主控部分：用STM32F103C8T6做下位机，直接驱动控制二自由度云台，直接驱动两路避障超声波SR04，串口与电机驱动板通信，轮子和电机均使用PID控制

视觉（加了HAILO加速）：讲解后面再写

hardware:https://github.com/hsin-hsv/control_science_sensor/tree/homework-4
=======
FreeRTOS系统调度：任务1（LED1任务，方便观察单片机程序是否运行，1000ms阻塞），任务2（print任务等待串口接受数据释放二元信号量然后进行存储数据，防止卡死100ms查询）

任务3（PID任务，根据当前偏差设置PID参数），任务4（TARGET任务，1000ms阻塞，然后运动到设定位置），任务5（motor\_init任务，保证比赛开始冲线和传感器初始化）

任务6（motor任务，超声波传感器中断，避障，2min没结束比赛换位置找敌方小车，5min挂起进程停车）

视觉：









2026.5.7更新：

1\.利用互斥量和临界区，防止全局变量posbuf1或posbuf2产生数据撕裂；PID未加保护，因为PID频率很高，添加保护在比赛中很容易阻塞进程

2\.不再采用估计时间，采用FREERTOS系统滴答记录实时时间，方便为比赛调试



>>>>>>> ec70c8e (fourth:promotion :global variable and thread protection)
