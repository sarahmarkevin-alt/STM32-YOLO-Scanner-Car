
/***********************************************************
重要说明
关于舵机控制的说明
舵机的定时器频率为50HZ
控制舵机运动的占空比数值范围是250-1250，占空比为750时是舵机的正中心位置
控制舵机运动的代码在control.c文件中
***************************************************************/

#include "stm32f10x.h"
#include "sys.h"
#include "control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "SR04.h"
#include "AllHeader.h"


//u8 Flag_Show=0;                //停止标志位和 显示标志位 默认停止 显示打开
float Velocity1,Velocity2;     //电机PWM变量
// Positon1为上下300~900，Position2为左右，300~1200为宜，从下到上，从右到左
// C70摄像头的差值对应为一个横-150~150纵-100~100的区间
// 700，630-->950, 750(镜头角度50)为合适的默认点位，恰好为直视前方
float  Position1=950,Position2=750; 
float Speed=10;                   //运动速度设置                 
float Target1=950,Target2=750;     //电机目标值
float pos1buf=0,pos2buf=0;
float	Position_KP=100,Position_KI=0,Position_KD=5;  //位置控制PID参数
uint8_t Urxbuf[8]; //串口接收数据数组

//RTOS声明
#define START_TASK_PRIO 6 //任务优先级
#define START_STK_SIZE 128 //任务堆栈大小
TaskHandle_t StartTask_Handler; //任务句柄
void start_task(void *pvParameters); //任务函数
 
#define LED1_TASK_PRIO 2 //任务优先级
#define LED1_STK_SIZE 64 //任务堆栈大小
TaskHandle_t LED1Task_Handler; //任务句柄
void led1_task(void *p_arg); //任务函数

// 定义打印任务的优先级和堆栈大小
#define PRINT_TASK_PRIO 6 // 任务优先级
#define PRINT_STK_SIZE 128 // 任务堆栈大小
TaskHandle_t PrintTask_Handler;// 定义打印任务的句柄
void print_task(void *p_arg);// 打印任务函数

// 定义 PID 任务的优先级和堆栈大小
#define PID_TASK_PRIO 5 // 任务优先级5
#define PID_STK_SIZE 64 // 任务堆栈大小
TaskHandle_t PIDTask_Handler; // 定义 PID 任务的句柄
void pid_task(void *p_arg); // PID 任务函数

#define TARGET_TASK_PRIO 3 //任务优先级5
#define TARGET_STK_SIZE 64 //任务堆栈大小
TaskHandle_t TARGETTask_Handler; //任务句柄
void target_task(void *p_arg); //任务函数

// 定义电机控制任务的优先级和堆栈大小
#define MOTOR_TASK_PRIO 6 // 任务优先级3
#define MOTOR_STK_SIZE 256 // 任务堆栈大小
TaskHandle_t MotorTask_Handler; // 任务句柄
void motor_task(void *p_arg); // 电机控制任务函数

#define MOTOR_INIT_TASK_PRIO 1
#define MOTOR_INIT_STK_SIZE 64
TaskHandle_t MotorInitTASK_Handler;
void motor_init_task(void *p_arg);

SemaphoreHandle_t MotorStartSem;//二元信号量保证电机在初始化后才进行运动//
SemaphoreHandle_t xTargetMutex;
int main(void)
  { 
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		LED_Init();
		delay_init();	    	            //=====延时函数初始化	
		uart_init(57600);                  //=====串口初始化
		TIM4_PWM_Init(9999,95);        //=====舵机PWM初始化
		TIM4->CCR3=Position1;    //GPIOB8,云台舵机
		TIM4->CCR4=Position2;
		//创建开始任务
		xTaskCreate((TaskFunction_t )start_task, //任务函数
								(const char* )"start_task", //任务名称
								(uint16_t )START_STK_SIZE, //任务堆栈大小
								(void* )NULL, //传递给任 务函数的参数
								(UBaseType_t )START_TASK_PRIO, //任务优先级
								(TaskHandle_t* )&StartTask_Handler); //任务句柄
								
		vTaskStartScheduler(); //开启任务调度		
}

void start_task(void *pvParameters) 
{ 
	MotorStartSem=xSemaphoreCreateBinary();
	xTargetMutex=xSemaphoreCreateMutex();
	taskENTER_CRITICAL(); //进入临界区
	//创建 LED1任务
	xTaskCreate((TaskFunction_t )led1_task, 
							(const char* )"led1_task", 
							(uint16_t )LED1_STK_SIZE, 
							(void* )NULL, 
							(UBaseType_t )LED1_TASK_PRIO, 
							(TaskHandle_t* )&LED1Task_Handler);
	//创建 print 任务
	xTaskCreate(
        (TaskFunction_t)print_task, // 任务函数
        (const char *)"print_task", // 任务名称
        (uint16_t)PRINT_STK_SIZE,   // 任务堆栈大小
        (void *)NULL,               // 任务参数
        (UBaseType_t)PRINT_TASK_PRIO, // 任务优先级
        (TaskHandle_t *)&PrintTask_Handler // 任务句柄
    );
		// 创建 PID 任务
		xTaskCreate(
				(TaskFunction_t)pid_task, // 任务函数
				(const char *)"pid_task", // 任务名称
				(uint16_t)PID_STK_SIZE,   // 任务堆栈大小
				(void *)NULL,             // 任务参数
				(UBaseType_t)PID_TASK_PRIO, // 任务优先级
				(TaskHandle_t *)&PIDTask_Handler // 任务句柄
		);
		// 创建 TARGET 任务
		xTaskCreate(
				(TaskFunction_t)target_task, // 任务函数
				(const char *)"target_task", // 任务名称
				(uint16_t)TARGET_STK_SIZE,   // 任务堆栈大小
				(void *)NULL,             // 任务参数
				(UBaseType_t)TARGET_TASK_PRIO, // 任务优先级
				(TaskHandle_t *)&TARGETTask_Handler // 任务句柄
		);
		//创建电机初始化任务
		xTaskCreate(
				(TaskFunction_t)motor_init_task, 
				(const char *)"motor_init_task", 
				(uint16_t)MOTOR_INIT_STK_SIZE, 
				(void *)NULL, 
				(UBaseType_t)MOTOR_INIT_TASK_PRIO, 
				(TaskHandle_t *)&MotorInitTASK_Handler
		);
		// 创建电机控制任务
		xTaskCreate(
				(TaskFunction_t)motor_task, 
				(const char *)"motor_task", 
				(uint16_t)MOTOR_STK_SIZE, 
				(void *)NULL, 
				(UBaseType_t)MOTOR_TASK_PRIO, 
				(TaskHandle_t *)&MotorTask_Handler
		);

	taskEXIT_CRITICAL(); //退 出临界区
	vTaskDelete(StartTask_Handler); //删除开始任务
}

//LED1任务函数
void led1_task(void *p_arg) 
{ 
	while(1) 
	{ 
		Led_Flash(1);
//		TIM4->CCR3=Position1;    //GPIOB8,云台舵机
//		TIM4->CCR4=Position2;
		delay_ms(1000);
	} 
}
//接收后打印任务函数
void print_task(void *pvParameters)
{
    int count=0;
    BaseType_t err = pdFALSE;

    int size=50;
    uint8_t buf[64];//最多只取前64个数据

    //清空本地接收数组
    memset(buf,0,size);

    while(1)
    {
        err=xSemaphoreTake(uartSemaphore,100);   //获取信号量
        if(err==pdTRUE)                         //获取信号量成功
        { 
					//进入临界区，防止数据再被更改//
            taskENTER_CRITICAL();
					//printf("%s",Data);
            if(rx_cnt < size)//收到的数据长度在size范围内
            {
                //void *memcpy(void *str1, const void *str2, size_t n)  
                //从存储区 str2 复制 n 个字节到存储区 str1。
                memcpy(buf,Recv,rx_cnt);//有几个复制几个
                count=rx_cnt;
								buf[rx_cnt] = 0;
                //printf("%s\r\n", buf);
            }
            else//收到的数据长度太长了
            {
                memcpy(buf,Recv,size);//只复制size个
                count=size;
								buf[size] = 0;
            }
            rx_cnt=0;
						taskEXIT_CRITICAL();//退出临界区，开启全局中断//
        }

        if(count>0)
        {
						float p,i,d;
            count=0;
						if(buf[0] == '&'){
							xSemaphoreTake(xTargetMutex,portMAX_DELAY);//取走互斥信号，防止其余任务对pos1buf等进行操作
							sscanf((const char *)buf,"&%f,%f#",&pos1buf,&pos2buf);
							xSemaphoreGive(xTargetMutex);//给出互斥信号//
							//printf("receive:%s",buf);
						}
						else if(buf[0] == '$'){
							sscanf((const char *)buf,"$%f,%f,%f#",&p,&i,&d);
							Position_KP=p,Position_KI=i,Position_KD=d;
							//printf("config:%s",buf);
						}
            //------------------------------------------------------------------------------
            //这里可以继续对buf进行分析和处理，比如根据buf的不同内容执行不同的小任务

        }
    }
}
//PID任务函数
void pid_task(void *pvParameters){
	while(1){
		Velocity1=Position_PID_1(Position1,Target1); //舵机PID控制，可以根据目标位置进行速度调整，离目标位置越远速度越快
		Velocity2=Position_PID_2(Position2,Target2); //舵机PID控制，可以根据目标位置进行速度调整，离目标位置越远速度越快
		Xianfu_Velocity();
		Set_Pwm(Velocity1,Velocity2);    //赋值给PWM寄存器
		delay_ms(10);
	}
}
//TARGET任务函数
void target_task(void *pvParameters){
	while(1){
		xSemaphoreTake(xTargetMutex,portMAX_DELAY);
		Target1 -= pos2buf / 5;
		Target2 -= pos1buf / 4;
		pos2buf = pos1buf = 0;
		xSemaphoreGive(xTargetMutex);
		Xianfu_Pwm();
		delay_ms(1000);
	}
}
// 电机控制任务函数
void motor_task(void *p_arg) 
{ 
	if(xSemaphoreTake(MotorStartSem,portMAX_DELAY)==pdTRUE)
  {	
    TickType_t xStartTick = xTaskGetTickCount();
		const TickType_t xTwoMinTicks = pdMS_TO_TICKS(120000);
		const TickType_t xFiveMinTicks = pdMS_TO_TICKS(300000);
		float right_length;float left_length;
		while(1)
		{    
			TickType_t xElapsed = xTaskGetTickCount() - xStartTick;
			// 获取两路超声波距离
			right_length = HCSR04GetLength(HCSR04_PORT_Echo, HCSR04_ECHO);
			left_length = HCSR04GetLength(HCSR04_PORT_Echo2, HCSR04_ECHO2);
        //小于2min循环往复前进     
     if(xElapsed<xTwoMinTicks)
			 //前方有障碍物直接后退3s
     { if(right_length<25||left_length<25 )
			 {	Contrl_Speed(-300,-300,-300,-300);
             delay_ms(1500);
             delay_ms(1500);
			 }
			 //前方无障碍物低速前进
      else
			{   
				Contrl_Speed(15*20, 15*20, 15*20, 15*20);
             delay_ms(20);
			}
		 }
		 else if((xTwoMinTicks<xElapsed)&&(xElapsed<xFiveMinTicks))
		 {//两分钟后布朗运动
       avoid_obstacle(right_length,left_length);
       delay_ms(20);
		 }
		 //5分钟后直接停止
		else if(xElapsed>xFiveMinTicks)
		{
			Contrl_Speed(0,0,0,0);vTaskSuspend(NULL);					
		}
		 } 

	}
}
void motor_init_task(void *pvParameters)
{
	int is=50;
	HC_SR04_init();
	//和电机模块串口通信	Serial communication with motor module
	Motor_Usart_init();
	
	//先关闭上报	Close the report first
	send_upload_data(false,false,false);
  delay_ms(10);
	
	send_motor_type(1);//配置电机类型	Configure motor type
	delay_ms(10);
	send_pulse_phase(30);//配置减速比 查电机手册得出	Configure the reduction ratio. Check the motor manual to find out
	delay_ms(10);
	send_pulse_line(11);//配置磁环线 查电机手册得出	Configure the magnetic ring wire. Check the motor manual to get the result.
	delay_ms(10);
	send_wheel_diameter(80.00);//配置轮子直径,测量得出		Configure the wheel diameter and measure it
	delay_ms(10);
	send_motor_deadzone(1600);//配置电机死区,实验得出	Configure the motor dead zone, and the experiment shows
	delay_ms(10);
	
	//给电机模块发送需要上报的数据	Send the data that needs to be reported to the motor module
	send_upload_data(false,true,false);delay_ms(10);
	Contrl_Speed(0,0,0,0);	
	delay_ms(50);
	//全速冲过中线
	Contrl_Speed(is*20,is*20,is*20,is*20);	
	delay_ms(2000);
	xSemaphoreGive(MotorStartSem);
	vTaskDelete(NULL);	
	
}
