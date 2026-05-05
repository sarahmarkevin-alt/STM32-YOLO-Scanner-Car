/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-10-13

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-10-13

All rights reserved
***********************************************/
#include "control.h"	

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target;
u32 Flash_R_Count;
int Voltage_Temp,Voltage_Count,Voltage_All;
int Usart_Compelet; //串口接收完一组数据标志
int Mode_Usart_PS2, Last_Mode;

/**************************************************************************
函数功能：定时中断函数 所有的控制代码都在这里面		 
**************************************************************************/
//int TIM2_IRQHandler(void)
//{    
//	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
//	{   
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源
//		Last_Mode=Mode_Usart_PS2; //用于进入串口控制前清空之前的串口接收数据
//		Key_Scan(); //按键按下切换控制模式：串口控制/PS2手柄控制
//	  if(delay_flag==1)
//	  {
//		  if(++delay_50==5)	 delay_50=0,delay_flag=0;  //给主函数提供50ms的精准延时
//	  }

//		if(Turn_Off(Voltage)==0)               //===如果电池电压不存在异常
//		{
//			if(Mode_Usart_PS2)
//			{
//				if(Last_Mode==0)Urxbuf[0]=0,Urxbuf[1]=0; //
//				Usart_Control();	   //串口控制单个舵机
//			}
//      else Control(Speed/3);	   //默认Mode_Usart_PS2=0，PS2控制舵机
//			Xianfu_Pwm();   
//			Velocity1=Position_PID_1(Position1,Target1); //舵机PID控制，可以根据目标位置进行速度调整，离目标位置越远速度越快
//			Velocity2=Position_PID_2(Position2,Target2); //舵机PID控制，可以根据目标位置进行速度调整，离目标位置越远速度越快
//			Xianfu_Velocity();
//			Set_Pwm(Velocity1,Velocity2);    //赋值给PWM寄存器
//  	 }
//		 Led_Flash(100);                     //===LED闪烁;常规模式 1s改变一次指示灯的状态	
//		 Voltage_All+=Get_battery_volt();    //多次采样累积
//		 if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//求平均值 获取电池电压	       
//   	}       	
//	 return 0;	  
//} 
/**************************************************************************
函数功能：赋值给PWM寄存器,并且判断转向
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(float velocity1,float velocity2)
{	
	Position1+=velocity1;		   //速度的积分，得到舵机的位置
	Position2+=velocity2;
	
	//赋值给STM32的寄存器
	TIM4->CCR3=Position1;    //GPIOB8,云台舵机
	TIM4->CCR4=Position2;    //GPIOB9,外臂舵机
}
/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<1000)//电池电压低于10V  舵机将不能控制
			{	                                                
      temp=1;      
      }
			else
      temp=0;
      return temp;			
}
/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  //舵机1脉宽极限值，即限制舵机转角13.5°-256.5° 
	  //舵机2脉宽极限值，即限制舵机转角9°   -171°
	  int Amplitude_H1=1100, Amplitude_L1=800;  //中心位置为750 ，运动范围±450  750±450；  750-250）/90=5.5；    5.5*80≈450；
    int Amplitude_H2=1200, Amplitude_L2=300;  //中心位置为750 ，运动范围±450  750±450； （750-250）/90=5.5；  5.5*80≈450；	
    if(Target1<Amplitude_L1)  Target1=Amplitude_L1;	
		if(Target1>Amplitude_H1)  Target1=Amplitude_H1;	
	  if(Target2<Amplitude_L2)  Target2=Amplitude_L2;	
		if(Target2>Amplitude_H2)  Target2=Amplitude_H2;	
}
/**************************************************************************
函数功能：限制速度 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Velocity(void)
{	
	  int Amplitude_H=Speed, Amplitude_L=-Speed;  
    if(Velocity1<Amplitude_L)  Velocity1=Amplitude_L;	
		if(Velocity1>Amplitude_H)  Velocity1=Amplitude_H;	
	  if(Velocity2<Amplitude_L)  Velocity2=Amplitude_L;	
		if(Velocity2>Amplitude_H)  Velocity2=Amplitude_H;	
}
///**************************************************************************
//函数功能：控制机械臂
//入口参数：控制目标
//返回  值：无
//**************************************************************************/
//void Control(float Step)
//{ 	
//				if(PS2_KEY==8)      Target1+=Step;    //底部舵机
//	 else if(PS2_KEY==6)		  Target1-=Step;
//	
//	 else if(PS2_KEY==5)			Target2-=Step;    //摇臂舵机
//	 else if(PS2_KEY==7)			Target2+=Step; 
//	
//	 if(PS2_KEY==11)		Speed+=0.05;  //速度控制
//	 else if(PS2_KEY==9)		Speed-=0.05;
//	 if(Speed<=3)Speed=3;
//	 if(Speed>=30)Speed=30;

//	
//}
/*************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM增量
**************************************************************************/
float Position_PID_1(float Position,float Target)
{ 	                                          //增量输出
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Position;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;  
}
/*************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM增量
**************************************************************************/
float Position_PID_2(float Position,float Target)
{ 	                                         //增量输出
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Position;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;  
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
函数功能：串口通信BBC校验与计算目标角度对应PWM值
入口参数：无
返回  值：无
**************************************************************************/
void Usart_Control()
{
  int i,error=1;
  u8  check=0;
	
	if(Usart_Compelet==1)//串口通信   对数据BCC校验
	 {
    for(i=0; i<7; i++)
    {
      check=Urxbuf[i]^check; //异或，用于检测数据是否出错
    }
    if(check==Urxbuf[7]) error=0; //检验成功
    else error=1;
		Usart_Compelet=0;

   }	

	 if(error==0)
	 {
		if(Urxbuf[0]!=0)
		 {	
			Target1=Urxbuf[0]*1000;  //底部舵机
			Target1=Target1/270+250; //将角度转换为PWM值
		 }
		if(Urxbuf[1]!=0)
		 {	
			Target2=Urxbuf[1]*1000;  //摇臂舵机
			Target2=Target2/180+250; //将角度转换为PWM值
		 }	
		error=1;
	 }
}

/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key_Scan(void)
{	
	u8 temp;
	temp=click_N_Double(50);         //双击检测
  if(temp==1)	Mode_Usart_PS2=!Mode_Usart_PS2;//单击切换PS2手柄/Usart串口控制模式
}
