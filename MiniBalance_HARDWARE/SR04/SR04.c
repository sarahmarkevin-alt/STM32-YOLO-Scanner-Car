#include "SR04.h"

int time=0;

void turnleft_90(void)
{
	Contrl_Speed(30*-10,30*-10,-30*10,-30*10);
				delay_ms(200);
	Contrl_Speed(-300,-300,300,300);
				Deal_data_real();
			  delay_ms(500);
	Contrl_Speed(0,0,0,0);
	       delay_ms(500); 
		
  }
void turnright_90(void )
{
	 Contrl_Speed(30*-10,30*-10,-30*10,-30*10);
				delay_ms(200);
  	Contrl_Speed(300,300,-300,-300);
				Deal_data_real();
			  delay_ms(500);
		Contrl_Speed(0,0,0,0);
	       delay_ms(500); 
  }
void turn_onecircle(void)
{
		Contrl_Speed(30*-20,30*-20,0*10,0*10);
				Deal_data_real();
			  delay_ms(1000);
}
// 初始化两路超声波
void HC_SR04_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_hcsr04init;

    // 第一路超声波初始化
    RCC_APB2PeriphClockCmd(HCSR04_CLK_Trig | HCSR04_CLK_Echo, ENABLE);
    GPIO_InitStructure.GPIO_Pin = HCSR04_TRIG;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HCSR04_PORT_Trig, &GPIO_InitStructure);
    GPIO_ResetBits(HCSR04_PORT_Trig, HCSR04_TRIG);
 
    GPIO_InitStructure.GPIO_Pin = HCSR04_ECHO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(HCSR04_PORT_Echo, &GPIO_InitStructure);
    
    // 第二路超声波初始化
    RCC_APB2PeriphClockCmd(HCSR04_CLK_Trig2 | HCSR04_CLK_Echo2, ENABLE);
    GPIO_InitStructure.GPIO_Pin = HCSR04_TRIG2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(HCSR04_PORT_Trig2, &GPIO_InitStructure);
    GPIO_ResetBits(HCSR04_PORT_Trig2, HCSR04_TRIG2);
 
    GPIO_InitStructure.GPIO_Pin = HCSR04_ECHO2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(HCSR04_PORT_Echo2, &GPIO_InitStructure);
 
    // 定时器初始化（共用TIM3）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_hcsr04init.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_hcsr04init.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_hcsr04init.TIM_Period        = 1000-1;
    TIM_hcsr04init.TIM_Prescaler     = 48-1;
    TIM_TimeBaseInit(TIM3, &TIM_hcsr04init);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 
    NVIC_InitTypeDef NVIC_InitStructure;
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;       
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM3, DISABLE);
}

// 启动指定超声波
void HC_SR04_start(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{   
    GPIO_ResetBits(GPIOx, GPIO_Pin);
    delay_us(2);
    GPIO_SetBits(GPIOx, GPIO_Pin);
    delay_us(10);
    GPIO_ResetBits(GPIOx, GPIO_Pin);
}

//获取定时器2计数器值
u32 GetEchoTimer(void)
{
   u32 t = 0;
   t = time*1000;                                                           
   t += TIM_GetCounter(TIM3);
   TIM3->CNT = 0;  
   delay_ms(50);
   return t;
}

//定时器3中断
void TIM3_IRQHandler(void)  
{
   if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  
   {
       TIM_ClearITPendingBit(TIM3, TIM_IT_Update  ); 
       time++;
   }
}

//通过定时器计数器值推算距离
// 获取指定超声波的距离
float HCSR04GetLength(GPIO_TypeDef* Echo_PORT, uint16_t Echo_PIN)
{
   u32 t = 0;
   int i = 0;
   float lengthTemp = 0;
   float sum = 0;
   
   while(i < 2)  // 减少采样次数提高响应速度
   {
      // 根据Echo端口确定Trig端口
      if(Echo_PORT == HCSR04_PORT_Echo && Echo_PIN == HCSR04_ECHO)
          HC_SR04_start(HCSR04_PORT_Trig, HCSR04_TRIG);
      else
          HC_SR04_start(HCSR04_PORT_Trig2, HCSR04_TRIG2);
      
      while(GPIO_ReadInputDataBit(Echo_PORT, Echo_PIN) == 0);      
      TIM_SetCounter(TIM3, 0);
      time = 0;
      TIM_Cmd(TIM3, ENABLE);
      i++;
      
      while(GPIO_ReadInputDataBit(Echo_PORT, Echo_PIN) == 1);
      TIM_Cmd(TIM3, DISABLE);
      t = GetEchoTimer();
      lengthTemp = ((float)t/58.8); // cm
      sum += lengthTemp;    
    }
    return sum/3.0;
}
// 避障策略
void avoid_obstacle(float front_dist, float left_dist)
{
    if(left_dist<20 || front_dist <20)
    {
         turnleft_90();
    }
    else {
        // 无障碍物直行
        Contrl_Speed(15*20, 15*20, 15*20, 15*20);
    }
}
void stop_elcmotor()
{
	Contrl_Speed(0,0,0,0);
	delay_ms(4000);
}
