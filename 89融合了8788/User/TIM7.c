#include "TIM6.h" 
#include "TIM4.h" 
#include "led.h" 
#include "oled.h"
#include "imu.h"
#include "bsp_usart.h"
#include "TIM7.h" 
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "ALIENTEK.h"
extern unsigned char motor_mode;
extern float pitch,roll,yaw; 	        //欧拉角
extern float yaww_offset;
extern float yaww;
// 中断优先级配置
static void TIM7_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    // 设置中断组为0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		
		// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn ;	
		// 设置主优先级为 1
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;	 
	  // 设置抢占优先级为3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 *typedef struct
 *{ TIM_Prescaler            都有
 *	TIM_CounterMode			     TIMx,x[6,7]没有，其他都有
 *  TIM_Period               都有
 *  TIM_ClockDivision        TIMx,x[6,7]没有，其他都有
 *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]才有
 *}TIM_TimeBaseInitTypeDef; 
 *-----------------------------------------------------------------------------
 */


static void TIM7_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		// 开启定时器时钟,即内部时钟CK_INT=72M
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	
		// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
    TIM_TimeBaseStructure.TIM_Period = 999;	// 999 100hz 4999 20hz  

	  // 时钟预分频数为
    TIM_TimeBaseStructure.TIM_Prescaler= 719;//72M 72 000 000        1 000 00hz
	
		// 时钟分频因子 ，基本定时器没有，不用管
    //TIM_TimeBaseStructure.TIM_ClockDivision=0;
		
		// 计数器计数模式，基本定时器只能向上计数，没有计数模式的设置
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
		
		// 重复计数器的值，基本定时器没有，不用管
		//TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	
	  // 初始化定时器
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
		
		// 清除计数器中断标志位
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	  
		// 开启计数器中断
    TIM_ITConfig(TIM7,TIM_IT_Update|TIM_IT_Trigger,ENABLE);
		
		// 使能计数器
    TIM_Cmd(TIM7, ENABLE);	
		
}

void TIM7_Init(void)
{
	TIM7_NVIC_Config();
	TIM7_Mode_Config();
}
/*
g2	you
g3	mid
g4 zuo
g5	xia
g6	shang

*/
void TIM7_IRQHandler(void)//10ms
{
	if ( TIM_GetITStatus( TIM7, TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM7 , TIM_FLAG_Update);
						//-------------------------------------------hongwai
								if(motor_mode==1||motor_mode==2)
			{
										if(PGin(3)==0&&PGin(2)==0)//没压线y_0
													{
														if(motor_mode==2&&hongwaiflags.y_2==1)
																hongwaiflags.y+=1;	
														if(motor_mode==1&&hongwaiflags.y_2==1)
																hongwaiflags.y-=1;
															if(hongwaiflags.y<=0)
																hongwaiflags.y=0;
															hongwaiflags.y_2=0;
														}
										if(PGin(3)==1&&PGin(2)==1)//双压y_2
													{
														hongwaiflags.y_2=1;
														hongwaiflags.y_1=0;
														hongwaiflags.y_3=0;
													}		
					}			
//---------------------------------------------------------x
						if(motor_mode==3||motor_mode==4)
						{
										if(PFin(0)==0&&PFin(3)==0&&PFin(2)==0)//没压线x_0
										{
											if(hongwaiflags.x_1==1)
											{
												if(hongwaiflags.x_0==1)
												{
													hongwaiflags.x+=1;
												}
											}
											if(hongwaiflags.x_3==1)
											{
												if(hongwaiflags.x_0==1)
												{
													hongwaiflags.x-=1;
													if(hongwaiflags.x<=0)
													{
														hongwaiflags.x=0;
													}
												}												
											}
											hongwaiflags.x_0=0;
											hongwaiflags.x_1=0;
											hongwaiflags.x_2=0;
											hongwaiflags.x_3=0;
										}	
										if(PFin(0)==1&&PFin(3)==0&&PFin(2)==0)//右x_1
										{
													if(hongwaiflags.x_2==1&&hongwaiflags.x_3==1)
												{
													hongwaiflags.x_2=0;
													hongwaiflags.x_3=0;
													hongwaiflags.x_0=1;
												}
												hongwaiflags.x_1=1;
										}				
										if(PFin(0)==0&&PFin(3)==1&&PFin(2)==0)//中x_2
										{
											hongwaiflags.x_2=1;
											hongwaiflags.x_0=0;
										}		
										if(PFin(0)==0&&PFin(3)==0&&PFin(2)==1)//左x_3
										{
													if(hongwaiflags.x_1==1&&hongwaiflags.x_2==1)
												{
													hongwaiflags.x_1=0;
													hongwaiflags.x_2=0;
													hongwaiflags.x_0=1;
												}
												hongwaiflags.x_3=1;
										}															
							}				
		//----------------------------IMU
			if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0)
			{
			//	printf("\r\n%f",yaw);

				yaww=-yaw-yaww_offset;//已改
				/*
				jizhun0 meishi
				179 180 -179
								-358+360
						1		2
				-179 -180   179
							-1		358-360
										-2
				*/
				if(yaww<-180)yaww+=360;
					else if(yaww>180)yaww-=360;
			//	printf("\r\n%f",yaww);
				
			}				
			//-------------------------MOTOR
						switch(motor_mode){
							case 0:{
										Stop_it();
										break;}
							case 1:{
										forward();
										break;}
							case 2:{
										backward();
										break;}
							case 3:{
										leftward();
										break;}
							case 4:{
										rightward();
										break;}
							case 5:{
										forward1();
										break;}
							case 6:{
										backward1();
										break;}
							case 7:{
										leftward1();
										break;}
							case 8:{
										rightward1();
										break;}												
							default:{break;}
					}
						//-------------------------------------------hongwai
								if(motor_mode==1||motor_mode==2)
			{
										if(PGin(3)==0&&PGin(2)==0)//没压线y_0
													{
														if(motor_mode==2&&hongwaiflags.y_2==1)
																hongwaiflags.y+=1;	
														if(motor_mode==1&&hongwaiflags.y_2==1)
																hongwaiflags.y-=1;
															if(hongwaiflags.y<=0)
																hongwaiflags.y=0;
															hongwaiflags.y_2=0;
														}
										if(PGin(3)==1&&PGin(2)==1)//双压y_2
													{
														hongwaiflags.y_2=1;
														hongwaiflags.y_1=0;
														hongwaiflags.y_3=0;
													}		
					}			
//---------------------------------------------------------x
						if(motor_mode==3||motor_mode==4)
						{
										if(PFin(0)==0&&PFin(3)==0&&PFin(2)==0)//没压线x_0
										{
											if(hongwaiflags.x_1==1)
											{
												if(hongwaiflags.x_0==1)
												{
													hongwaiflags.x+=1;
												}
											}
											if(hongwaiflags.x_3==1)
											{
												if(hongwaiflags.x_0==1)
												{
													hongwaiflags.x-=1;
													if(hongwaiflags.x<=0)
													{
														hongwaiflags.x=0;
													}
												}												
											}
											hongwaiflags.x_0=0;
											hongwaiflags.x_1=0;
											hongwaiflags.x_2=0;
											hongwaiflags.x_3=0;
										}	
										if(PFin(0)==1&&PFin(3)==0&&PFin(2)==0)//右x_1
										{
													if(hongwaiflags.x_2==1&&hongwaiflags.x_3==1)
												{
													hongwaiflags.x_2=0;
													hongwaiflags.x_3=0;
													hongwaiflags.x_0=1;
												}
												hongwaiflags.x_1=1;
										}				
										if(PFin(0)==0&&PFin(3)==1&&PFin(2)==0)//中x_2
										{
											hongwaiflags.x_2=1;
											hongwaiflags.x_0=0;
										}		
										if(PFin(0)==0&&PFin(3)==0&&PFin(2)==1)//左x_3
										{
													if(hongwaiflags.x_1==1&&hongwaiflags.x_2==1)
												{
													hongwaiflags.x_1=0;
													hongwaiflags.x_2=0;
													hongwaiflags.x_0=1;
												}
												hongwaiflags.x_3=1;
										}															
							}								
	}		 	
}
/*********************************************END OF FILE**********************/
