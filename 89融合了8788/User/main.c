#include "stm32f10x.h"
#include "key.h"
#include "led.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "bsp_usart3.h"
#include "TIM2.h"
#include "TIM6.h"
#include "TIM3.h"
#include "TIM4.h"
#include "TIM1.h"
#include "TIM8.h"
#include "oled.h"
#include "filter.h"
#include "iic.h"
#include "mpu6050.h"
#include "imu.h"
#include "hongwai.h"
#include "uart4.h"
#include "mpu9250.h"
#include "DataScope_DP.h"
#include "1iic.h"
#include "HMC5983.h"
#include "uart5.h"
#include "encoder.h"
#include "motor.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "ALIENTEK.h"
#include "TIM7.h"
#include "steer.h"
#include "new_struct.h"
#include "openmv.h"
/*
资源总览-拟定
TIM1  CH1-A8 CH4-A11//舵机
TIM4	CH1-B6 CH2-B7 CH3-B8 CH4-B9//电机
12345
A11 A8 C7 C6 C8
123(FAN)54
方向			G13		G11			D8			D10
					G14		G12			D9			D11
编码器			B1		B0			A7			A6
					↓ 		← 			↑ 			→
					
新的  		4→			3↓			2←			1↑
右接右上 下接右下 左接左下 上接左上
左上接1 左下接2
右接1 中接2 左接3
方向			G13		G11			D8			D10
					G14		G12			D9			D11
					A2A3  A0A1   B0B1   A6A7
					

			PGout(11)=1;//1前
			PGout(12)=0;//0前
			PDout(10)=0;//0前
			PDout(11)=1;//1前	

			PGout(13)=1;//1左
			PGout(14)=0;//0左
			PDout(8)=1;//1左
			PDout(9)=0;//0左					
TIM5	CH1-A0 CH2-A1//待定
TIM6
TIM8	CH1-C6 CH2-C7 CH3-C8//舵机

LED	E8 E9 E10 
KEY	D3 D4 E7 D12 A12 F12 F13 F15

UART1 TX-A9左 RX-A10右
UART3 TX-B10右 RX-B11左
5V
GND
B11
B10
红外对管	G2 G3 G4 G5      G2 G3  e f   


				G6 G7 G8 C9				G6 G7 G8 f m e
				
				F0 F1(F3) F2 f m e
				G2 G3  e f
陀螺仪 SCLA1 SDAA0
OLED B12 B13
			B14 B15
白 白 黑
信号 正 负
黄 红 褐
信号 正 负
1号舵机 TIM1CH1-A8  720放 340 抓
300 480 660 
2号舵机 TIM1CH4-A11 240 前  860后   start760 max 240
300 760 700
3号舵机 TIM8CH1-C6  180收 740放
200 820 800
4号舵机 TIM8CH2-C7  400夹死 280放
480 360 360夹死

5号舵机 TIM8CH3-C8
			PDout(2)=1;//1前0后
			
			PBout(3)=1;//0后1前
			PBout(4)=0;//0前1后
			PBout(5)=0;//0前1后
	TIM_SetCompare1(TIM4,100);//右后轮
	TIM_SetCompare2(TIM4,100);//右前轮
	TIM_SetCompare3(TIM4,100);//左后轮
	TIM_SetCompare4(TIM4,100);//左前轮
	uart4
	tx C10
	rx C11
	mpu9250 D6 D7
	HMC5983 D4 D5
	uart5
	5v gnd D2 C12
	tx PC12 shang  
	rx PD2	xia
	F4 P2
	F5 P3
		IN4 IN5 GND 5V IN3 IN2 IN1
C6 C8 C7 A8 A11
A8 1 
A11 2
C6 3
C7 4
识别颜色
770
830
540
655
看色环1
1270
830
450
655
看色环2
1460
830
450
655
看二维码
790
830
470
655
*/
u16 tim1ch1=1090;
u16 tim1ch4=1120;
u16 tim8ch1=360;//
u16 tim8ch2=655;//485
extern unsigned int MotorSpeed;
extern int quanju_x;
extern int quanju_y;
extern unsigned int CaptureNumberA1;//右
extern unsigned int CaptureNumberA2;//上
extern unsigned int CaptureNumberB1;
extern unsigned int CaptureNumberB2;
extern FLY_TYPE FLY;
extern FLOAT_ANGLE Att_Angle;  
extern unsigned char uart5_receice[8];
double uart5_angle=0;
double angleoffset=0;
extern unsigned char motor_mode;

float pitch,roll,yaw; 	        //欧拉角
float yaww_offset=0.0;
float yaww=0.0;

extern unsigned char uart5_mode;
extern unsigned char color_mid;
extern unsigned char color_right;


unsigned char flag_mid=0;
unsigned char flag_right=0;
unsigned char flag_code=0;
unsigned char code_count=0;
unsigned short int JG_distance=0;

extern unsigned char Openmv_Mode;//openmv模式
extern unsigned short int qd_code;//二维码
extern unsigned char color_shunjian;//颜色瞬间
extern unsigned char order_1;//物块顺序
extern unsigned char order_2;//色环顺序
extern unsigned short int color_mode;//色环顺序
void chumen(void);

void fineqd(void);
void take_1(void);
void quzhaoerweima(void);
void hui50_1(void);
void goto_blue(void);
void fromblue_totake(void);
void all_oled(void);
void goto_green(void);
void fromgreen_totake(void);
void goto_red(void);
void fromred_totake(void);//1
void fromtake_tohome(void);

int main(void)
{
	//int mpu_count=0;
	int i;
	float yaw_sum=0;
	int yaw_count=0;
	delay_ms(1000);
		OLED_Init();
		OLED_Clear();
//		OLED_Init1();
//		OLED_Clear1();
		USART_Config();
	//	USART_Config3();
		OLED_ShowString(byte(0),line1,"OLED_Init",6);	
//		OLED_ShowString1(byte(0),line1,"OLED_Init",6);	

		led_init();
		OLED_ShowString(byte(0),line1,"LED_Init",6);	

//		//	SysTick_Init();
		key_init();
		OLED_ShowString(byte(0),line1,"KEY_Init",6);		
		/*D6  D7*/
		IIC_GPIO_Init(); //模拟IIC初始化 
		OLED_ShowString(byte(0),line1,"IIC_Init   ",6);
		Delay_ms(100);
		//MPU9250_Init(); //MPU6050初始化（绿） 	
		//MPU9250_Init();		
		#if 1
		while(mpu_dmp_init())
	//	while(1)
		{   
			delay_ms(100);
		//	LED0=!LED0;//DS0闪烁 
		}	
		OLED_ShowString(byte(0),line1,"MPU9250_Init",6);
		for(i=0;i<10;i++)
		{
			if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0)
			{		
				
			}			
			delay_ms(100);
		}				
		for(i=0;i<10;i++)
		{
			if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0)
			{
				yaw_sum-=yaw;
				yaw_count++;
			}			
			delay_ms(100);
		}
		yaww_offset=yaw_sum/yaw_count;
		#endif
		TIM4_Init();//电机
		TIM6_Init();//基本定时器
		
		hongwai_init();
		if(yaww_offset==0)
		OLED_ShowString(byte(0),line1,"9250_InitError",6);
		else 
			OLED_ShowString(byte(0),line1,"9250_InitSuccess",6);
		UART_Config4();
		UART_Config5();
		TIM1_Init();//舵机1 4
		Delay_ms(100);
		TIM8_Init();//舵机1 2 3
		Delay_ms(100);
		TIM_SetCompare1(TIM1,tim1ch1);
		TIM_SetCompare4(TIM1,tim1ch4);
		TIM_SetCompare1(TIM8,tim8ch1);
		TIM_SetCompare2(TIM8,tim8ch2);
		TIM_SetCompare3(TIM8,tim8ch2);
		TIM4_Init();//电机
		Delay_ms(500);
		IncPIDInit();//PID初始化
		OLED_ShowString(byte(0),line1,"DJ_Init",6);
		Openmv_Init();/////////////////////////////////////////////////
		//TIM7_Init();
		TIM6_Init();//通用定时器
		OLED_ShowString(byte(0),line1,"___tim7_Init",6);
		while(PGin(4)==1)
		{
			
		}
		OLED_ShowString(byte(0),line1,"OOOOOOOOOO_Init",6);
		new_chumen();
		Observing_color();
		OpenMv_10();//三色
	//	OpenMv_01();//圆环单色
		OLED_ShowNum(byte(2),line2,Openmv_Mode,1,6); //10
		new_qrcode();//前往二维码区域
		OLED_ShowNum(byte(2),line3,order_1,1,6); //10
		
		OLED_ShowNum(byte(2),line2,Openmv_Mode,1,6); //10
		Observing_qrcode();//改变角度看二维码
		if(order_1==0)
		{
			order_1=1;
		}		
		OpenMv_00();//二维码
		/***跳出二维码识别***/
		while(flag_code==0)
		{
			Delay_ms(100);
			code_count++;
			if(code_count>40)
			{code_count=0;
				flag_code=1;//跳出
				qd_code=213;
			}
		}
		
		OLED_ShowNum(byte(2),line4,qd_code,4,6); //00
		new_qrget();//从二维码到中间
		new_placeedge();//中间去找色环
		sehuan_fa();//观察发车区色环
		OpenMv_01();//圆环单色
		OLED_ShowNum(byte(2),line2,Openmv_Mode,1,6); //10
		flag_right=0;
		while(flag_right==0)
		{
			Delay_ms(100);
			code_count++;
			
			if(code_count>20)
			{
				code_count=0;
			//	color_right=2;
				flag_right=1;//跳出
				//qd_code=1;			
			}
		}
		color_right=color_shunjian;
		sehuan_zhong();//观察中间色环
		flag_mid=0;
		while(flag_mid==0)
		{
			Delay_ms(100);
			code_count++;
			if(code_count>20)
			{
				code_count=0;
				flag_mid=1;//跳出		
			}
		}		
		color_mid=color_shunjian;
		Color_judgement();
		new_getmid();//从色环回来
		qd_code=123;
		order_1=1;
		color_mode=312;		
		huizong();	
		gohome();
		for(;;)
			{
				OLED_ShowNum(byte(2),line2,Openmv_Mode,1,6); //10
		}
}


/*
1号舵机 TIM1CH1-A8  720放 340 抓
300 480 660 
2号舵机 TIM1CH4-A11 240 前  860后   start760 max 240
300 760 700
3号舵机 TIM8CH1-C6  180收 740放
200 820 800
4号舵机 TIM8CH2-C7  400夹死 280放
480 360 360夹死

5号舵机 TIM8CH3-C8
u16 tim1ch1=480;
u16 tim1ch4=700;
u16 tim8ch1=810;
u16 tim8ch2=475;

get1 3
355
405->450
605
475

putdown
660
360->450->700
545
380

get 2
305
415]]
645
475->380

putdown2    165mm
660
415->500->700
630
380->475
*/
/*********************************************END OF FILE**********************/
