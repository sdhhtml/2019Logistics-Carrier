#ifndef __LED_H
#define	__LED_H
#include "stm32f10x.h"
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����
/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)		 {p->BSRR=i;}	 //���Ϊ�ߵ�ƽ		
#define digitalLo(p,i)		 {p->BRR=i;}	 //����͵�ƽ
#define digitalToggle(p,i) {p->ODR ^=i;} //�����ת״̬
#define LED1_TOGGLE		 digitalToggle(GPIOB,GPIO_Pin_5)
#define LED2_TOGGLE		 digitalToggle(GPIOE,GPIO_Pin_5)
//#define LED3_TOGGLE		 digitalToggle(GPIOC,GPIO_Pin_2)
//#define LED4_TOGGLE		 digitalToggle(GPIOC,GPIO_Pin_3)
void led_init(void);
//ϵͳ״̬
struct GLOBAL_FLAGS {
  //�����������������ʱ��ֵΪ1
  unsigned char flag1;
  //�����ڽ��յ�����ʱ��ֵΪ1
  unsigned char flag2;
  //���������������ʱ,ֵΪ1
  unsigned char flag3;
	unsigned char TIM1CH1_up;
	unsigned char TIM1CH1_down;
	unsigned char TIM1CH4_up;
	unsigned char TIM1CH4_down;
	unsigned char TIM8CH1_up;
	unsigned char TIM8CH1_down;
	unsigned char TIM8CH2_up;
	unsigned char TIM8CH2_down;
	unsigned char MAG_flag;
	unsigned char uart5;
	unsigned char start;
//	unsigned char TIM8CH3;
};
struct HONGWAI_FLAGS {

  unsigned char x_0;
	unsigned char x_1;
  unsigned char x_2;
	unsigned char x_3;
	
	unsigned char y_0;
	unsigned char y_1;
	unsigned char y_2;
	unsigned char y_3;
	unsigned char y_4;
	unsigned char y_5;
	int x;
	int y;
};
extern struct GLOBAL_FLAGS status;
extern struct HONGWAI_FLAGS hongwaiflags;
#define TRUE 1
#define FALSE 0
#endif
/****/
