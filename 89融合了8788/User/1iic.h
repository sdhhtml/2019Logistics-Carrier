#ifndef   _1IIC_H
#define   _1IIC_H
#include "stdint.h"
#include "stm32f10x.h"
#define SCL_H1         GPIOD->BSRR = GPIO_Pin_4
#define SCL_L1         GPIOD->BRR  = GPIO_Pin_4
#define SDA_H1         GPIOD->BSRR = GPIO_Pin_5
#define SDA_L1         GPIOD->BRR  = GPIO_Pin_5
#define SDA_read1      ((GPIOD->IDR & GPIO_Pin_5)!=0)?1:0
//IO��������
 
#define SDA_IN1()  {GPIOD->CRL&=0XFF0FFFFF;GPIOD->CRL|=(u32)8<<28;}
#define SDA_OUT1() {GPIOD->CRL&=0XFF0FFFFF;GPIOD->CRL|=(u32)3<<28;}
void IIC_GPIO_Init1(void);        //��ʼ��IIC��IO��				 
void IIC_Start1(void);			 //����IIC��ʼ�ź�
void IIC_Stop1(void);	  	  	 //����IICֹͣ�ź�
void IIC_Ack1(void);				 //IIC����ACK�ź�
void IIC_NAck1(void);			 //IIC������ACK�ź�
uint8_t IIC_WaitAck1(void); 		 //IIC�ȴ�ACK�ź�

void IIC_SendByte1(uint8_t data);  //IIC����һ���ֽ�
uint8_t IIC_ReadByte1(uint8_t ack);//IIC��ȡһ���ֽ�

uint8_t IIC_ReadByteFromSlave1(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t IIC_ReadMultByteFromSlave1(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_WriteByteToSlave1(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t IIC_WriteMultByteToSlave1(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);


#endif

