/*******************************************************************************************
										    �� ��
    ����Ŀ�����������ѧϰʹ�ã�����������ֲ�޸ģ������뱣����������Ϣ����ֲ�����г�������
	
���ɹ�����BUG��������������κ����Ρ��������ã�

* ����汾��V1.01
* �������ڣ�2018-8-18
* �������ߣ���ŭ��С��
* ��Ȩ���У��������������Ϣ�������޹�˾
*******************************************************************************************/
#include "stm32f10x.h"
#include "hmciic.h"
#include "bsp_SysTick.h"


////////////////////////////////////////// ��ֲ��֪��//////////////////////////////////////
//
// ��ֲ��ģ��IICʱ����Ҫע�� IIC_WriteByteToSlave() д�������ĵ�ַʱ�����ַ�ʽ�ĵ�ַ���Ѿ�
// ���ƹ�1λ�ģ�ע���� ( I2C_Addr <<1 | 0 )�ķ�ʽ�������֣��������������ͨ��,������ֲ!
//
///////////////////////////////////////////////////////////////////////////////////////////


/******************************************************************************
* ��  ����void IIC_GPIO_Init(void)
* �����ܣ�ģ��IIC���ų�ʼ��
* ��  ������
* ����ֵ����
* ��  ע��PB6->SCL	PB7->SDA
*******************************************************************************/
void HMCIIC_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;   
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);   
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5; 
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP; 
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;  
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	GPIO_SetBits(GPIOD,GPIO_Pin_4|GPIO_Pin_5);
}

/******************************************************************************
* ��  ����void IIC_Start(void)
* �����ܣ�����IIC��ʼ�ź�
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/	
void HMCIIC_Start(void)
{
	HMCSDA_OUT(); //SDA����� 
	HMCSDA_H;
	HMCSCL_H;	
	Delay_us(4);
 	HMCSDA_L;
	Delay_us(4);
	HMCSCL_L;
}

/******************************************************************************
* ��  ����void IIC_Stop(void)
* �����ܣ�����IICֹͣ�ź�
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/	  
void HMCIIC_Stop(void)
{
	HMCSDA_OUT(); //SDA�����
	HMCSCL_L;
	HMCSDA_L;
	Delay_us(4);
	HMCSCL_H; 
	HMCSDA_H;
	Delay_us(4);							   	
}

/******************************************************************************
* ��  ��: uint8_t IIC_WaitAck(void)
* ������: �ȴ�Ӧ���źŵ��� ����ЧӦ�𣺴ӻ���9�� SCL=0 ʱ SDA ���ӻ�����,
*         ���� SCL = 1ʱ SDA��ȻΪ�ͣ�
* ��  ������
* ����ֵ��1������Ӧ��ʧ��	0������Ӧ��ɹ�
* ��  ע���ӻ���������Ӧ��
*******************************************************************************/
uint8_t HMCIIC_WaitAck(void)
{
	uint8_t ucErrTime=0;
	HMCSDA_IN(); //SDA����Ϊ����  ���ӻ���һ���͵�ƽ��ΪӦ�� 
	HMCSDA_H;
	Delay_us(1);	   
	HMCSCL_H;
	Delay_us(1);;	 
	while(HMCSDA_read)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			HMCIIC_Stop();
			return 1;
		}
	}
	HMCSCL_L; //ʱ�������	   
	return 0;  
} 

/******************************************************************************
* ��  ��: void IIC_Ack(void)
* ������: ����ACKӦ�� ������������һ���ֽ����ݺ�����������ACK֪ͨ�ӻ�һ��
*         �ֽ���������ȷ���գ�
* ��  ������
* ����ֵ����
* ��  ע���������ӻ���Ӧ��
*******************************************************************************/
void HMCIIC_Ack(void)
{
	HMCSCL_L;
	HMCSDA_OUT();
	HMCSDA_L;
	Delay_us(1);
	HMCSCL_H;
	Delay_us(2);
	HMCSCL_L;
}

/******************************************************************************
* ��  ��: void IIC_NAck(void)
* ������: ����NACKӦ�� ���������������һ���ֽ����ݺ�����������NACK֪ͨ�ӻ�
*         ���ͽ������ͷ�SDA,�Ա���������ֹͣ�źţ�
* ��  ������
* ����ֵ����
* ��  ע���������ӻ���Ӧ��
*******************************************************************************/
void HMCIIC_NAck(void)
{
	HMCSCL_L;
	HMCSDA_OUT();
	HMCSDA_H;
	Delay_us(1);
	HMCSCL_H;
	Delay_us(1);
	HMCSCL_L;
}					 				     

/******************************************************************************
* ��  ����void IIC_SendByte(uint8_t data)
* ��  �ܣ�IIC����һ���ֽ�
* ��  ����data Ҫд������
* ����ֵ����
* ��  ע���������ӻ���
*******************************************************************************/		  
void HMCIIC_SendByte(uint8_t data)
{                        
    uint8_t t;   
		HMCSDA_OUT(); 	    
    HMCSCL_L; //����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {
		if(data&0x80)
			HMCSDA_H;
		else
			HMCSDA_L;
		data <<= 1;
		Delay_us(1);			
		HMCSCL_H;
		Delay_us(1);
		HMCSCL_L;	
		Delay_us(1);
    }	 
} 	 
   
/******************************************************************************
* ��  ����uint8_t IIC_ReadByte(uint8_t ack)
* ��  �ܣ�IIC��ȡһ���ֽ�
* ��  ����ack=1 ʱ���������ݻ�û������ ack=0 ʱ����������ȫ���������
* ����ֵ����
* ��  ע���ӻ���������
*******************************************************************************/	
uint8_t HMCIIC_ReadByte(uint8_t ack)
{
	uint8_t i,receive=0;
	HMCSDA_IN(); //SDA����Ϊ����ģʽ �ȴ����մӻ���������
	for(i=0;i<8;i++ )
	{
		HMCSCL_L; 
		Delay_us(1);
		HMCSCL_H;
		receive<<=1;
		if(HMCSDA_read)receive++; //�ӻ����͵ĵ�ƽ
			Delay_us(1); 
	}					 
    if(ack)
        HMCIIC_Ack(); //����ACK 
    else
        HMCIIC_NAck(); //����nACK  
    return receive;
}

/******************************************************************************
* ��  ����uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t addr)
* �����ܣ���ȡָ���豸 ָ���Ĵ�����һ��ֵ
* ��  ����I2C_Addr  Ŀ���豸��ַ
		  reg	    �Ĵ�����ַ
          *buf      ��ȡ����Ҫ�洢�ĵ�ַ    
* ����ֵ������ 1ʧ�� 0�ɹ�
* ��  ע����
*******************************************************************************/ 
uint8_t HMCIIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	HMCIIC_Start();	
	HMCIIC_SendByte(I2C_Addr);	 //���ʹӻ���ַ
	if(HMCIIC_WaitAck()) //����ӻ�δӦ�������ݷ���ʧ��
	{
		HMCIIC_Stop();
		return 1;
	}
	HMCIIC_SendByte(reg); //���ͼĴ�����ַ
	HMCIIC_WaitAck();	  
	
	HMCIIC_Start();
	HMCIIC_SendByte(I2C_Addr+1); //�������ģʽ			   
	HMCIIC_WaitAck();
	*buf=HMCIIC_ReadByte(0);	   
	HMCIIC_Stop(); //����һ��ֹͣ����
	return 0;
}

/*************************************************************************************
* ��  ����uint8_t IIC_WriteByteFromSlave(uint8_t I2C_Addr,uint8_t addr��uint8_t buf))
* �����ܣ�д��ָ���豸 ָ���Ĵ�����һ��ֵ
* ��  ����I2C_Addr  Ŀ���豸��ַ
		  reg	    �Ĵ�����ַ
          buf       Ҫд�������
* ����ֵ��1 ʧ�� 0�ɹ�
* ��  ע����
**************************************************************************************/ 
uint8_t HMCIIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	HMCIIC_Start();
	HMCIIC_SendByte(I2C_Addr); //���ʹӻ���ַ
	if(HMCIIC_WaitAck())
	{
		HMCIIC_Stop();
		return 1; //�ӻ���ַд��ʧ��
	}
	HMCIIC_SendByte(reg); //���ͼĴ�����ַ
	HMCIIC_WaitAck();	  
	HMCIIC_SendByte(data); 
	if(HMCIIC_WaitAck())
	{
		HMCIIC_Stop(); 
		return 1; //����д��ʧ��
	}
	HMCIIC_Stop(); //����һ��ֹͣ����
	return 0;
}

/***************************************************************************************
* ��  ����uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
* �����ܣ���ȡָ���豸 ָ���Ĵ����� length��ֵ
* ��  ����dev     Ŀ���豸��ַ
		  reg	  �Ĵ�����ַ
          length  Ҫ�����ֽ���
		  *data   ���������ݽ�Ҫ��ŵ�ָ��
* ����ֵ��1�ɹ� 0ʧ��
* ��  ע����
***************************************************************************************/ 
uint8_t HMCIIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t count = 0;
	uint8_t temp;
	HMCIIC_Start();
	HMCIIC_SendByte(dev); //���ʹӻ���ַ
	if(HMCIIC_WaitAck())
	{
		HMCIIC_Stop(); 
		return 1; //�ӻ���ַд��ʧ��
	}
	HMCIIC_SendByte(reg); //���ͼĴ�����ַ
	HMCIIC_WaitAck();	  
	HMCIIC_Start();
	HMCIIC_SendByte(dev+1); //�������ģʽ	
	HMCIIC_WaitAck();
	for(count=0;count<length;count++)
	{
		if(count!=(length-1))
			temp = HMCIIC_ReadByte(1); //��ACK�Ķ�����
		else  
			temp = HMCIIC_ReadByte(0); //���һ���ֽ�NACK

		data[count] = temp;
	}
    HMCIIC_Stop(); //����һ��ֹͣ����
	return 0;
}

/****************************************************************************************
* ��  ����uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
* �����ܣ�������ֽ�д��ָ���豸 ָ���Ĵ���
* ��  ����dev     Ŀ���豸��ַ
		  reg	  �Ĵ�����ַ
		  length  Ҫд���ֽ���
		  *data   Ҫд������ݽ�Ҫ��ŵ�ָ��
* ����ֵ��1�ɹ� 0ʧ��
* ��  ע����
****************************************************************************************/ 
uint8_t HMCIIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
 	uint8_t count = 0;
	HMCIIC_Start();
	HMCIIC_SendByte(dev); //���ʹӻ���ַ
	if(HMCIIC_WaitAck())
	{
		HMCIIC_Stop();
		return 1; //�ӻ���ַд��ʧ��
	}
	HMCIIC_SendByte(reg); //���ͼĴ�����ַ
	HMCIIC_WaitAck();	  
	for(count=0;count<length;count++)
	{
		HMCIIC_SendByte(data[count]); 
		if(HMCIIC_WaitAck()) //ÿһ���ֽڶ�Ҫ�ȴӻ�Ӧ��
		{
			HMCIIC_Stop();
			return 1; //����д��ʧ��
		}
	}
	HMCIIC_Stop(); //����һ��ֹͣ����
	return 0;
}


