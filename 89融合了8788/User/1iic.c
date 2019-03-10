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
#include "1iic.h"
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
void IIC_GPIO_Init1(void)
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
void IIC_Start1(void)
{
	SDA_OUT1(); //SDA����� 
	SDA_H1;
	SCL_H1;	
	Delay_us(4);
 	SDA_L1;
	Delay_us(4);
	SCL_L1;
}

/******************************************************************************
* ��  ����void IIC_Stop(void)
* �����ܣ�����IICֹͣ�ź�
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/	  
void IIC_Stop1(void)
{
	SDA_OUT1(); //SDA�����
	SCL_L1;
	SDA_L1;
	Delay_us(4);
	SCL_H1; 
	SDA_H1;
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
uint8_t IIC_WaitAck1(void)
{
	uint8_t ucErrTime=0;
	SDA_IN1(); //SDA����Ϊ����  ���ӻ���һ���͵�ƽ��ΪӦ�� 
	SDA_H1;
	Delay_us(1);	   
	SCL_H1;
	Delay_us(1);;	 
	while(SDA_read1)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop1();
			return 1;
		}
	}
	SCL_L1; //ʱ�������	   
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
void IIC_Ack1(void)
{
	SCL_L1;
	SDA_OUT1();
	SDA_L1;
	Delay_us(1);
	SCL_H1;
	Delay_us(2);
	SCL_L1;
}

/******************************************************************************
* ��  ��: void IIC_NAck(void)
* ������: ����NACKӦ�� ���������������һ���ֽ����ݺ�����������NACK֪ͨ�ӻ�
*         ���ͽ������ͷ�SDA,�Ա���������ֹͣ�źţ�
* ��  ������
* ����ֵ����
* ��  ע���������ӻ���Ӧ��
*******************************************************************************/
void IIC_NAck1(void)
{
	SCL_L1;
	SDA_OUT1();
	SDA_H1;
	Delay_us(1);
	SCL_H1;
	Delay_us(1);
	SCL_L1;
}					 				     

/******************************************************************************
* ��  ����void IIC_SendByte(uint8_t data)
* ��  �ܣ�IIC����һ���ֽ�
* ��  ����data Ҫд������
* ����ֵ����
* ��  ע���������ӻ���
*******************************************************************************/		  
void IIC_SendByte1(uint8_t data)
{                        
    uint8_t t;   
	SDA_OUT1(); 	    
    SCL_L1; //����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {
		if(data&0x80)
			SDA_H1;
		else
			SDA_L1;
		data <<= 1;
		Delay_us(1);			
		SCL_H1;
		Delay_us(1);
		SCL_L1;	
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
uint8_t IIC_ReadByte1(uint8_t ack)
{
	uint8_t i,receive=0;
	SDA_IN1(); //SDA����Ϊ����ģʽ �ȴ����մӻ���������
	for(i=0;i<8;i++ )
	{
		SCL_L1; 
		Delay_us(1);
		SCL_H1;
		receive<<=1;
		if(SDA_read1)receive++; //�ӻ����͵ĵ�ƽ
			Delay_us(1); 
	}					 
    if(ack)
        IIC_Ack1(); //����ACK 
    else
        IIC_NAck1(); //����nACK  
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
uint8_t IIC_ReadByteFromSlave1(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	IIC_Start1();	
	IIC_SendByte1(I2C_Addr);	 //���ʹӻ���ַ
	if(IIC_WaitAck1()) //����ӻ�δӦ�������ݷ���ʧ��
	{
		IIC_Stop1();
		return 1;
	}
	IIC_SendByte1(reg); //���ͼĴ�����ַ
	IIC_WaitAck1();	  
	
	IIC_Start1();
	IIC_SendByte1(I2C_Addr+1); //�������ģʽ			   
	IIC_WaitAck1();
	*buf=IIC_ReadByte1(0);	   
	IIC_Stop1(); //����һ��ֹͣ����
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
uint8_t IIC_WriteByteToSlave1(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	IIC_Start1();
	IIC_SendByte1(I2C_Addr); //���ʹӻ���ַ
	if(IIC_WaitAck1())
	{
		IIC_Stop1();
		return 1; //�ӻ���ַд��ʧ��
	}
	IIC_SendByte1(reg); //���ͼĴ�����ַ
	IIC_WaitAck1();	  
	IIC_SendByte1(data); 
	if(IIC_WaitAck1())
	{
		IIC_Stop1(); 
		return 1; //����д��ʧ��
	}
	IIC_Stop1(); //����һ��ֹͣ����
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
uint8_t IIC_ReadMultByteFromSlave1(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t count = 0;
	uint8_t temp;
	IIC_Start1();
	IIC_SendByte1(dev); //���ʹӻ���ַ
	if(IIC_WaitAck1())
	{
		IIC_Stop1(); 
		return 1; //�ӻ���ַд��ʧ��
	}
	IIC_SendByte1(reg); //���ͼĴ�����ַ
	IIC_WaitAck1();	  
	IIC_Start1();
	IIC_SendByte1(dev+1); //�������ģʽ	
	IIC_WaitAck1();
	for(count=0;count<length;count++)
	{
		if(count!=(length-1))
			temp = IIC_ReadByte1(1); //��ACK�Ķ�����
		else  
			temp = IIC_ReadByte1(0); //���һ���ֽ�NACK

		data[count] = temp;
	}
    IIC_Stop1(); //����һ��ֹͣ����
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
uint8_t IIC_WriteMultByteToSlave1(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
 	uint8_t count = 0;
	IIC_Start1();
	IIC_SendByte1(dev); //���ʹӻ���ַ
	if(IIC_WaitAck1())
	{
		IIC_Stop1();
		return 1; //�ӻ���ַд��ʧ��
	}
	IIC_SendByte1(reg); //���ͼĴ�����ַ
	IIC_WaitAck1();	  
	for(count=0;count<length;count++)
	{
		IIC_SendByte1(data[count]); 
		if(IIC_WaitAck1()) //ÿһ���ֽڶ�Ҫ�ȴӻ�Ӧ��
		{
			IIC_Stop1();
			return 1; //����д��ʧ��
		}
	}
	IIC_Stop1(); //����һ��ֹͣ����
	return 0;
}


