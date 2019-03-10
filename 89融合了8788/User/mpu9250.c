#include "mpu9250.h"
#include "stdio.h"
#include "DataScope_DP.h"
#include "oled.h"
#include "stm32f10x.h"
#include "mpu9250.h"
#include "iic.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "led.h"
#include "stdio.h"
#include "structconfig.h"
#include "paramsave.h"
#include "math.h"
#include "imu.h"
uint8_t    MPU9250_buff[14];                  //���ٶ� ������ �¶� ԭʼ����
static uint8_t MAG_buff[6];
extern INT16_XYZ	 GYRO_OFFSET_RAW,ACC_OFFSET_RAW;		 //��Ư����
INT16_XYZ	 MPU9250_ACC_RAW,MPU9250_GYRO_RAW;	     //��ȡֵԭʼ����
extern uint8_t    	 SENSER_OFFSET_FLAG;                     //������У׼��־λ
INT16_XYZ	 MAG_OFFSET_RAW;
INT16_XYZ MPU9250_MAG_RAW;	     //�ش�
double angle=0;
int angle_off=0;
/*****************************************************************************
* ��  ����uint8_t MPU6050_WriteByte(uint8_t addr,uint8_t reg,uint8_t data)
* ��  �ܣ�дһ���ֽ����ݵ� MPU6050 �Ĵ���
* ��  ����reg�� �Ĵ�����ַ
*         data: Ҫд�������
* ����ֵ��0�ɹ� 1ʧ��
* ��  ע��MPU6050������ֲֻ���I2C�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t MPU9250_WriteByte(uint8_t reg,uint8_t data)
{
	if(IIC_WriteByteToSlave(MPU9250Addr,reg,data))
		return 1;
	else
		return 0;
}

/*****************************************************************************
* ��  ����uint8_t MPU6050_ReadByte(uint8_t reg,uint8_t *buf)
* ��  �ܣ���ָ��MPU6050�Ĵ�����ȡһ���ֽ�����
* ��  ����reg�� �Ĵ�����ַ
*         buf:  ��ȡ���ݴ�ŵĵ�ַ
* ����ֵ��1ʧ�� 0�ɹ�
* ��  ע��MPU6050������ֲֻ���I2C�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t MPU9250_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(IIC_ReadByteFromSlave(MPU9250Addr,reg,buf))
		return 1;
	else
		return 0;
}

/*****************************************************************************
* ��  ����uint8_t MPU6050_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
* ��  �ܣ���ָ���Ĵ���д��ָ����������
* ��  ����reg���Ĵ�����ַ
*         len��д�����ݳ��� 
*         buf: д�����ݴ�ŵĵ�ַ
* ����ֵ��0�ɹ� 1ʧ��
* ��  ע��MPU6050������ֲֻ���I2C�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t MPU9250_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_WriteMultByteToSlave(MPU9250Addr,reg,len,buf))
		return 1;
	else
		return 0;
}

/*****************************************************************************
* ��  ����uint8_t MPU6050_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
* ��  �ܣ���ָ���Ĵ�����ȡָ����������
* ��  ����reg���Ĵ�����ַ
*         len����ȡ���ݳ��� 
*         buf: ��ȡ���ݴ�ŵĵ�ַ
* ����ֵ��0�ɹ� 0ʧ��
* ��  ע��MPU6050������ֲֻ���I2C�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t MPU9250_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_ReadMultByteFromSlave(MPU9250Addr,reg,len,buf))
		return 1;
	else
		return 0;
}
uint8_t MAG_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_ReadMultByteFromSlave(MAG_ADDRESS,reg,len,buf))
		return 1;
	else
		return 0;
}
/*============================���ϴ�����ֲʱ��Ҫ�޸�=========================*/

/******************************************************************************
* ��  ����uint8_tMPU6050_getDeviceID(void)
* ��  �ܣ���ȡ  MPU6050 WHO_AM_I ��ʶ������ 0x68
* ��  ������
* ����ֵ�����ض�ȡ����
* ��  ע����
*******************************************************************************/
uint8_t MPU9250_getDeviceID(void)
{
	uint8_t buf;
	MPU9250_ReadByte(MPU9250_RA_WHO_AM_I, &buf);

	printf("%d\r\n",buf);
	return buf;
}

/******************************************************************************
* ��  ����uint8_tMPU6050_testConnection(void)
* ��  �ܣ����MPU6050 �Ƿ��Ѿ�����
* ��  ������
* ����ֵ��1������ 0δ����
* ��  ע����
*******************************************************************************/
uint8_t MPU9250_testConnection(void) 
{
	if(MPU9250_getDeviceID() == 104)  

		
		return 1;
	else 
		return 0;
}

/******************************************************************************
* ��  ����void MPU6050_Check()
* ��  �ܣ����IIC�����ϵ�MPU6050�Ƿ����
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU9250_Check(void) 
{ 
	while(!MPU9250_testConnection())
	{
		printf("\rMPU9250 no connect...\r\n");
//		RGB_LED_green(); //�̵Ƴ���
	}
}

/******************************************************************************
* ��  ����void MPU6050_AccRead(int16_t *accData)
* ��  �ܣ���ȡ���ٶȵ�ԭʼ����
* ��  ����*accData ԭʼ���ݵ�ָ��
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU9250_AccRead(int16_t *accData)
{
    uint8_t buf[6];
   	MPU9250_ReadMultBytes(MPU9250_RA_ACCEL_XOUT_H,6,buf);
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

/******************************************************************************
* ��  ����void MPU6050_GyroRead(int16_t *gyroData)
* ��  �ܣ���ȡ�����ǵ�ԭʼ����
* ��  ����*gyroData ԭʼ���ݵ�ָ��
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU9250_GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
		MPU9250_ReadMultBytes(MPU9250_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]) ;
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]) ;
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]) ;
}

/******************************************************************************
* ��  ����void MPU6050_TempRead(float *tempdata)
* ��  �ܣ��¶�ֵ��ȡ
* ��  ����*tempdata �¶����ݵ�ָ��
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU9250_TempRead(float *tempdata)
{
	uint8_t buf[2];
	short data;
	MPU9250_ReadMultBytes(MPU9250_RA_TEMP_OUT_H, 2, buf);
	data = (int16_t)((buf[0] << 8) | buf[1]) ;
	*tempdata = 36.53f + ((float)data/340.0f);
}
/******************************************************************************
* ��  ����void MPU6050_MagRead(int16_t *gyroData)
* ��  �ܣ���ȡ�����ǵ�ԭʼ����
* ��  ����*gyroData ԭʼ���ݵ�ָ��
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU9250_MagRead(int16_t *magData)
{
    uint8_t buf[6];
		IIC_WriteByteToSlave(MPU9250Addr,0x37,0x02);
		Delay_ms(10);
		IIC_WriteByteToSlave(MAG_ADDRESS,0x0A,0x01);
		Delay_ms(10);	
		MAG_ReadMultBytes(MAG_XOUT_L, 6, buf);
    magData[0] = (int16_t)((buf[1] << 8) | buf[0]) ;
    magData[1] = (int16_t)((buf[3] << 8) | buf[2]) ;
    magData[2] = (int16_t)((buf[5] << 8) | buf[4]) ;
}
/******************************************************************************
* ��  ����void MPU6050_Init(void)
* ��  �ܣ���ʼ��MPU6050���빤��״̬
* ��  ������
* ����ֵ����
* ��  ע��DLPF �����Ϊ����Ƶ�ʵ�һ�룡����
*******************************************************************************/
/*
void MPU9250_Init(void)
{

	MPU9250_Check(); //���MPU6050�Ƿ�����

	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x80); //��λMPU6050
	Delay_ms(100);
	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x00); //����MPU6050����ѡ��������x��PLLΪʱ��Դ
	MPU9250_WriteByte(0x19, 0x00);
	MPU9250_WriteByte(0X1A,0x04);
//	MPU9250_WriteByte(MPU9250_RA_INT_ENABLE, 0x00); //��ֹ�ж�
	MPU9250_WriteByte(MPU9250_RA_GYRO_CONFIG, 0x18); //������������+-2000��/�� (��ͷֱ��� = 2^15/2000 = 16.4LSB/��/�� 
	MPU9250_WriteByte(MPU9250_RA_ACCEL_CONFIG, 0x10); //���ٶ�������+-4g   (��ͷֱ��� = 2^15/4g = 8196LSB/g )
	MPU9250_WriteByte(0x6C,0X00);
//	MPU9250_WriteByte(MPU9250_RA_CONFIG, MPU9250_DLPF_BW_20);//�������ݵ����Ϊ1kHZ,DLPF=20Hz 
//	MPU9250_WriteByte(MPU9250_RA_SMPLRT_DIV, 0x00);  //������Ƶ (����Ƶ�� = ���������Ƶ�� / (1+DIV)������Ƶ��1000hz��
//	MPU9250_WriteByte(MPU9250_RA_INT_PIN_CFG, 0x02); //MPU ��ֱ�ӷ���MPU6050����I2C
//	MPU9250_WriteByte(0x6a, 0x00); 
	
	GYRO_Offset();


}
*/
/******************************************************************************
* ��  ����void MPU6050_CalOff(void)
* ��  �ܣ������Ǽ��ٶ�У׼
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU9250_CalOff(void)
{

	 SENSER_FLAG_SET(ACC_OFFSET);//���ٶ�У׼
	 SENSER_FLAG_SET(GYRO_OFFSET);//������У׼
}
void MAG_CalOff(void)
{

	 SENSER_FLAG_SET(MAG_OFFSET);//���ٶ�У׼
}
/******************************************************************************
* ��  ����void MPU6050_CalOff_Acc(void)
* ��  �ܣ����ٶȼ�У׼
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU9250_CalOff_Acc(void)
{
	 SENSER_FLAG_SET(ACC_OFFSET);//���ٶ�У׼
}

/******************************************************************************
* ��  ����void MPU6050_CalOff_Gyr(void)
* ��  �ܣ�������У׼
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU9250_CalOff_Gyr(void)
{
	 SENSER_FLAG_SET(GYRO_OFFSET);//������У׼
}

/******************************************************************************
* ��  ����void MPU6050_Read(void)
* ��  �ܣ���ȡ�����Ǽ��ٶȼƵ�ԭʼ����
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU9250_Read(void)
{
//	int iii;
	MPU9250_ReadMultBytes(MPU9250_RA_ACCEL_XOUT_H, 14, MPU9250_buff);// ��ѯ����ȡMPU6050��ԭʼ����
//	for(iii=0;iii<14;iii++)
//	{
	//printf("%c\r\n",MPU6050_buff[iii]);
//	}
	//printf("sss\r\n");
}
void MAG_Read(void)
{
//	int iii;
	IIC_WriteByteToSlave(MPU9250Addr,0x37,0x02);
	Delay_ms(10);
	IIC_WriteByteToSlave(MAG_ADDRESS,0x0A,0x01);
	Delay_ms(10);		
	MAG_ReadMultBytes(MAG_XOUT_L, 6, MAG_buff);// ��ѯ����ȡMPU6050��ԭʼ����
	
//	for(iii=0;iii<14;iii++)
//	{
	//printf("%c\r\n",MPU6050_buff[iii]);
//	}
	//printf("sss\r\n");
}
/******************************************************************************
* ��  ����uint8_t MPU6050_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
* ��  �ܣ�MPU6050��ƫУ׼
* ��  ����value�� 	 MPU6050ԭʼ����
*         offset��	 У׼�����ƫֵ
*         sensivity�����ٶȼƵ�������
* ����ֵ��1У׼��� 0У׼δ���
* ��  ע����
*******************************************************************************/
uint8_t MPU9250_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
{
	static int32_t tempgx=0,tempgy=0,tempgz=0; 
	static uint16_t cnt_a=0;//ʹ��static���εľֲ������������α������о�̬�洢���ڣ�Ҳ����˵�ú���ִ������ͷ��ڴ�
	if(cnt_a==0)
	{
		value.X=0;
		value.Y=0;
		value.Z=0;
		tempgx = 0;
		tempgy = 0;
		tempgz = 0;
		cnt_a = 1;
		sensivity = 0;
		offset->X = 0;
		offset->Y = 0;
		offset->Z = 0;
	}
	tempgx += value.X;
	tempgy += value.Y; 
	tempgz += value.Z-sensivity ;//���ٶȼ�У׼ sensivity ���� MPU6050��ʼ��ʱ���õ�������ֵ��8196LSB/g��;������У׼ sensivity = 0��
	if(cnt_a==200)               //200����ֵ��ƽ��
	{
		offset->X=tempgx/cnt_a;
		offset->Y=tempgy/cnt_a;
		offset->Z=tempgz/cnt_a;
		cnt_a = 0;
		return 1;
	}
	cnt_a++;
	return 0;
}	
void MPU9250_MAG_OffSet(void)
{

   char count,i,j;
   int32_t 	value_buf[36];
   int32_t  sum1=0,sum2=0,sum3=0,temp=0;
   for  (count=0;count<N;count++)
   {
		 MAG_Read();
		 value_buf[count]=((((int16_t)MAG_buff[1]) << 8) | MAG_buff[0]) - MAG_OFFSET_RAW.X;
		 value_buf[count+12]=((((int16_t)MAG_buff[3]) << 8) | MAG_buff[2]) - MAG_OFFSET_RAW.Y;
		 value_buf[count+24]=((((int16_t)MAG_buff[5]) << 8) | MAG_buff[4]) - MAG_OFFSET_RAW.Z;
   }
    for (j=0;j<N-1;j++)
   {
      for (i=0;i<N-j;i++)
      {
         if ( value_buf[i]>value_buf[i+1] )
         {
            temp = value_buf[i];
            value_buf[i] = value_buf[i+1]; 
             value_buf[i+1] = temp;
         }
         if ( value_buf[i+12]>value_buf[i+13] )
         {
            temp = value_buf[i+12];
            value_buf[i+12] = value_buf[i+13]; 
             value_buf[i+13] = temp;
         }
         if ( value_buf[i+24]>value_buf[i+25] )
         {
            temp = value_buf[i+24];
            value_buf[i+24] = value_buf[i+25]; 
             value_buf[i+25] = temp;
         }				 
      }
   }

   for(count=1;count<N-1;count++)
	 {
			sum1 += value_buf[count];
			sum2 += value_buf[count+12];
			sum3 += value_buf[count+24];
	 }
   MAG_OFFSET_RAW.X=sum1/(N-2);
	 MAG_OFFSET_RAW.Y=sum2/(N-2);
	 MAG_OFFSET_RAW.Z=sum3/(N-2);
}	

/******************************************************************************
* ��  ����void MPU6050_DataProcess(void)
* ��  �ܣ���MPU6050����ȥ��ƫ����
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU9250_Offset(void)
{
	//���ٶ�ȥ��ƫADֵ 
	MPU9250_ACC_RAW.X =((((int16_t)MPU9250_buff[0]) << 8) | MPU9250_buff[1]) - ACC_OFFSET_RAW.X;
	MPU9250_ACC_RAW.Y =((((int16_t)MPU9250_buff[2]) << 8) | MPU9250_buff[3]) - ACC_OFFSET_RAW.Y;
	MPU9250_ACC_RAW.Z =((((int16_t)MPU9250_buff[4]) << 8) | MPU9250_buff[5]) - ACC_OFFSET_RAW.Z;
	//������ȥ��ƫADֵ 
	MPU9250_GYRO_RAW.X =((((int16_t)MPU9250_buff[8]) << 8) | MPU9250_buff[9]) - GYRO_OFFSET_RAW.X;
	MPU9250_GYRO_RAW.Y =((((int16_t)MPU9250_buff[10]) << 8) | MPU9250_buff[11]) - GYRO_OFFSET_RAW.Y;
	MPU9250_GYRO_RAW.Z =((((int16_t)MPU9250_buff[12]) << 8) | MPU9250_buff[13]) - GYRO_OFFSET_RAW.Z;
	
	if(GET_FLAG(GYRO_OFFSET)) //�����ǽ�����ƫУ׼
	{
		if(MPU9250_OffSet(MPU9250_GYRO_RAW,&GYRO_OFFSET_RAW,0))
		{
			
			 SENSER_FLAG_RESET(GYRO_OFFSET);
		//	 PID_WriteFlash(); //���������ǵ���ƫ����
			 //GYRO_Offset_LED();
		     SENSER_FLAG_SET(ACC_OFFSET);//У׼���ٶ�
			
//			 printf("GYRO_OFFSET_RAW Value :X=%d  Y=%d  Z=%d\n",GYRO_OFFSET_RAW.X,GYRO_OFFSET_RAW.Y,GYRO_OFFSET_RAW.Z);
//			 printf("\n");
		}
	}
	if(GET_FLAG(ACC_OFFSET)) //���ٶȼƽ�����ƫУ׼ 
	{
		if(MPU9250_OffSet(MPU9250_ACC_RAW,&ACC_OFFSET_RAW,8196))
		{
			 SENSER_FLAG_RESET(ACC_OFFSET);
			// PID_WriteFlash(); //������ٶȼƵ���ƫ����
			// ACC_Offset_LED();
//			 printf("ACC_OFFSET_RAW Value X=%d  Y=%d  Z=%d\n",ACC_OFFSET_RAW.X,ACC_OFFSET_RAW.Y,ACC_OFFSET_RAW.Z); 
//			 printf("\n");
		}
	}
}

void MAG_Offset(void)
{
	unsigned char ii;          //��������
	unsigned char Send_Count; //������Ҫ���͵����ݸ���	
	MAG_Read();
	MPU9250_MAG_RAW.X =((((int16_t)MAG_buff[1]) << 8) | MAG_buff[0]) - MAG_OFFSET_RAW.X;
	MPU9250_MAG_RAW.Y =((((int16_t)MAG_buff[3]) << 8) | MAG_buff[2]) - MAG_OFFSET_RAW.Y;
	MPU9250_MAG_RAW.Z =((((int16_t)MAG_buff[5]) << 8) | MAG_buff[4]) - MAG_OFFSET_RAW.Z;
/*
	DataScope_Get_Channel_Data( MPU9250_MAG_RAW.X , 1 );  //������ 1.0  д��ͨ�� 1
	DataScope_Get_Channel_Data( MPU9250_MAG_RAW.Y , 2 );  //������ 2.0  д��ͨ�� 2
	DataScope_Get_Channel_Data( MPU9250_MAG_RAW.Z , 3 );  //������ 3.0  д��ͨ�� 3	
	Send_Count = DataScope_Data_Generate(3); //����3��ͨ���� ��ʽ��֡���ݣ�����֡���ݳ���
	for( ii = 0 ; ii < Send_Count; ii++)  //ѭ������,ֱ���������   
	{
		while((USART1->SR&0X40)==0); 
			USART1->DR = DataScope_OutPut_Buffer[ii]; //�Ӵ��ڶ�һ���ֽ����ݳ�ȥ      
	 }	
*/
}
void mag_once(void)
{
	int16_t 	value_buf[2];
	double angle1=0;
	IIC_WriteByteToSlave(MPU9250Addr,0x37,0x02);
	Delay_ms(10);
	IIC_WriteByteToSlave(MAG_ADDRESS,0x0A,0x01);
	Delay_ms(10);	
	
	MAG_ReadMultBytes(MAG_XOUT_L, 4, MAG_buff);// ��ѯ����ȡMPU6050��ԭʼ����	
	value_buf[0]=((((int16_t)MAG_buff[1]) << 8) | MAG_buff[0]) - MAG_OFFSET_RAW.X;
	value_buf[1]=((((int16_t)MAG_buff[3]) << 8) | MAG_buff[2]) - MAG_OFFSET_RAW.Y;
	angle1= atan2(value_buf[1],value_buf[0]) * (180 / 3.14159265) + 180;
	if(angle1<0)
	{
		OLED_ShowString(byte(0),line6,"ANGLE1:-",6);
		angle1 = -1*angle1;
	}else
	{
		OLED_ShowString(byte(0),line6,"ANGLE1:",6);
		angle1=angle1;
	}	 

		OLED_ShowNum(byte(6),line6,angle1,3,6); //�����	
}

void medianfilter(void)
{
	
	unsigned char ii;          //��������
	unsigned char Send_Count; //������Ҫ���͵����ݸ���	
	
   char count,i,j;
   int16_t 	value_buf[36];
   int16_t  sum1=0,sum2=0,sum3=0,temp=0;
	 int16_t MAGY;
   for  (count=0;count<N;count++)
   {
		 MAG_Read();
		 value_buf[count]=((((int16_t)MAG_buff[1]) << 8) | MAG_buff[0]) - MAG_OFFSET_RAW.X;
		 value_buf[count+12]=((((int16_t)MAG_buff[3]) << 8) | MAG_buff[2]) - MAG_OFFSET_RAW.Y;
		 value_buf[count+24]=((((int16_t)MAG_buff[5]) << 8) | MAG_buff[4]) - MAG_OFFSET_RAW.Z;
   }
    for (j=0;j<N-1;j++)
   {
      for (i=0;i<N-j;i++)
      {
         if ( value_buf[i]>value_buf[i+1] )
         {
            temp = value_buf[i];
            value_buf[i] = value_buf[i+1]; 
             value_buf[i+1] = temp;
         }
         if ( value_buf[i+12]>value_buf[i+13] )
         {
            temp = value_buf[i+12];
            value_buf[i+12] = value_buf[i+13]; 
             value_buf[i+13] = temp;
         }
         if ( value_buf[i+24]>value_buf[i+25] )
         {
            temp = value_buf[i+24];
            value_buf[i+24] = value_buf[i+25]; 
             value_buf[i+25] = temp;
         }				 
      }
   }

   for(count=1;count<N-1;count++)
	 {
			sum1 += value_buf[count];
			sum2 += value_buf[count+12];
			sum3 += value_buf[count+24];
	 }
   MPU9250_MAG_RAW.X=sum1/(N-2);
	 MPU9250_MAG_RAW.Y=sum2/(N-2);
	 MPU9250_MAG_RAW.Z=sum3/(N-2);
 #if 1
   angle= atan2(MPU9250_MAG_RAW.Y,MPU9250_MAG_RAW.X) * (180 / 3.14159265) + 180; 	 
//	 printf("\r\n%lf\r\n",angle);
#endif	 
		if(angle<0)
		{
			OLED_ShowString(byte(0),line8,"MAGY:-",6);
			MAGY = -1*angle;
		}else
		{
			OLED_ShowString(byte(0),line8,"MAGY:",6);
			MAGY=angle;
		}	 
		
		OLED_ShowNum(byte(6),line8,MAGY,3,6); //�����
	 status.MAG_flag=1;
#if 0
//	 printf("x=%d\r\ny=%d\r\nz=%d\r\n",MPU9250_MAG_RAW.X,MPU9250_MAG_RAW.Y,MPU9250_MAG_RAW.Z);
	 
	DataScope_Get_Channel_Data( MPU9250_MAG_RAW.X , 1 );  //������ 1.0  д��ͨ�� 1
	DataScope_Get_Channel_Data( MPU9250_MAG_RAW.Y , 2 );  //������ 2.0  д��ͨ�� 2
	DataScope_Get_Channel_Data( MPU9250_MAG_RAW.Z , 3 );  //������ 3.0  д��ͨ�� 3	
	Send_Count = DataScope_Data_Generate(3); //����3��ͨ���� ��ʽ��֡���ݣ�����֡���ݳ���
	for( ii = 0 ; ii < Send_Count; ii++)  //ѭ������,ֱ���������   
	{
		while((USART1->SR&0X40)==0); 
			USART1->DR = DataScope_OutPut_Buffer[ii]; //�Ӵ��ڶ�һ���ֽ����ݳ�ȥ      
	 }	
#endif	
	 
}
void angle_offset(void)
{
	angle_off=angle;
}
void mpu9250quling(void)
{
}
/**/
