#include "oled.h"
#include "oledfont.h"
#include "bsp_SysTick.h"
//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 			   
/**********************************************
//IIC Start
**********************************************/
/**********************************************
//IIC Start
**********************************************/
void OLEDIIC_Start()
{

	OLED_SCLK_Set() ;
	OLED_SDIN_Set();
	OLED_SDIN_Clr();
	OLED_SCLK_Clr();
}

void OLEDIIC_Start1()
{

	OLED_SCLK_Set1() ;
	OLED_SDIN_Set1();
	OLED_SDIN_Clr1();
	OLED_SCLK_Clr1();
}
/**********************************************
//IIC Stop
**********************************************/
void OLEDIIC_Stop()
{
  OLED_SCLK_Set() ;
//	OLED_SCLK_Clr();
	OLED_SDIN_Clr();
	OLED_SDIN_Set();
	
}

void OLEDIIC_Stop1()
{
  OLED_SCLK_Set1() ;
//	OLED_SCLK_Clr();
	OLED_SDIN_Clr1();
	OLED_SDIN_Set1();
	
}

void OLEDIIC_Wait_Ack()
{

	//GPIOB->CRH &= 0XFFF0FFFF;	//设置PB12为上拉输入模式
	//GPIOB->CRH |= 0x00080000;
//	OLED_SDA = 1;
//	delay_us(1);
	//OLED_SCL = 1;
	//delay_us(50000);
/*	while(1)
	{
		if(!OLED_SDA)				//判断是否接收到OLED 应答信号
		{
			//GPIOB->CRH &= 0XFFF0FFFF;	//设置PB12为通用推免输出模式
			//GPIOB->CRH |= 0x00030000;
			return;
		}
	}
*/
	OLED_SCLK_Set();
	OLED_SCLK_Clr();
}

void OLEDIIC_Wait_Ack1()
{
		OLED_SCLK_Set1();
	OLED_SCLK_Clr1();
}
/**********************************************
// IIC Write byte
**********************************************/

void OLEDWrite_IIC_Byte(unsigned char IIC_Byte)
{
	unsigned char i;
	unsigned char m,da;
	da=IIC_Byte;
	OLED_SCLK_Clr();
	for(i=0;i<8;i++)		
	{
		m=da;
		//	OLED_SCLK_Clr();
		m=m&0x80;
		if(m==0x80)
		{OLED_SDIN_Set();}
		else OLED_SDIN_Clr();
			da=da<<1;
		OLED_SCLK_Set();
		OLED_SCLK_Clr();
	}


}

void OLEDWrite_IIC_Byte1(unsigned char IIC_Byte)
{
	unsigned char i;
	unsigned char m,da;
	da=IIC_Byte;
	OLED_SCLK_Clr1();
	for(i=0;i<8;i++)		
	{
		m=da;
		//	OLED_SCLK_Clr();
		m=m&0x80;
		if(m==0x80)
		{OLED_SDIN_Set1();}
		else OLED_SDIN_Clr1();
			da=da<<1;
		OLED_SCLK_Set1();
		OLED_SCLK_Clr1();
	}


}
/**********************************************
// IIC Write Command
**********************************************/
void OLEDWrite_IIC_Command(unsigned char IIC_Command)
{
   OLEDIIC_Start();
   OLEDWrite_IIC_Byte(0x78);       //Slave address,SA0=0
	OLEDIIC_Wait_Ack();	
   OLEDWrite_IIC_Byte(0x00);			 //write command
	OLEDIIC_Wait_Ack();	
   OLEDWrite_IIC_Byte(IIC_Command); 
	OLEDIIC_Wait_Ack();	
   OLEDIIC_Stop();
}

void OLEDWrite_IIC_Command1(unsigned char IIC_Command)
{
   OLEDIIC_Start1();
   OLEDWrite_IIC_Byte1(0x78);       //Slave address,SA0=0
	OLEDIIC_Wait_Ack1();	
   OLEDWrite_IIC_Byte1(0x00);			 //write command
	OLEDIIC_Wait_Ack1();	
   OLEDWrite_IIC_Byte1(IIC_Command); 
	OLEDIIC_Wait_Ack1();	
   OLEDIIC_Stop1();
}
/**********************************************
// IIC Write Data
**********************************************/
void OLEDWrite_IIC_Data(unsigned char IIC_Data)
{
  OLEDIIC_Start();
   OLEDWrite_IIC_Byte(0x78);			//D/C#=0; R/W#=0
	OLEDIIC_Wait_Ack();	
   OLEDWrite_IIC_Byte(0x40);			//write data
	OLEDIIC_Wait_Ack();	
   OLEDWrite_IIC_Byte(IIC_Data);
	OLEDIIC_Wait_Ack();	
   OLEDIIC_Stop();
}

void OLEDWrite_IIC_Data1(unsigned char IIC_Data)
{
  OLEDIIC_Start1();
   OLEDWrite_IIC_Byte1(0x78);			//D/C#=0; R/W#=0
	OLEDIIC_Wait_Ack1();	
   OLEDWrite_IIC_Byte1(0x40);			//write data
	OLEDIIC_Wait_Ack1();	
   OLEDWrite_IIC_Byte1(IIC_Data);
	OLEDIIC_Wait_Ack1();	
   OLEDIIC_Stop1();
}
void OLED_WR_Byte(unsigned dat,unsigned cmd)
{
	if(cmd)
	{
   OLEDWrite_IIC_Data(dat);
	}
	else 
	{
   OLEDWrite_IIC_Command(dat);	
	}

}

void OLED_WR_Byte1(unsigned dat,unsigned cmd)
{
	if(cmd)
	{
   OLEDWrite_IIC_Data1(dat);
	}
	else 
	{
   OLEDWrite_IIC_Command1(dat);	
	}

}
/********************************************
// fill_Picture
********************************************/
void fill_picture(unsigned char fill_Data)
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		OLED_WR_Byte(0xb0+m,0);		//page0-page1
		OLED_WR_Byte(0x00,0);		 //low column start address
		OLED_WR_Byte(0x10,0);		 //high column start address
		for(n=0;n<128;n++)
			{
				OLED_WR_Byte(fill_Data,1);
			}
	}
}

void fill_picture1(unsigned char fill_Data)
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		OLED_WR_Byte1(0xb0+m,0);		//page0-page1
		OLED_WR_Byte1(0x00,0);		 //low column start address
		OLED_WR_Byte1(0x10,0);		 //high column start address
		for(n=0;n<128;n++)
			{
				OLED_WR_Byte1(fill_Data,1);
			}
	}
}
/***********************Delay****************************************/
void Delay_50ms(unsigned int Del_50ms)
{
	unsigned int m;
	for(;Del_50ms>0;Del_50ms--)
		for(m=6245;m>0;m--);
}

void Delay_1ms(unsigned int Del_1ms)
{
	unsigned char j;
	while(Del_1ms--)
	{	
		for(j=0;j<123;j++);
	}
}

//坐标设置

void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 
	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f),OLED_CMD); 
}   	

void OLED_Set_Pos1(unsigned char x, unsigned char y) 
{ 
	OLED_WR_Byte1(0xb0+y,OLED_CMD);
	OLED_WR_Byte1(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte1((x&0x0f),OLED_CMD); 
} 
//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}

void OLED_Display_On1(void)
{
	OLED_WR_Byte1(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte1(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte1(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   	

void OLED_Display_Off1(void)
{
	OLED_WR_Byte1(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte1(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte1(0XAE,OLED_CMD);  //DISPLAY OFF
}
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)
			OLED_WR_Byte(0,OLED_DATA); 
	} //更新显示
}

void OLED_Clear1(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte1 (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte1 (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte1 (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)
			OLED_WR_Byte1(0,OLED_DATA); 
	} //更新显示
}
//清蓝屏函数	  
void OLED_ClearBlue(void)  
{  
	u8 i,n;		    
	for(i=2;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} //更新显示
}

void OLED_ClearBlue1(void)  
{  
	u8 i,n;		    
	for(i=2;i<8;i++)  
	{  
		OLED_WR_Byte1 (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte1 (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte1 (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte1(0,OLED_DATA); 
	} //更新显示
}
//清行函数
void OLED_ClearLine(u8 line)
{
	u8 n;
	OLED_WR_Byte (0xb0+line,OLED_CMD);    //设置页地址（0~7）
	OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
	OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
	for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
}

void OLED_ClearLine1(u8 line)
{
	u8 n;
	OLED_WR_Byte1 (0xb0+line,OLED_CMD);    //设置页地址（0~7）
	OLED_WR_Byte1 (0x00,OLED_CMD);      //设置显示位置—列低地址
	OLED_WR_Byte1 (0x10,OLED_CMD);      //设置显示位置—列高地址   
	for(n=0;n<128;n++)OLED_WR_Byte1(0,OLED_DATA); 
}
void OLED_On(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(1,OLED_DATA); 
	} //更新显示
}

void OLED_On1(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte1 (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte1 (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte1 (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte1(1,OLED_DATA); 
	} //更新显示
}
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 8/6
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size)
{      	
	unsigned char c=0,i=0;	
		c=chr-' ';//得到偏移后的值			
		if(x>Max_Column-1){x=0;y=y+2;}
		if(Char_Size ==8)//字体为8号
			{
			OLED_Set_Pos(x,y);	
			for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
			OLED_Set_Pos(x,y+1);
			for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
			}
			else {	 ////字体为6号
				OLED_Set_Pos(x,y);
				for(i=0;i<6;i++)
				OLED_WR_Byte(F6x8[c][i],OLED_DATA);
				
			}
}

void OLED_ShowChar1(u8 x,u8 y,u8 chr,u8 Char_Size)
{      	
	unsigned char c=0,i=0;	
		c=chr-' ';//得到偏移后的值			
		if(x>Max_Column-1){x=0;y=y+2;}
		if(Char_Size ==8)//字体为8号
			{
			OLED_Set_Pos1(x,y);	
			for(i=0;i<8;i++)
			OLED_WR_Byte1(F8X16[c*16+i],OLED_DATA);
			OLED_Set_Pos1(x,y+1);
			for(i=0;i<8;i++)
			OLED_WR_Byte1(F8X16[c*16+i+8],OLED_DATA);
			}
			else {	 ////字体为6号
				OLED_Set_Pos1(x,y);
				for(i=0;i<6;i++)
				OLED_WR_Byte1(F6x8[c][i],OLED_DATA);
				
			}
}
//m^n函数
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  

u32 oled_pow1(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}			
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+size*t,y,' ',size);
				continue;
			}
			else 
				enshow=1; 
		}
	 	OLED_ShowChar(x+size*t,y,temp+'0',size); 
	}
} 

void OLED_ShowNum1(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow1(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar1(x+size*t,y,' ',size);
				continue;
			}
			else 
				enshow=1; 
		}
	 	OLED_ShowChar1(x+size*t,y,temp+'0',size); 
	}
} 
//显示一个字符号串
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar(x,y,chr[j],Char_Size);
			x+=Char_Size;//字间距
		if(x>120) {x=0;y+=2;}
			j++;
	}
}

void OLED_ShowString1(u8 x,u8 y,u8 *chr,u8 Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar1(x,y,chr[j],Char_Size);
			x+=Char_Size;//字间距
		if(x>120) {x=0;y+=2;}
			j++;
	}
}
//显示汉字
//x,y :起点坐标	 
//data[][16] 汉字取模
//num:二维数组行数	16x16
void OLED_ShowCHinese(u8 x,u8 y,const char data[][16],u8 num)
{      			    
	u8 t;
	OLED_Set_Pos(x,y);//设置坐标
	for(t=0;t<16;t++)
	{
		OLED_WR_Byte(data[2*num][t],OLED_DATA);
	}	
	OLED_Set_Pos(x,y+1);//设置坐标
	for(t=0;t<16;t++)
	{	
		OLED_WR_Byte(data[2*num+1][t],OLED_DATA);
	}					
}

//显示汉字
//x,y :起点坐标	 
//data[][16] 汉字取模
//num:二维数组行数	7x7
void OLED_ShowCHinese7x7(u8 x,u8 y,const char data[][16],u8 num)
{      			    
	u8 t;
	OLED_Set_Pos(x,y);//设置坐标
	for(t=0;t<7;t++)
	{
		OLED_WR_Byte(data[2*num][t],OLED_DATA);
	}	
	OLED_Set_Pos(x,y+1);//设置坐标
	for(t=0;t<7;t++)
	{	
		OLED_WR_Byte(data[2*num+1][t],OLED_DATA);
	}					
}
//显示一串汉字
//x,y :起点坐标	 
//data[][16] 汉字取模
//num:二维数组行数	16x16
void OLED_ShowCHineseString(u8 x,u8 y,const char data[][16],u8 num)
{      			    
	u8 i;
	for(i=0;i<num;i++)
	OLED_ShowCHinese(x+i*16,y,data,i);
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{ 	
 unsigned int j=0;
 unsigned char x,y;
  
  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
	    {      
	    	OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
	    }
	}
} 

					    
void OLED_Init(void)
{ 	
 	 
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能A端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	  //初始化GPIOD3,6
 	GPIO_SetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13);	


	Delay_ms(100);
	OLED_WR_Byte(0xAE,OLED_CMD);//--display off
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  
	OLED_WR_Byte(0xB0,OLED_CMD);//--set page address
	OLED_WR_Byte(0x81,OLED_CMD); // contract control
	OLED_WR_Byte(0xFF,OLED_CMD);//--128   
	OLED_WR_Byte(0xA1,OLED_CMD);//set segment remap 
	OLED_WR_Byte(0xA6,OLED_CMD);//--normal / reverse
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3F,OLED_CMD);//--1/32 duty
	OLED_WR_Byte(0xC8,OLED_CMD);//Com scan direction
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset
	OLED_WR_Byte(0x00,OLED_CMD);//
	
	OLED_WR_Byte(0xD5,OLED_CMD);//set osc division
	OLED_WR_Byte(0x80,OLED_CMD);//
	
	OLED_WR_Byte(0xD8,OLED_CMD);//set area color mode off
	OLED_WR_Byte(0x05,OLED_CMD);//
	
	OLED_WR_Byte(0xD9,OLED_CMD);//Set Pre-Charge Period
	OLED_WR_Byte(0xF1,OLED_CMD);//
	
	OLED_WR_Byte(0xDA,OLED_CMD);//set com pin configuartion
	OLED_WR_Byte(0x12,OLED_CMD);//
	
	OLED_WR_Byte(0xDB,OLED_CMD);//set Vcomh
	OLED_WR_Byte(0x30,OLED_CMD);//
	
	OLED_WR_Byte(0x8D,OLED_CMD);//set charge pump enable
	OLED_WR_Byte(0x14,OLED_CMD);//
	
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
}  

void OLED_Init1(void)
{ 	
 	 
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能A端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	  //初始化GPIOD3,6
 	GPIO_SetBits(GPIOB,GPIO_Pin_14|GPIO_Pin_15);	


	Delay_ms(100);
	OLED_WR_Byte1(0xAE,OLED_CMD);//--display off
	OLED_WR_Byte1(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte1(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte1(0x40,OLED_CMD);//--set start line address  
	OLED_WR_Byte1(0xB0,OLED_CMD);//--set page address
	OLED_WR_Byte1(0x81,OLED_CMD); // contract control
	OLED_WR_Byte1(0xFF,OLED_CMD);//--128   
	OLED_WR_Byte1(0xA1,OLED_CMD);//set segment remap 
	OLED_WR_Byte1(0xA6,OLED_CMD);//--normal / reverse
	OLED_WR_Byte1(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte1(0x3F,OLED_CMD);//--1/32 duty
	OLED_WR_Byte1(0xC8,OLED_CMD);//Com scan direction
	OLED_WR_Byte1(0xD3,OLED_CMD);//-set display offset
	OLED_WR_Byte1(0x00,OLED_CMD);//
	
	OLED_WR_Byte1(0xD5,OLED_CMD);//set osc division
	OLED_WR_Byte1(0x80,OLED_CMD);//
	
	OLED_WR_Byte1(0xD8,OLED_CMD);//set area color mode off
	OLED_WR_Byte1(0x05,OLED_CMD);//
	
	OLED_WR_Byte1(0xD9,OLED_CMD);//Set Pre-Charge Period
	OLED_WR_Byte1(0xF1,OLED_CMD);//
	
	OLED_WR_Byte1(0xDA,OLED_CMD);//set com pin configuartion
	OLED_WR_Byte1(0x12,OLED_CMD);//
	
	OLED_WR_Byte1(0xDB,OLED_CMD);//set Vcomh
	OLED_WR_Byte1(0x30,OLED_CMD);//
	
	OLED_WR_Byte1(0x8D,OLED_CMD);//set charge pump enable
	OLED_WR_Byte1(0x14,OLED_CMD);//
	
	OLED_WR_Byte1(0xAF,OLED_CMD);//--turn on oled panel
}  
