#include "steer.h"
#include "new_struct.h"
#include "openmv.h"
#include "motor.h"
unsigned char Openmv_Mode=0;

//extern unsigned char Openmv_Mode;//openmvģʽ
extern unsigned short int qd_code;//��ά��
/*����������ά��
123
132
213
231
312
321
*/
extern unsigned char color_shunjian;//��ɫ˲��
// 1 2 3
extern unsigned char order_1;//���˳��
//1 2 3 4 5 6
extern unsigned char order_2;//ɫ��˳��
extern unsigned short int color_mode;//ɫ��˳��
/*����������ά��
123
132
213
231
312
321
*/
void Openmv_Init(void)
{
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*����LED��ص�GPIO����ʱ��*/

	  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOF, ENABLE);
		/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;	

		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*���ÿ⺯������ʼ��GPIO*/
		GPIO_Init(GPIOF, &GPIO_InitStructure);	


		/* �ر�����led��	*/
		PFout(4)=0;
		PFout(5)=0;
		PFout(6)=0;
		PFout(7)=0;
}
void OpenMv_00(void)//��ά��
{
		PFout(4)=0;
		PFout(5)=0;	
		PFout(6)=0;
		PFout(7)=0;	
		Openmv_Mode=0;
}
void OpenMv_11(void)//wu
{
		PFout(4)=1;
		PFout(5)=1;	
			PFout(6)=1;
		PFout(7)=2;
		Openmv_Mode=3;
}
void OpenMv_01(void)//Բ����ɫ
{
		PFout(4)=0;
		PFout(5)=1;	
		PFout(6)=0;
		PFout(7)=1;	
		Openmv_Mode=1;
}
void OpenMv_10(void)//��ɫ
{
		PFout(4)=1;
		PFout(5)=0;	
			PFout(6)=1;
		PFout(7)=0;
		Openmv_Mode=2;
}
void getwk(int ch)
{
	if(ch==1)
	{
		Grasp_right();//(ץȡ�ұ߷�������
		Unfetched_state();//׼��ץȡ״̬			
	}
	else if(ch==2)
	{
		Grasp_mid();//(ץȡ�м䣩
		Unfetched_state();//׼��ץȡ״̬		
	}
	else if(ch==3)
	{
		Grasp_left();//(ץȡ��߶�ά�뷽��
		Unfetched_state();//׼��ץȡ״̬			
	}
}
void placewk(int ch)
{
	if(ch==3)
	{
		Place_left();//������߶�ά�뷽��
		Unfetched_state();//׼��ץȡ״̬		
	}
	else if(ch==2)
	{
		Place_mid();//�����м�
		Unfetched_state();//׼��ץȡ״̬	
	}
	else if(ch==1)
	{
		Place_right();//�����ұ߷���������
		Unfetched_state();//׼��ץȡ״̬		
	}	
}
void nine_mode(int ch)
{
	switch(ch){
		case 1:{//1��1
			getwk(1);
			new_placeedge();
			placewk(1);
			new_getmid();
			break;
		}
		case 2:{//1��2
			getwk(1);
			new_placemid();
			placewk(2);
			new_getmid();			
			break;
		}
		case 3:{//1��3
			getwk(1);
			new_placeedge();
			placewk(3);
			new_getmid();
			break;
		}
		case 4:{//2��1
			getwk(2);
			new_placeedge();
			placewk(1);
			new_getmid();
			break;
		}
		case 5:{//2��2
			getwk(2);
			new_placemid();
			placewk(2);
			new_getmid();
			break;
		}
		case 6:{//2��3
			getwk(2);
			new_placeedge();
			placewk(3);
			new_getmid();
			break;
		}
		case 7:{//3��1
			getwk(3);
			new_placeedge();
			placewk(1);
			new_getmid();
			break;
		}
		case 8:{//3��2
			getwk(3);
			new_placemid();
			placewk(2);
			new_getmid();			
			break;
		}
		case 9:{//3��3
			getwk(3);
			new_placeedge();
			placewk(3);
			new_getmid();
			break;
		}		
	}
}
void huizong(void)
{
	if(qd_code==123&&order_1==1&&color_mode==123)///123 1
	{
		nine_mode(1);
		nine_mode(5);
		nine_mode(9);
	}
	else if(qd_code==123&&order_1==1&&color_mode==132)
	{
		nine_mode(1);
		nine_mode(6);
		nine_mode(8);		
	}		
	else if(qd_code==123&&order_1==1&&color_mode==213)
	{
		nine_mode(2);
		nine_mode(4);
		nine_mode(9);		
	}		
	else if(qd_code==123&&order_1==1&&color_mode==231)
	{
		nine_mode(3);
		nine_mode(4);
		nine_mode(8);			 
	}	
	else if(qd_code==123&&order_1==1&&color_mode==312)
	{
		nine_mode(2);
		nine_mode(6);
		nine_mode(7);				
	}		
	else if(qd_code==123&&order_1==1&&color_mode==321)
	{
		nine_mode(3);
		nine_mode(5);
		nine_mode(7);				
	}		
}

