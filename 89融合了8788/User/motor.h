#ifndef __MOTOR_H
#define	__MOTOR_H
#include "stm32f10x.h"
void chumen(void);
void quzhaoerweima(void);

void fromtake_tohome(void);
void all_oled(void);
void order(void);//˳��


void red_123(void);//������ȡ��123
void downred_123(void);//�����ɫ���õ�123
void returnred_123(void);//�ص���ȡ��123

void green_123(void);//�����м�ȡ��123
void downgreen_123(void);//������ɫ���õ�123
void returngreen_123(void);//�ص��м�ȡ��

void blue_123(void);//�����м�ȡ��123
void downblue_123(void);//������ɫ���õ�123
void returnblue_123(void);//�ص���ɫȡ��


void code_mid(void);//�Ӷ�ά�뵽�м�
void mid_place(void);//���м䵽����
void place_mid(void);//�ӷ��õ��м�
void place_start(void);//�ӷ��ûس�ʼ�㡣
void mid_place_bian(void);//���м䵽����bian


void new_chumen(void);//������Ų���1
void new_qrcode(void);//����Թ�ȥ�Ҷ�ά��
void new_qrget(void);//�Ӷ�ά�뵽�м�ץȡ��
void new_placemid(void);//��ץȡ���м���õ�
void new_placeedge(void);//��ץȡ����Ե���õ�
void new_getmid(void);//�ӷ��õ㵽�м�ץȡ��
void gohome(void);
#define y_speed 260
#endif
/****/
