#ifndef __STEER_H_
#define __STEER_H_
#include <stm32f10x.h>
void Observation_center(void);//�۲��м�
void Observe_the_right_side(void);//�۲��ұ�
void Color_reset(void);//�۲츴λ

void Reading_Two_Dimensional_Code(void);//��ȡ��ά��
void Read_reset(void);//��ȡ��λ

void Grab_left(void);//ץȡ���
void Drop_left(void);//�������
void Left_reduction(void);//��߸�λ

void Grasp_Center(void);//ץȡ�м�
void Drop_middle(void);//�����м�
void Intermediate_reset(void);//�м临λ

void Unfetched_state(void);//׼��ץȡ״̬

#endif

