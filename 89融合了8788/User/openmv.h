#ifndef __OPENMV_H
#define	__OPENMV_H
#include "stm32f10x.h"
#include "led.h"
void Openmv_Init(void);
void OpenMv_00(void);//��ά��
void OpenMv_11(void);//��
void OpenMv_01(void);//Բ����ɫ
void OpenMv_10(void);//��ɫ
void getwk(int ch);
void placewk(int ch);
void nine_mode(int ch);
void huizong(void);
#endif
