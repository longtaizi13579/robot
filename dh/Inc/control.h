#ifndef __control_H
#define __control_H
#include "main.h"

extern void megnet();
extern void direction_control();
extern int rightspeed;
extern float rightspeedset;
extern int leftspeed;
extern float leftspeedset;
extern void speed_control();
extern int angleset;
extern int target;
extern int angle ;
extern int autostate;//�Զ�����״̬
extern int leftmaichong;//������
extern int rightmaichong;//������
extern float leftspeedset;//�����ٶ�Ԥֵ
extern float rightspeedset;//�����ٶ�Ԥֵ
extern void auto_control1();
extern float leftspeederroracc;//�����ٶ��ۼ����
extern float rightspeederroracc;//�����ٶ��ۼ����
extern void maichongnew();
extern void gy_get();
extern float angle_speedacc;
extern void gy_control();
extern int counter;
extern float calangle;
#endif /*__ control_H */

