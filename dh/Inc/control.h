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
extern int autostate;//自动控制状态
extern int leftmaichong;//脉冲数
extern int rightmaichong;//脉冲数
extern float leftspeedset;//左轮速度预值
extern float rightspeedset;//右轮速度预值
extern void auto_control1();
extern float leftspeederroracc;//左轮速度累计误差
extern float rightspeederroracc;//右轮速度累计误差
extern void maichongnew();
extern void gy_get();
extern float angle_speedacc;
extern void gy_control();
extern int counter;
extern float calangle;
#endif /*__ control_H */

