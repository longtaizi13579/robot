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
extern float leftspeedset;//左轮速度预值
extern float rightspeedset;//右轮速度预值
#endif /*__ control_H */

