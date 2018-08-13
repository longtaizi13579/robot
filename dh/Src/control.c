#include "control.h"
#include "mpu9250.h"
#include "main.h"
#include "usart.h"
#include "math.h"
int max_x = 0;
int max_y = 0;
int min_x = 0;
int min_y = 0;
int mid_x = -13;
int mid_y = -13;

int16_t gy[3];   //三轴陀螺仪的值，分别为 X　Y　Z的角速度
int16_t ac[3];  //XYZ三轴的加速度
int16_t mag[3]; //三轴磁场

double KP = 7;
double KI = 0.01;//0.02;
double KD = 5;//10;

double PID = 0;

int32_t accu_angle = 0;
int32_t angle = 0;
int32_t last_angle = 0;
int32_t target = 0;
uint8_t PID_ENABLE = 0;

int leftspeed=0;//左轮速度
int rightspeed=0;//右轮速度
int leftspeedset=0;//左轮速度预值
int rightspeedset=0;//右轮速度预值
float leftspeedkp=100;//左轮速度p值
float leftspeedki=0;//左轮速度i值
float leftspeedkd=0;//左轮速度d值
int leftspeederroracc=0;//左轮速度累计误差
int leftspeederrorlast=0;//左轮上次误差

float rightspeedkp=0;//右轮速度p值
float rightspeedki=0;//右轮速度i值
float rightspeedkd=0;//右轮速度d值
int rightspeederroracc=0;//右轮速度累计误差
int rightspeederrorlast=0;//右轮上次误差
int speedenable=1;//速度环使能

void megnet()//磁力计获取角度
{
  MPU_Read6500(&MPU9250,ac,gy);
    if(MPU9250.MAG_OK){  //判断该芯片是否含有磁力计，如果是MPU6050，是不含磁力计的
      MPU_ReadM_Mag(&MPU9250,mag);
    max_x = max_x > mag[0] ? max_x : mag[0];
    min_x = min_x < mag[0] ? min_x : mag[0];
    max_y = max_y > mag[1] ? max_y : mag[1];
    min_y = min_y < mag[1] ? min_y : mag[1];
    mid_x = (max_x + min_x) / 2;
    mid_y = (max_y + min_y) / 2;
    
      mag[0] -= mid_x;
      mag[1] -= mid_y;
      angle = (int32_t)(atan2((double)mag[0], (double)mag[1]) * 180 / 3.1415926 - target);
}
}
void direction_control()//方向环PID
{
    
   if(PID_ENABLE == 0 || timecounter >= 0)
    return;
    accu_angle += angle;
    uprintf("angle=%d",angle);
    if(accu_angle > 7500)
      accu_angle = 7500;
    if(accu_angle < -7500)
      accu_angle = -7500;
    PID = accu_angle * KI + angle * KP + (angle-last_angle) * KD;
    int pwm1=pulseright-(int)PID/4;
    int pwm2=pulseleft+(int)PID/4;
    pwm_control(pwm1,pwm2);
    last_angle=angle;

}

void speed_control()//速度环
{
    leftspeed=TIM2->CNT;
    rightspeed=TIM4->CNT;
    if(rightspeed>30000)
    {
      rightspeed=rightspeed-65536;
    }
    if(leftspeed>30000)
    {
      leftspeed=leftspeed-65536;
    }
    rightspeed=rightspeed*(-1);
    //左轮pid
    int lefterror=leftspeedset-leftspeed;
    leftspeederroracc+=lefterror;
    
    int leftPIDcontrol=(int)(leftspeedkp*lefterror+leftspeedki*leftspeederroracc+leftspeedkd*(lefterror-leftspeederrorlast));
    leftspeederrorlast=lefterror;
    //右轮pid
    int righterror=rightspeedset-rightspeed;
    rightspeederroracc+=righterror;
    int rightPIDcontrol=(int)(rightspeedkp*righterror+rightspeedki*rightspeederroracc+rightspeedkd*(righterror-rightspeederrorlast));
    rightspeederrorlast=righterror;
    //if(speedenable)
    //{
      pwm_control(leftPIDcontrol,rightPIDcontrol);
      //uprintf("leftPIDcontrol=%d,rightPIDcontrol=%d",leftPIDcontrol,rightPIDcontrol);
    //}
    TIM2->CNT=0;
    TIM4->CNT=0;
}