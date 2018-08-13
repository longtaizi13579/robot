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

int16_t gy[3];   //���������ǵ�ֵ���ֱ�Ϊ X��Y��Z�Ľ��ٶ�
int16_t ac[3];  //XYZ����ļ��ٶ�
int16_t mag[3]; //����ų�

double KP = -0.21;
double KI = 0;//0.02;
double KD = 0.05;//10;

double PID = 0;

int32_t accu_angle = 0;
int32_t angle = 0;
int32_t last_angle = 0;
int32_t target = 0;
uint8_t PID_ENABLE = 0;

int leftspeed=0;//�����ٶ�
int rightspeed=0;//�����ٶ�
float leftspeedset=0;//�����ٶ�Ԥֵ
float rightspeedset=0;//�����ٶ�Ԥֵ
float leftspeedkp=128;//�����ٶ�pֵ
float leftspeedki=0.35;//�����ٶ�iֵ
float leftspeedkd=-15;//�����ٶ�dֵ
float leftspeederroracc=0;//�����ٶ��ۼ����
float leftspeederrorlast=0;//�����ϴ����

float rightspeedkp=135;//�����ٶ�pֵ
float rightspeedki=0.42;//�����ٶ�iֵ
float rightspeedkd=-15;//�����ٶ�dֵ
float rightspeederroracc=0;//�����ٶ��ۼ����
float rightspeederrorlast=0;//�����ϴ����
int speedenable=1;//�ٶȻ�ʹ��
int angleset=0;

void megnet()//�����ƻ�ȡ�Ƕ�
{
  MPU_Read6500(&MPU9250,ac,gy);
    if(MPU9250.MAG_OK){  //�жϸ�оƬ�Ƿ��д����ƣ������MPU6050���ǲ��������Ƶ�
      MPU_ReadM_Mag(&MPU9250,mag);
    max_x = max_x > mag[0] ? max_x : mag[0];
    min_x = min_x < mag[0] ? min_x : mag[0];
    max_y = max_y > mag[1] ? max_y : mag[1];
    min_y = min_y < mag[1] ? min_y : mag[1];
    mid_x = (max_x + min_x) / 2;
    mid_y = (max_y + min_y) / 2;
    
      mag[0] -= mid_x;
      mag[1] -= mid_y;
      angle = (int32_t)(atan2((double)mag[0], (double)mag[1]) * 180 / 3.1415926 - target)+angleset;
      angle = angle > 180 ? angle - 360 : angle;
      angle = angle <= -180 ? angle + 360 : angle;
}
}
void direction_control()//����PID
{
    accu_angle += angle;
    //uprintf("angle=%d",angle);
    if(accu_angle > 10000)
      accu_angle = 10000;
    if(accu_angle < -10000)
      accu_angle = -10000;
    PID = accu_angle * KI + angle * KP + (angle-last_angle) * KD;
    leftspeedset=(float)initrightspeedset-PID;
    rightspeedset=(float)initrightspeedset+PID;
    //pwm_control(pwm1,pwm2);
    last_angle=angle;
    send_wave((float)angle,(float)leftspeedset,(float)rightspeedset,(float)0);

}

void speed_control()//�ٶȻ�
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
    //����pid
    float lefterror=leftspeedset-(float)leftspeed;
    leftspeederroracc+=lefterror;
    
    int leftPIDcontrol=(int)(leftspeedkp*lefterror+leftspeedki*leftspeederroracc+leftspeedkd*(lefterror-leftspeederrorlast));
    leftspeederrorlast=lefterror;
    //����pid
    float righterror=rightspeedset-(float)rightspeed;
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