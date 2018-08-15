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

float gyset[3];//���ٶ�Ԥ��ֵ

double KP = -0.21;
double KI = 0;//0.02;
double KD = 0.05;//10;

double PID = 0;

int32_t accu_angle = 0;
int angle = 0;
int32_t last_angle = 0;
int target = 0;
uint8_t PID_ENABLE = 0;

int leftspeed=0;//�����ٶ�
int rightspeed=0;//�����ٶ�
float leftspeedset=0;//�����ٶ�Ԥֵ
float rightspeedset=0;//�����ٶ�Ԥֵ
float leftspeedkp=100;//�����ٶ�pֵ
float leftspeedki=10;//�����ٶ�iֵ
float leftspeedkd=0;//�����ٶ�dֵ
float leftspeederroracc=0;//�����ٶ��ۼ����
float leftspeederrorlast=0;//�����ϴ����
int leftmaichong=0;//������
int rightmaichong=0;//������
float rightspeedkp=105;//�����ٶ�pֵ
float rightspeedki=10.5;//�����ٶ�iֵ
float rightspeedkd=0;//�����ٶ�dֵ
float rightspeederroracc=0;//�����ٶ��ۼ����
float rightspeederrorlast=0;//�����ϴ����
int speedenable=1;//�ٶȻ�ʹ��
int angleset=0;//�ٶȻ�Ԥ��ֵ
int gy_enable=1;
int direction_enable=0;//����ʹ��
float angle_speedPID=0;
float last_angle_speed=0;
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
float calangle=0;
int counter=0;
float gz3=0;
void gy_get()//���ٶȲɼ�
{
  //���ٶ�ת��
  gz3=((float)gy[2]/32768.0f)*500.0f;
}
float angle_speed=0;
float angle_speedacc=0;
float angle_error=0;
//���ٶȿ��ƣ�ȫ����λ
void gy_control()//���ٶȻ�pid
{
    angle_speed=((gz3-0.161791)*0.005)/163.18*360.0;
    angle_error=(-1)*(angleset-angle_speed);
    angleset-=angle_speed;
    uprintf("angleset=%f\n",angleset);
    if(!gy_enable)//�ر�ʹ�ܺ󣬿����ٶȿ���ֱ�ӵ���leftspeedset,rightspeedset
      return ;
    angle_speedPID = angle_error * KP + (angle_error-last_angle_speed) * KD;
    leftspeedset=(-1)*angle_speedPID;
    rightspeedset=angle_speedPID;
    last_angle_speed=angle_error;
}
void direction_control()//����PID
{
    if(!direction_enable)//�ر�ʹ�ܺ󣬿����ٶȿ���ֱ�ӵ���leftspeedset,rightspeedset
      return ;
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
    //send_wave((float)angle,(float)leftspeedset,(float)rightspeedset,(float)0);

}

void speed_control()//�ٶȻ�
{
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
    leftspeed=0;
    rightspeed=0;
    //uprintf("left=%d,right=%d",leftmaichong,rightmaichong);
}
int autostate=0;//�Զ�����״̬
int laststate=0;//��һ��״̬
void maichongnew()//�������壨1msһ�Σ�
{
      int getleft=TIM2->CNT;
      int getright=TIM4->CNT;
     if(getright>30000)
     {
        getright=getright-65536;
      }
      if(getleft>30000)
      {
        getleft=getleft-65536;
      }
      getright=getright*(-1);
      leftspeed+=getleft;
      rightspeed+=getright;
      leftmaichong+=getleft;
      rightmaichong+=getright;
      TIM2->CNT=0;
      TIM4->CNT=0;
}
void auto_control1()
{
  //state1=610;
  //state3=1110;

  if(rightmaichong>250&&autostate==1)
  {
    leftmaichong=0;
    rightmaichong=0;
    autostate=2;
  }
  else if(autostate==2 && leftmaichong>892)
  {
    leftmaichong=0;
    rightmaichong=0;
    autostate=3;
  
  }
  else if(autostate==3 && leftmaichong>860)
  {
    leftmaichong=0;
    rightmaichong=0;
    autostate=4; 
  }
  else if(autostate==4 && rightmaichong>192)
  {
    leftmaichong=0;
    rightmaichong=0;
    autostate=5; 
  }
 else if(autostate==6 && leftmaichong<-490)
 {
     leftmaichong=0;
    rightmaichong=0;
    autostate=7; 
 }
  else if(autostate==7 && rightmaichong<-1310)
 {
     leftmaichong=0;
    rightmaichong=0;
    autostate=8; 
 }
 else if(autostate==8 && rightmaichong>920)
 {
     leftmaichong=0;
    rightmaichong=0;
    autostate=9; 
 }
 else if(autostate==9 && leftmaichong<-170)
 {
     leftmaichong=0;
    rightmaichong=0;
    autostate=10; 
 }
  if(laststate!=autostate)
  {
    if(autostate==1)
    {
      direction_enable=0;
      rightspeedset=4.5;
      leftspeedset=12.285;
    }
    else if(autostate==2)
    {
      direction_enable=0;
      leftspeedset=8;
      rightspeedset=12.4;
    }
    else if(autostate==3)
    {
      leftspeedset=8;
      rightspeedset=12.292;
      direction_enable=0;
    }
    else if(autostate==4)
    {
      leftspeedset=12.9;
      rightspeedset=4;
      direction_enable=0;
    }
    else if(autostate==5)
    {
      direction_enable=0;
      leftspeedset=0;
      rightspeedset=0;
    
    }
    else if(autostate==6)
    {
      direction_enable=0;
      leftspeedset=-12.066;
      rightspeedset=-3.5;
     
    }
   else if(autostate==7)
    {
      direction_enable=0;
      leftspeedset=-9;
      rightspeedset=-12.11;
    
    }
    else if(autostate==8)
    {
      direction_enable=0;
      leftspeedset=0;
      rightspeedset=12.5;
    
    }
     else if(autostate==9)
    {
      direction_enable=0;
      leftspeedset=-11;
      rightspeedset=-11.95;
    
    }
    else if(autostate==10)
    {
      direction_enable=0;
      leftspeedset=0;
      rightspeedset=0;
    
    }
  }
  laststate=autostate;
}