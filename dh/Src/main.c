/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "mpu9250.h"
#include "math.h"
#include "control.h"
int pulseleft;
int pulseright;

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void pwm_init(void);//电机测试代码（初始化）

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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


int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
  //MPU9250_Init(&MPU9250);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  pwm_init();
  //pwm_control(-1000,-1000);
  uprintf("helloworld\n");
    //uprintf2("helloworld\n");
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
    //uprintf("PID=%d\n",(int)PID);
  /* USER CODE BEGIN 3 */
  //uprintf("pulseleft=%d,pulseright=%d",pulseleft,pulseright);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void pwm_init(void)//电机测试代码（初始化）
{
  TIM3->CCR1=1000;
  TIM3->CCR2=1000;
  TIM3->CCR3=1000;
  TIM3->CCR4=1000;
  pulseleft=0;
  pulseright=0;

}
void pwm_control(int pulse1,int pulse2)//电机占空比变化代码
{
  if(pulse1>1000)
  {
    pulse1=1000;
    pulseleft=1000;
  }
  if(pulse2>1000)
  {
    pulse2=1000;
    pulseright=1000;
  }
   if(pulse1<-1000)
   {
     pulse1=-1000;
     pulseleft=-1000;
   }
  if(pulse2<-1000)
  {
    pulse2=-1000;
    pulseright=-1000;  
  }
  if(pulse1>=0&&pulse2>=0)
  {
    TIM3->CCR1=1000;
    TIM3->CCR2=1000-pulse1;
    TIM3->CCR3=1000;
    TIM3->CCR4=1000-pulse2;
  }
  else if(pulse1>=0&&pulse2<=0)
  {
    TIM3->CCR1=1000;
    TIM3->CCR2=1000-pulse1;
    TIM3->CCR3=1000+pulse2;
    TIM3->CCR4=1000;
  }
    else if(pulse1<=0&&pulse2>=0)
  {
    TIM3->CCR1=1000+pulse1;
    TIM3->CCR2=1000;
    TIM3->CCR3=1000;
    TIM3->CCR4=1000-pulse2;
  }
    else if(pulse1<=0&&pulse2<=0)
  {
    TIM3->CCR1=1000+pulse1;
    TIM3->CCR2=1000;
    TIM3->CCR3=1000+pulse2;
    TIM3->CCR4=1000;
  }
  if(pulse1==0)
  {
    TIM3->CCR1=1000;
    TIM3->CCR2=1000; 
  }
   if(pulse2==0)
  {
    TIM3->CCR3=1000;
    TIM3->CCR4=1000; 
  }
    //uprintf("pulse1=%d,pulse2=%d",pulse1,pulse2);

}
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void HAL_SYSTICK_Callback(){
  timecounter--;
  if(timecounter<150&&timecounter>=0)
  {
     pwm_control(0,0);
  }
  static int time_1ms_cnt = 0;
  static int time_50ms_cnt=0;
  time_1ms_cnt++;
  time_50ms_cnt++;
  if(time_1ms_cnt==5){
    time_1ms_cnt=0;
    //uprintf("\n\nleftspeedkd=%f",leftspeedkd);
    //uprintf("\n\nleftspeedkp=%f",leftspeedkp);
    //速度环
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
    //uprintf("rightspeederroracc=%d",rightspeederroracc);
    int rightPIDcontrol=(int)(rightspeedkp*righterror+rightspeedki*rightspeederroracc+rightspeedkd*(righterror-rightspeederrorlast));
    rightspeederrorlast=righterror;
    //if(speedenable)
    //{
      pwm_control(leftPIDcontrol,rightPIDcontrol);
      //uprintf("leftPIDcontrol=%d,rightPIDcontrol=%d",leftPIDcontrol,rightPIDcontrol);
    //}
    TIM2->CNT=0;
    TIM4->CNT=0;
    megnet();
    direction_control();
  }
  if(time_50ms_cnt==50)
  {
    send_wave((float)rightspeed,(float)rightspeedset,(float)leftspeed,(float)leftspeedset);
    time_50ms_cnt=0;
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  *
 @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
