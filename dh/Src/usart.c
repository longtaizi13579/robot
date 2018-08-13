/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"
#include "string.h"
#include "gpio.h"
#include "main.h"
/* USER CODE BEGIN 0 */
#include <stdarg.h>
#include "tim.h"
/* USER CODE END 0 */
char uart_buffer[100 + 1];
char buffer_rx_temp;
char buffer1_rx_temp;
int pulse;
extern int32_t angle;
extern int32_t target;
extern uint8_t PID_ENABLE;
UART_HandleTypeDef huart1;
int timecounter=200000;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&buffer_rx_temp,1);
 // uprintf("1.1ok\n");
}

UART_HandleTypeDef huart2;
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UART_Receive_IT(&huart2,(uint8_t *)&buffer1_rx_temp,1);
  //uprintf("2.1ok\n");
 // HAL_UART_Receive_IT(&huart1,(uint8_t *)&buffer_rx_temp,6);
  //uprintf("%s\r\n",buffer_rx_temp);
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
   // uprintf("1.2ok\n");
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */
  //uprintf("2.2ok\n");
  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART1 i0nterrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void uprintf(char *fmt, ...)
{
	int size;
	
	va_list arg_ptr;
	
	va_start(arg_ptr, fmt);  
	
	size=vsnprintf(uart_buffer, 100 + 1, fmt, arg_ptr);
	va_end(arg_ptr);
    HAL_UART_Transmit(&huart1,(uint8_t *)uart_buffer,size,1000);
}
void uprintf2(char *fmt, ...)
{
	int size;
	
	va_list arg_ptr;
	
	va_start(arg_ptr, fmt);  
	
	size=vsnprintf(uart_buffer, 100 + 1, fmt, arg_ptr);
	va_end(arg_ptr);
    HAL_UART_Transmit(&huart2,(uint8_t *)uart_buffer,size,1000);
}
int num=0 ;
/* USER CODE END 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  static uint16_t stick_1 = 0;
  static uint16_t stick_2 = 0;
  static uint8_t state = 0;
  if(huart->Instance==USART1){
    HAL_UART_Receive_IT(&huart1,(uint8_t *)&buffer_rx_temp,1);
    if(buffer_rx_temp=='a')
    {
      leftspeedkp=leftspeedkp+5;
      //uprintf("\n\nleftspeedkp=%lf",leftspeedkp);
    }
    else if(buffer_rx_temp=='b')
    {
      leftspeedkp=leftspeedkp-5;
      //uprintf("\n\nleftspeedkp=%lf",leftspeedkp);
    }
    else if(buffer_rx_temp=='c')
    {
      leftspeedkd=leftspeedkd+0.5;
      //uprintf("\n\nleftspeedkd=%lf",leftspeedkd);
    }
    else if(buffer_rx_temp=='d')
    {
      leftspeedkd=leftspeedkd-0.5;
      //uprintf("\n\nleftspeedkd=%lf",leftspeedkd);
    }
    else if(buffer_rx_temp=='e')
    {
      rightspeedset=rightspeedset+60;
      leftspeedset=leftspeedset+60;
    }
    else if(buffer_rx_temp=='f')
    {
      rightspeedset=rightspeedset-60;
      leftspeedset=leftspeedset-60;
    }
   else if(buffer_rx_temp=='g')
    {
      rightspeedset=0;
      leftspeedset=7;
    }
    else if(buffer_rx_temp=='h')
    {
      rightspeedset=0;
      leftspeedset=0;
    }
    else if(buffer_rx_temp=='i')
    {
      uprintf("\n\nleftspeedkp=%lf",leftspeedkp);
    }
    else if(buffer_rx_temp=='j')
    {
      uprintf("\n\nleftspeedkd=%lf",leftspeedkd);
    }
  }
  else if(huart->Instance==USART2){
    HAL_UART_Receive_IT(&huart2,(uint8_t *)&buffer1_rx_temp,1);
    if(buffer1_rx_temp=='W' || buffer1_rx_temp=='Q' || buffer1_rx_temp=='P' || buffer1_rx_temp=='S')
    {
      state = buffer1_rx_temp;
    }
    else if(buffer1_rx_temp<='9' && buffer1_rx_temp>='0')
    {
      if(state=='W' || state=='Q')
      {
        stick_1 *= 10;
        stick_1 += buffer1_rx_temp - '0';
      }
      if(state=='P' || state=='S')
      {
        stick_2 *= 10;
        stick_2 += buffer1_rx_temp - '0';
      }
    }
    else if(buffer1_rx_temp=='\r')
    {
      if(state=='P')
      {
        int front_back = stick_1 - 128;
        int left_right = stick_2 - 128;
        if(front_back > 5)
          left_right = -left_right;
        pulseleft=6*((int)left_right-front_back);
        pulseright=6*(-(int)left_right-front_back);
        pwm_control(pulseleft,pulseright);
      }
      else if(state=='S')
      {
      }
      stick_1 = stick_2 = 0;
      timecounter=300;
    }
    else if(buffer1_rx_temp=='A'){
      pulseleft=1000;
      pulseright=1000;
      pwm_control(pulseleft,pulseright);
      timecounter=300;
    }
    else if(buffer1_rx_temp=='B'){
      pulseleft=-1000;
      pulseright=-1000;
       pwm_control(pulseleft,pulseright);
       timecounter=300;
    }
    else if(buffer1_rx_temp=='C'){
      if(pulseleft>=0)
      {
      pulseleft=500;
      pulseright=1000;
      }
      else
      {
      pulseleft=-500;
      pulseright=-1000;
      }
      pwm_control(pulseleft,pulseright);
      timecounter=300;
    }
    else if(buffer1_rx_temp=='D'){
      if(pulseleft>=0)
      {
      pulseleft=1000;
      pulseright=500;
      }
      else
      {
      pulseleft=-1000;
      pulseright=-500;
      }
      pwm_control(pulseleft,pulseright);
      timecounter=300;
    }
    else if(buffer1_rx_temp=='I'){
      num=23;
      changestate(num);
      uprintf("%d\r\n",TIM4->CCR3);
      uprintf("hello world\r\n");
    }
    else if(buffer1_rx_temp=='J'){
      num=43;
      changestate(num);
      uprintf("%d\r\n",TIM4->CCR3);
      uprintf("hello world\r\n");
    }
    else if(buffer1_rx_temp=='K'){//右占空比增加
       pulseleft=pulseleft-50;
      pulseright=pulseright+50;
      pwm_control(pulseleft,pulseright);
    }
    else if(buffer1_rx_temp=='L'){//左占空比增加
      pulseleft=pulseleft+50;
      pulseright=pulseright-50;
      pwm_control(pulseleft,pulseright);
    }
    else if(buffer1_rx_temp=='E'){//两轮等速
      pulseleft=(pulseleft+pulseright)/2;
      pulseright=pulseleft;
      if(pulseright>0)
        pulseleft=pulseleft+5;
       if(pulseright<0)
        pulseleft=pulseleft-5;
      pwm_control(pulseleft,pulseright);
    }
    else if(buffer1_rx_temp=='F'){//停止
      pulseleft=0;
      pulseright=0;
      pwm_control(pulseleft,pulseright);
    }
    else if(buffer1_rx_temp=='G'){//调整目标角度
      target = angle;
    }
    else if(buffer1_rx_temp=='H'){//启动、关闭PID
      PID_ENABLE = PID_ENABLE > 0 ? 0 : 1;
    }
    
  }
}
char s[22]={'b','y',16,6};
void send_wave(float arg1,float arg2,float arg3,float arg4){

  s[2]=16;  //length
  s[3]=6;   //type
  s[20]='\r';
  s[21]='\n';
  memcpy(s+4,&arg1,sizeof(arg1));
  memcpy(s+8,&arg2,sizeof(arg1));
  memcpy(s+12,&arg3,sizeof(arg1));
  memcpy(s+16,&arg4,sizeof(arg1));
  HAL_UART_Transmit(&huart1,(uint8_t *)s, 22,1000);

}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
