/**
  ******************************************************************************
  * @file    Templates/Src/stm32f4xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.2.5
  * @date    04-November-2016
  * @brief   HAL MSP module.
  *
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
    This file is generated automatically by STM32CubeMX and eventually modified
    by the user

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "../Scr/main.h"
#include "../Scr/adc.h"
#include "../Scr/uart_debug.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"




/**
  * @brief  Initializes the Global MSP.
  * @param  None
  * @retval None
  */
//void HAL_MspInit(void)
//{
//    /* NOTE : This function is generated automatically by STM32CubeMX and eventually
//            modified by the user
//   */
//}

void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * @brief  DeInitializes the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspDeInit(void)
{
    /* NOTE : This function is generated automatically by STM32CubeMX and eventually
            modified by the user
   */
    uint8_t index;
    for(index = 0; index < 4; index++){
        HAL_GPIO_DeInit(sensor_check_phone[index].port, sensor_check_phone[index].pin);
    }
}

/**
  * @}
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if(huart->Instance==USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* Peripheral interrupt init */
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 2);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    if(huart->Instance==UART5){
        __HAL_RCC_UART5_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* Peripheral interrupt init */
        HAL_NVIC_SetPriority(UART5_IRQn, 0, 2);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if(huart->Instance==USART1){
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
        /* Peripheral interrupt DeInit*/
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
    if(huart->Instance==UART5){
        __HAL_RCC_UART5_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
        /* Peripheral interrupt DeInit*/
        HAL_NVIC_DisableIRQ(UART5_IRQn);
    }
}


// TIMER-PWM

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
  if(htim_pwm->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
  if(htim_pwm->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
  if(htim_pwm->Instance==TIM8)
  {
    __HAL_RCC_TIM8_CLK_ENABLE();
  }
  if(htim_pwm->Instance==TIM9)
  {
    __HAL_RCC_TIM9_CLK_ENABLE();
  }

}


void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();
  }
  if(htim_pwm->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
  if(htim_pwm->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_DISABLE();
  }
  if(htim_pwm->Instance==TIM8)
  {
    __HAL_RCC_TIM8_CLK_DISABLE();
  }
  if(htim_pwm->Instance==TIM9)
  {
    __HAL_RCC_TIM9_CLK_DISABLE();
  }

}


void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
	uint8_t index;
    if(hadc->Instance==ADC1)
    {
        __HAL_RCC_ADC1_CLK_DISABLE();
        for(index = 0; index < NUMBER_ADC_CHANNEL; index++){
            HAL_GPIO_DeInit(r_init_adc[index].port, r_init_adc[index].pin);
        }

        HAL_DMA_DeInit(hadc->DMA_Handle);
    }
}


void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C1)
  {
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_I2C1_CLK_ENABLE();
  }

}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if(hi2c->Instance==I2C1)
  {
    __HAL_RCC_I2C1_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  }

}

