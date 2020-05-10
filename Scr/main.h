/**
  ******************************************************************************
  * @file    LibJPEG/LibJPEG_Decoding/Inc/main.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014 
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include <stm32f4xx_hal.h>
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "sensor.h"
#include "board.h"
#include "timer.h"
#include "adc.h"
#include "uart_debug.h"
#include "protocol.h"
#include <stdbool.h>

#ifndef OS_NORMAL_STKSIZE
 #define OS_NORMAL_STKSIZE    512
#endif

#ifndef USBDEVICE_THREAD_STKSIZE
 #define USBDEVICE_THREAD_STKSIZE    1024
#endif

#define ID_UNIQUE_ADDRESS		0x1FFF7A10
#define ID_DBGMCU_IDCODE		0xE0042000
#define ID_FLASH_ADDRESS		0x1FFF7A22

#define system_getFlashSize()	(*(uint16_t *) (ID_FLASH_ADDRESS))
#define system_getRevision()		(*(uint16_t *) (DBGMCU->IDCODE + 2))
#define system_getSignature()	((*(uint16_t *) (ID_DBGMCU_IDCODE)) & 0x0FFF)
#define system_getUnique8(x)	((x >= 0 && x < 12) ? (*(uint8_t *) (ID_UNIQUE_ADDRESS + (x))) : 0)
#define system_getUnique16(x)	((x >= 0 && x < 6) ? (*(uint16_t *) (ID_UNIQUE_ADDRESS + 2 * (x))) : 0)
#define system_getUnique32(x)	((x >= 0 && x < 3) ? (*(uint32_t *) (ID_UNIQUE_ADDRESS + 4 * (x))) : 0)

#define TOUCH_ON            0
#define TOUCH_OFF           1

#define USB_IN              0x01
#define USB_OUT             0x81

#define USB_DEV             0

#define USB_REV             1
#define USB_NOREV           0

#define PORT_DEBUG_TX           GPIOE
#define PIN_DEBUG_TX            GPIO_PIN_8

#define PORT_DEBUG_RX           GPIOE
#define PIN_DEBUG_RX            GPIO_PIN_7

#define HW_CONFIG_1_PORT        GPIOC
#define HW_CONFIG_1_PIN         GPIO_PIN_11
#define HW_CONFIG_2_PORT        GPIOC
#define HW_CONFIG_2_PIN         GPIO_PIN_10
#define HW_CONFIG_3_PORT        GPIOA
#define HW_CONFIG_3_PIN         GPIO_PIN_15

#define PIN_HIGH    GPIO_PIN_SET
#define PIN_LOW     GPIO_PIN_RESET

#define NO_PHONE    0
#define HAVE_PHONE  1

#define SERVO_SPEED             15
#define LINEAR_SPEED_CLAMP      10
#define LINEAR_SPEED_RELEASE    17

#define STOP_AUTO_PROCESS  0
#define START_AUTO_PROCESS  1

#define STYLE_SMALL_PHONE   0
#define STYLE_BIG_PHONE     1

#define LINEAR_TIMEOUT 200
#define LINEAR_RUNNING_TIMEOUT  40

enum calib_gohome_ctrl_step{
    CALIB = 0,
    GO_HOME,
    CONTROL
};

extern uint32_t os_time;

extern uint8_t start_process, start_gohome, check_system_init;
extern UART_HandleTypeDef usart1_pc_handel, uart_debug;
extern uint8_t motor_step[16];
extern uint8_t check_calib_gohome_ctrl_step, start_auto_process[4], check_on_auto_process[4];
extern uint32_t time_point_1[16];
extern uint16_t duty_cycle_hold_clamp[16];

void update_motor_current(void);
void control_motor_Thread(void const *argument);
void process_ctrl_servo(void);
void system_resetMCU(void);
void get_hw_version(void);
uint32_t HAL_GetTick(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
