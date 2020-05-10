#ifndef TIMER_H
#define TIMER_H

#include "main.h"
#include "adc.h"
#include <stdbool.h>

#define DUTY_CYCLE_MIN      10000   //ms
#define DUTY_CYCLE_MAX      20000   //ms

#define PWM_NUM_CHANNEL             16

#define SERVO_1_TIM                 TIM1
#define SERVO_1_PWM_PIN             GPIO_PIN_9
#define SERVO_1_PWM_PORT            GPIOE
#define SERVO_1_TIM_CHANNEL			TIM_CHANNEL_1

#define SERVO_2_TIM                 TIM1
#define SERVO_2_PWM_PIN             GPIO_PIN_11
#define SERVO_2_PWM_PORT            GPIOE
#define SERVO_2_TIM_CHANNEL			TIM_CHANNEL_2

#define SERVO_3_TIM                 TIM3
#define SERVO_3_PWM_PIN             GPIO_PIN_5
#define SERVO_3_PWM_PORT            GPIOB
#define SERVO_3_TIM_CHANNEL			TIM_CHANNEL_2

#define SERVO_4_TIM                 TIM3
#define SERVO_4_PWM_PIN             GPIO_PIN_4
#define SERVO_4_PWM_PORT            GPIOB
#define SERVO_4_TIM_CHANNEL			TIM_CHANNEL_1

#define SERVO_5_TIM                 TIM4
#define SERVO_5_PWM_PIN             GPIO_PIN_12
#define SERVO_5_PWM_PORT            GPIOD
#define SERVO_5_TIM_CHANNEL			TIM_CHANNEL_1

#define SERVO_6_TIM                 TIM4
#define SERVO_6_PWM_PIN             GPIO_PIN_13
#define SERVO_6_PWM_PORT            GPIOD
#define SERVO_6_TIM_CHANNEL			TIM_CHANNEL_2

#define SERVO_7_TIM                 TIM1
#define SERVO_7_PWM_PIN             GPIO_PIN_13
#define SERVO_7_PWM_PORT            GPIOE
#define SERVO_7_TIM_CHANNEL			TIM_CHANNEL_3

#define SERVO_8_TIM                 TIM1
#define SERVO_8_PWM_PIN             GPIO_PIN_14
#define SERVO_8_PWM_PORT            GPIOE
#define SERVO_8_TIM_CHANNEL			TIM_CHANNEL_4

#define LINEAR_1_TIM                TIM8
#define LINEAR_1_PWM_PIN            GPIO_PIN_6
#define LINEAR_1_PWM_PORT           GPIOC
#define LINEAR_1_TIM_CHANNEL        TIM_CHANNEL_1

#define LINEAR_2_TIM                TIM8
#define LINEAR_2_PWM_PIN            GPIO_PIN_7
#define LINEAR_2_PWM_PORT           GPIOC
#define LINEAR_2_TIM_CHANNEL        TIM_CHANNEL_2

#define LINEAR_3_TIM                TIM8
#define LINEAR_3_PWM_PIN            GPIO_PIN_8
#define LINEAR_3_PWM_PORT           GPIOC
#define LINEAR_3_TIM_CHANNEL        TIM_CHANNEL_3

#define LINEAR_4_TIM                TIM8
#define LINEAR_4_PWM_PIN            GPIO_PIN_9
#define LINEAR_4_PWM_PORT           GPIOC
#define LINEAR_4_TIM_CHANNEL        TIM_CHANNEL_4

#define LINEAR_5_TIM                TIM9
#define LINEAR_5_PWM_PIN            GPIO_PIN_5
#define LINEAR_5_PWM_PORT           GPIOE
#define LINEAR_5_TIM_CHANNEL        TIM_CHANNEL_1

#define LINEAR_6_TIM                TIM9
#define LINEAR_6_PWM_PIN            GPIO_PIN_6
#define LINEAR_6_PWM_PORT           GPIOE
#define LINEAR_6_TIM_CHANNEL        TIM_CHANNEL_2

#define LINEAR_7_TIM                TIM4
#define LINEAR_7_PWM_PIN            GPIO_PIN_8
#define LINEAR_7_PWM_PORT           GPIOB
#define LINEAR_7_TIM_CHANNEL        TIM_CHANNEL_3

#define LINEAR_8_TIM                TIM4
#define LINEAR_8_PWM_PIN            GPIO_PIN_9
#define LINEAR_8_PWM_PORT           GPIOB
#define LINEAR_8_TIM_CHANNEL        TIM_CHANNEL_4

static const r_gpio_module r_init_pwm[] = {
    {SERVO_1_PWM_PORT, SERVO_1_PWM_PIN,  SERVO_1_TIM_CHANNEL},
    {SERVO_2_PWM_PORT, SERVO_2_PWM_PIN,  SERVO_2_TIM_CHANNEL},
    {SERVO_3_PWM_PORT, SERVO_3_PWM_PIN,  SERVO_3_TIM_CHANNEL},
    {SERVO_4_PWM_PORT, SERVO_4_PWM_PIN,  SERVO_4_TIM_CHANNEL},
    {SERVO_5_PWM_PORT, SERVO_5_PWM_PIN,  SERVO_5_TIM_CHANNEL},
    {SERVO_6_PWM_PORT, SERVO_6_PWM_PIN,  SERVO_6_TIM_CHANNEL},
    {SERVO_7_PWM_PORT, SERVO_7_PWM_PIN,  SERVO_7_TIM_CHANNEL},
    {SERVO_8_PWM_PORT, SERVO_8_PWM_PIN,  SERVO_8_TIM_CHANNEL},
    {LINEAR_1_PWM_PORT, LINEAR_1_PWM_PIN,  LINEAR_1_TIM_CHANNEL},
    {LINEAR_2_PWM_PORT, LINEAR_2_PWM_PIN,  LINEAR_2_TIM_CHANNEL},
    {LINEAR_3_PWM_PORT, LINEAR_3_PWM_PIN,  LINEAR_3_TIM_CHANNEL},
    {LINEAR_4_PWM_PORT, LINEAR_4_PWM_PIN,  LINEAR_4_TIM_CHANNEL},
    {LINEAR_5_PWM_PORT, LINEAR_5_PWM_PIN,  LINEAR_5_TIM_CHANNEL},
    {LINEAR_6_PWM_PORT, LINEAR_6_PWM_PIN,  LINEAR_6_TIM_CHANNEL},
    {LINEAR_7_PWM_PORT, LINEAR_7_PWM_PIN,  LINEAR_7_TIM_CHANNEL},
    {LINEAR_8_PWM_PORT, LINEAR_8_PWM_PIN,  LINEAR_8_TIM_CHANNEL}
};

#define SERVO_1_PWR_PIN             GPIO_PIN_2
#define SERVO_1_PWR_PORT            GPIOE

#define SERVO_2_PWR_PIN             GPIO_PIN_3
#define SERVO_2_PWR_PORT            GPIOE

#define SERVO_3_PWR_PIN             GPIO_PIN_4
#define SERVO_3_PWR_PORT            GPIOE

#define SERVO_4_PWR_PIN             GPIO_PIN_13 // PC.13
#define SERVO_4_PWR_PORT            GPIOC

#define SERVO_5_PWR_PIN             GPIO_PIN_7 // PD.7
#define SERVO_5_PWR_PORT            GPIOD

#define SERVO_6_PWR_PIN             GPIO_PIN_6 // PD.6
#define SERVO_6_PWR_PORT            GPIOD

#define SERVO_7_PWR_PIN             GPIO_PIN_5 // PD.5
#define SERVO_7_PWR_PORT            GPIOD

#define SERVO_8_PWR_PIN             GPIO_PIN_4 // PD.4
#define SERVO_8_PWR_PORT            GPIOD

#define LINEAR_1_PWR_PIN             GPIO_PIN_1
#define LINEAR_1_PWR_PORT            GPIOE

#define LINEAR_2_PWR_PIN             GPIO_PIN_0
#define LINEAR_2_PWR_PORT            GPIOE

#define LINEAR_3_PWR_PIN             GPIO_PIN_9
#define LINEAR_3_PWR_PORT            GPIOD

#define LINEAR_4_PWR_PIN             GPIO_PIN_11
#define LINEAR_4_PWR_PORT            GPIOA

#define LINEAR_5_PWR_PIN             GPIO_PIN_8
#define LINEAR_5_PWR_PORT            GPIOA

#define LINEAR_6_PWR_PIN             GPIO_PIN_15
#define LINEAR_6_PWR_PORT            GPIOD

#define LINEAR_7_PWR_PIN             GPIO_PIN_11
#define LINEAR_7_PWR_PORT            GPIOD

#define LINEAR_8_PWR_PIN             GPIO_PIN_10
#define LINEAR_8_PWR_PORT            GPIOD

static const r_gpio r_init_pwR[] = {
    {SERVO_1_PWR_PORT, SERVO_1_PWR_PIN},
    {SERVO_2_PWR_PORT, SERVO_2_PWR_PIN},
    {SERVO_3_PWR_PORT, SERVO_3_PWR_PIN},
    {SERVO_4_PWR_PORT, SERVO_4_PWR_PIN},
    {SERVO_5_PWR_PORT, SERVO_5_PWR_PIN},
    {SERVO_6_PWR_PORT, SERVO_6_PWR_PIN},
    {SERVO_7_PWR_PORT, SERVO_7_PWR_PIN},
    {SERVO_8_PWR_PORT, SERVO_8_PWR_PIN},
    {LINEAR_1_PWR_PORT, LINEAR_1_PWR_PIN},
    {LINEAR_2_PWR_PORT, LINEAR_2_PWR_PIN},
    {LINEAR_3_PWR_PORT, LINEAR_3_PWR_PIN},
    {LINEAR_4_PWR_PORT, LINEAR_4_PWR_PIN},
    {LINEAR_5_PWR_PORT, LINEAR_5_PWR_PIN},
    {LINEAR_6_PWR_PORT, LINEAR_6_PWR_PIN},
    {LINEAR_7_PWR_PORT, LINEAR_7_PWR_PIN},
    {LINEAR_8_PWR_PORT, LINEAR_8_PWR_PIN}
};

enum{
    G1_SERVO_1 = 0,
    G1_SERVO_2,
	G2_SERVO_1,
    G2_SERVO_2,
	G3_SERVO_1,
    G3_SERVO_2,
	G4_SERVO_1,
	G4_SERVO_2,
    G1_LINEAR_SERVO_1,
    G1_LINEAR_SERVO_2,
    G2_LINEAR_SERVO_1,
    G2_LINEAR_SERVO_2,
    G3_LINEAR_SERVO_1,
    G3_LINEAR_SERVO_2,
	G4_LINEAR_SERVO_1,
	G4_LINEAR_SERVO_2
};

enum _MOTOR_TYPE_{
    SERVO = 0,
    SUB_LINEAR,
    MAIN_LINEAR
};

enum{
	TIMER_OFF = 0,
	TIMER_ON
};

enum _MOTOR_STATUS_{
    UNKNOW_ = 0,         // define when system has already start
    RUNNING,            // define for all motors
    CLAMP,              // define for all motors
    RELEASE,            // define for all motors
    OVER_CURRENT,        // define for all servo motors
    ERROR_SERVO,
    ERROR_LINEAR
};

enum _power_motor_{
    POWER_OFF = 0,
    POWER_ON
};

enum _motor_step_{
    UNKNOW_STEP = 0,
    STEP_1, // POWER ON
    STEP_2, // CTRL DUTY AND CHECK
    STEP_3,  // POWER OFF
};

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;

extern uint16_t release_duty_cycle[PWM_NUM_CHANNEL];
extern uint16_t clamp_duty_cycle[PWM_NUM_CHANNEL];

void MX_TIMER_Init(void);
void Init_GPIO_servo_pwr(void);
void on_off_pwm(uint8_t status, uint8_t servo_no);
void on_off_pwR(uint8_t servo_no, GPIO_PinState pin_state);
void init_servo_motors(void);
void ctrl_pwm_duty(uint8_t servo_no, uint16_t duty_cycle);
void control_servo_motor(void);
void check_allow_ctr_motor_sub_linear(void);
uint8_t check_is_motor_linear1(uint8_t index);
uint8_t check_is_motor_linear2(uint8_t index);

#endif // TIMER_H
