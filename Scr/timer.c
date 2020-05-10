#include "main.h"
#include "protocol.h"
#include "uart_debug.h"
#include "stm32f4xx_hal.h"
#include "timer.h"
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX


TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

uint16_t release_duty_cycle[PWM_NUM_CHANNEL];
uint16_t clamp_duty_cycle[PWM_NUM_CHANNEL];


/**
  * @brief  This function init timer handle to create pwm pulse at gpio
  * @param  none
  * @return None
  */
void MX_TIMER_Init(void)
{
    uint32_t f_timer = 1000000;
    uint16_t f_pwm = 333;
    uint8_t division_clock_timer_APB1 = 2; // depeding on the timer clock source is APB1 (APB1 is 2 and APB2 is 1)

    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    GPIO_InitTypeDef GPIO_InitStruct;

    // TIM1
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = (uint32_t)(SystemCoreClock / f_timer) - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = f_timer / f_pwm;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim1);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = f_timer / f_pwm / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, r_init_pwm[G1_SERVO_1].channel);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, r_init_pwm[G1_SERVO_2].channel);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, r_init_pwm[G4_SERVO_1].channel);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, r_init_pwm[G4_SERVO_2].channel);


    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    GPIO_InitStruct.Pin = r_init_pwm[G1_SERVO_1].pin;
    HAL_GPIO_Init(r_init_pwm[G1_SERVO_1].port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = r_init_pwm[G1_SERVO_2].pin;
    HAL_GPIO_Init(r_init_pwm[G1_SERVO_2].port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = r_init_pwm[G4_SERVO_1].pin;
    HAL_GPIO_Init(r_init_pwm[G4_SERVO_1].port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = r_init_pwm[G4_SERVO_2].pin;
    HAL_GPIO_Init(r_init_pwm[G4_SERVO_2].port, &GPIO_InitStruct);

    // TIM3
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = (uint32_t)(SystemCoreClock / f_timer / division_clock_timer_APB1 - 1);
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = f_timer / f_pwm;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = f_timer / f_pwm / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, r_init_pwm[G2_SERVO_1].channel);
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, r_init_pwm[G2_SERVO_2].channel);

    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    GPIO_InitStruct.Pin = r_init_pwm[G2_SERVO_1].pin;
    HAL_GPIO_Init(r_init_pwm[G2_SERVO_1].port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = r_init_pwm[G2_SERVO_2].pin;
    HAL_GPIO_Init(r_init_pwm[G2_SERVO_2].port, &GPIO_InitStruct);

    // TIM4
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = (uint32_t)(SystemCoreClock / f_timer / division_clock_timer_APB1 - 1);
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = f_timer / f_pwm;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim4);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = f_timer / f_pwm / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, r_init_pwm[G3_SERVO_1].channel);
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, r_init_pwm[G3_SERVO_2].channel);
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, r_init_pwm[G4_LINEAR_SERVO_1].channel);
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, r_init_pwm[G4_LINEAR_SERVO_2].channel);

    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    GPIO_InitStruct.Pin = r_init_pwm[G3_SERVO_1].pin;
    HAL_GPIO_Init(r_init_pwm[G3_SERVO_1].port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = r_init_pwm[G3_SERVO_2].pin;
    HAL_GPIO_Init(r_init_pwm[G3_SERVO_2].port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = r_init_pwm[G4_LINEAR_SERVO_1].pin;
    HAL_GPIO_Init(r_init_pwm[G4_LINEAR_SERVO_1].port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = r_init_pwm[G4_LINEAR_SERVO_2].pin;
    HAL_GPIO_Init(r_init_pwm[G4_LINEAR_SERVO_2].port, &GPIO_InitStruct);

    // TIM8
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = (uint32_t)(SystemCoreClock / f_timer - 1);
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = f_timer / f_pwm;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim8);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = f_timer / f_pwm / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, r_init_pwm[G1_LINEAR_SERVO_1].channel);
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, r_init_pwm[G1_LINEAR_SERVO_2].channel);
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, r_init_pwm[G2_LINEAR_SERVO_1].channel);
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, r_init_pwm[G2_LINEAR_SERVO_2].channel);

    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    GPIO_InitStruct.Pin = r_init_pwm[G1_LINEAR_SERVO_1].pin;
    HAL_GPIO_Init(r_init_pwm[G1_LINEAR_SERVO_1].port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = r_init_pwm[G1_LINEAR_SERVO_2].pin;
    HAL_GPIO_Init(r_init_pwm[G1_LINEAR_SERVO_2].port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = r_init_pwm[G2_LINEAR_SERVO_1].pin;
    HAL_GPIO_Init(r_init_pwm[G2_LINEAR_SERVO_1].port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = r_init_pwm[G2_LINEAR_SERVO_2].pin;
    HAL_GPIO_Init(r_init_pwm[G2_LINEAR_SERVO_2].port, &GPIO_InitStruct);

    // TIM9
    htim9.Instance = TIM9;
    htim9.Init.Prescaler = (uint32_t)(SystemCoreClock / f_timer - 1);
    htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim9.Init.Period = f_timer / f_pwm;
    htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim9);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim9, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = f_timer / f_pwm / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, r_init_pwm[G3_LINEAR_SERVO_1].channel);
    HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, r_init_pwm[G3_LINEAR_SERVO_2].channel);

    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
    GPIO_InitStruct.Pin = r_init_pwm[G3_LINEAR_SERVO_1].pin;
    HAL_GPIO_Init(r_init_pwm[G3_LINEAR_SERVO_1].port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = r_init_pwm[G3_LINEAR_SERVO_2].pin;
    HAL_GPIO_Init(r_init_pwm[G3_LINEAR_SERVO_2].port, &GPIO_InitStruct);

}

/**
  * @brief  This function init adc to read ADC value at gpio
  * @param  none
  * @return None
  */
void Init_GPIO_servo_pwr(void){
    uint8_t index;
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    for(index = 0 ; index < PWM_NUM_CHANNEL; index++){
        GPIO_InitStruct.Pin = r_init_pwR[index].pin;
        HAL_GPIO_Init(r_init_pwR[index].port, &GPIO_InitStruct);
    }
}


void on_off_pwm(uint8_t status, uint8_t servo_no){
    switch (servo_no) {
    case G1_SERVO_1:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim1, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim1, r_init_pwm[servo_no].channel);
        break;
    case G1_SERVO_2:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim1, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim1, r_init_pwm[servo_no].channel);
        break;
    case G2_SERVO_1:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim3, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim3, r_init_pwm[servo_no].channel);
        break;
    case G2_SERVO_2:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim3, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim3, r_init_pwm[servo_no].channel);
        break;
    case G3_SERVO_1:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim4, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim4, r_init_pwm[servo_no].channel);
        break;
    case G3_SERVO_2:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim4, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim4, r_init_pwm[servo_no].channel);
        break;
    case G4_SERVO_1:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim1, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim1, r_init_pwm[servo_no].channel);
        break;
    case G4_SERVO_2:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim1, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim1, r_init_pwm[servo_no].channel);
        break;
    case G1_LINEAR_SERVO_1:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim8, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim8, r_init_pwm[servo_no].channel);
        break;
    case G1_LINEAR_SERVO_2:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim8, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim8, r_init_pwm[servo_no].channel);
        break;
    case G2_LINEAR_SERVO_1:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim8, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim8, r_init_pwm[servo_no].channel);
        break;
    case G2_LINEAR_SERVO_2:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim8, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim8, r_init_pwm[servo_no].channel);
        break;
    case G3_LINEAR_SERVO_1:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim9, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim9, r_init_pwm[servo_no].channel);
        break;
    case G3_LINEAR_SERVO_2:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim9, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim9, r_init_pwm[servo_no].channel);
        break;
    case G4_LINEAR_SERVO_1:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim4, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim4, r_init_pwm[servo_no].channel);
        break;
    case G4_LINEAR_SERVO_2:
        if(status == TIMER_OFF) 	HAL_TIM_PWM_Stop(&htim4, r_init_pwm[servo_no].channel);
        else 						HAL_TIM_PWM_Start(&htim4, r_init_pwm[servo_no].channel);
        break;
    default:
        break;
    }
}

void on_off_pwR(uint8_t servo_no, GPIO_PinState pin_state){
    HAL_GPIO_WritePin(r_init_pwR[servo_no].port, r_init_pwR[servo_no].pin, pin_state);
    if(pin_state == GPIO_PIN_SET)
        motor_infor[servo_no].pwr = POWER_ON;
    else
        motor_infor[servo_no].pwr = POWER_OFF;
}

void init_servo_motors(void){
    uint8_t gripper = 0, index = 0;
    uint8_t have_phone_[4] = {0,0,0,0};

    send_debugging_data(" -->init_servo_motors\n\r");

    Is_phone_in_grippers();

    /*
     * truong hop co phone => dang kep phone
     * servo1.2 dang o vi tri kep phone => ko can power on, chi can dieu chinh xung cho no ve vi tri kep phone, bat timer
     * linear 1 dang o vi tri kep phone => cap xung clamp -> delay 200ms -> pwr on -> delay 200ms -> power off
     * linear 1 dang o vi tri kep phone => cap xung clamp -> delay 200ms -> pwr on
     */

    for(gripper = 0; gripper < 4; gripper++){
        if(gripper == GRIPPER_1){
            if(Grippers_infor.gripper_No1 == HAVE_PHONE){
                have_phone_[gripper] = HAVE_PHONE;
            }
            else{
                have_phone_[gripper] = NO_PHONE;
            }
        }
        else if(gripper == GRIPPER_2){
            if(Grippers_infor.gripper_No2 == HAVE_PHONE){
                have_phone_[gripper] = HAVE_PHONE;
            }
            else{
                have_phone_[gripper] = NO_PHONE;
            }
        }
        else if(gripper == GRIPPER_3){
            if(Grippers_infor.gripper_No3 == HAVE_PHONE){
                have_phone_[gripper] = HAVE_PHONE;
            }
            else{
                have_phone_[gripper] = NO_PHONE;
            }
        }
        else{// if(gripper == GRIPPER_4){
            if(Grippers_infor.gripper_No4 == HAVE_PHONE){
                have_phone_[gripper] = HAVE_PHONE;
            }
            else{
                have_phone_[gripper] = NO_PHONE;
            }
        }
    }
    for(gripper = 0; gripper < 4; gripper++){
        if(have_phone_[gripper] == HAVE_PHONE){
            // release motor 0, 2, 4, 6
            index = get_servo1_index_from_gripper_index(gripper);
            motor_infor[index].index = index;
            pl_control_motor[index].motor_index = index;
            motor_infor[index].status = CLAMP;
            motor_infor[index].duty = clamp_duty_cycle[index];
            pl_control_motor[index].duty = clamp_duty_cycle[index];
            // release motor 1, 3, 5, 7
            motor_infor[index+1].index = index + 1;
            pl_control_motor[index+1].motor_index = index+1;
            motor_infor[index+1].status = CLAMP;
            motor_infor[index+1].duty = clamp_duty_cycle[index+1];;
            pl_control_motor[index+1].duty = clamp_duty_cycle[index+1];

            ctrl_pwm_duty(index, motor_infor[index].duty);
            on_off_pwm(TIMER_ON, index);
            on_off_pwR(index, PIN_HIGH);
            ctrl_pwm_duty(index + 1, motor_infor[index+1].duty);
            on_off_pwm(TIMER_ON, index+1);
            on_off_pwR(index+1, PIN_HIGH);
        }
    }
    if((have_phone_[GRIPPER_1] == HAVE_PHONE)||(have_phone_[GRIPPER_2] == HAVE_PHONE)||(have_phone_[GRIPPER_3] == HAVE_PHONE)||(have_phone_[GRIPPER_4] == HAVE_PHONE)){
        osDelay(1000);

        for(gripper = 0; gripper < 4; gripper++){
            if(have_phone_[gripper] == HAVE_PHONE){
                // release motor 0, 2, 4, 6
                index = get_servo1_index_from_gripper_index(gripper);
                on_off_pwR(index, PIN_LOW);
                on_off_pwR(index+1, PIN_LOW);
            }
        }
    }
    for(gripper = 0; gripper < 4; gripper++){
        if(have_phone_[gripper] == HAVE_PHONE){
            // clamp motor 8, 10, 12, 14: linear 1
            index = get_servo1_index_from_gripper_index(gripper);
            motor_infor[index+8].index = index+8;
            pl_control_motor[index+8].motor_index = index+8;
            motor_infor[index+8].status = CLAMP;
            motor_infor[index+8].duty = clamp_duty_cycle[index+8];
            pl_control_motor[index+8].duty = clamp_duty_cycle[index+8];
            ctrl_pwm_duty(index+8, motor_infor[index+8].duty);
            on_off_pwm(TIMER_ON, index+8);
            on_off_pwR(index+8, PIN_HIGH);

            // clamp motor 9,11,13,15: linear 2
            motor_infor[index+9].index = index+9;
            pl_control_motor[index+9].motor_index = index+9;
            motor_infor[index+9].status = CLAMP;
            motor_infor[index+9].duty = clamp_duty_cycle[index+9];
            pl_control_motor[index+9].duty = clamp_duty_cycle[index+9];
            ctrl_pwm_duty(index+9, motor_infor[index+9].duty);
            on_off_pwm(TIMER_ON, index+9);
            on_off_pwR(index+9, PIN_HIGH);
        }
    }

    if((have_phone_[GRIPPER_1] == HAVE_PHONE)||(have_phone_[GRIPPER_2] == HAVE_PHONE)||(have_phone_[GRIPPER_3] == HAVE_PHONE)||(have_phone_[GRIPPER_4] == HAVE_PHONE)){
        osDelay(1500);

        for(gripper = 0; gripper < 4; gripper++){
            if(have_phone_[gripper] == HAVE_PHONE){
                // release motor 0, 2, 4, 6
                index = get_servo1_index_from_gripper_index(gripper);
                ctrl_pwm_duty(index+8,  duty_cycle_hold_clamp[index+8]);
                ctrl_pwm_duty(index+9, duty_cycle_hold_clamp[index+9]);

                // update gripper process
                process_of_gripper[gripper].gripper = gripper;
                process_of_gripper[gripper].step = PROCESS_CLAMP_SUB;
            }
        }
    }


    for(gripper = 0; gripper < 4; gripper++){
        if(have_phone_[gripper] == NO_PHONE){
            // clamp motor 8, 10, 12, 14: linear 1
            index = get_servo1_index_from_gripper_index(gripper);
            motor_infor[index+8].index = index+8;
            pl_control_motor[index+8].motor_index = index+8;
            motor_infor[index+8].status = CLAMP;
            motor_infor[index+8].duty = release_duty_cycle[index+8];
            pl_control_motor[index+8].duty = release_duty_cycle[index+8];
            ctrl_pwm_duty(index+8, motor_infor[index+8].duty);
            on_off_pwm(TIMER_ON, index+8);
            on_off_pwR(index+8, PIN_HIGH);

            // clamp motor 9,11,13,15: linear 2
            motor_infor[index+9].index = index+9;
            pl_control_motor[index+9].motor_index = index+9;
            motor_infor[index+9].status = CLAMP;
            motor_infor[index+9].duty = release_duty_cycle[index+9];
            pl_control_motor[index+9].duty = release_duty_cycle[index+9];
            ctrl_pwm_duty(index+9, motor_infor[index+9].duty);
            on_off_pwm(TIMER_ON, index+9);
            on_off_pwR(index+9, PIN_HIGH);
        }
    }

    if((have_phone_[GRIPPER_1] == NO_PHONE)||(have_phone_[GRIPPER_2] == NO_PHONE)||(have_phone_[GRIPPER_3] == NO_PHONE)||(have_phone_[GRIPPER_4] == NO_PHONE)){
        osDelay(1500);
    }

    for(gripper = 0; gripper < 4; gripper++){
        if(have_phone_[gripper] == NO_PHONE){
            // release motor 0, 2, 4, 6
            index = get_servo1_index_from_gripper_index(gripper);
            motor_infor[index].index = index;
            pl_control_motor[index].motor_index = index;
            motor_infor[index].status = CLAMP;
            motor_infor[index].duty = release_duty_cycle[index];
            pl_control_motor[index].duty = release_duty_cycle[index];
            // release motor 1, 3, 5, 7
            motor_infor[index+1].index = index + 1;
            pl_control_motor[index+1].motor_index = index+1;
            motor_infor[index+1].status = CLAMP;
            motor_infor[index+1].duty = release_duty_cycle[index+1];;
            pl_control_motor[index+1].duty = release_duty_cycle[index+1];

            ctrl_pwm_duty(index, motor_infor[index].duty);
            on_off_pwm(TIMER_ON, index);
            on_off_pwR(index, PIN_HIGH);
            ctrl_pwm_duty(index + 1, motor_infor[index+1].duty);
            on_off_pwm(TIMER_ON, index+1);
            on_off_pwR(index+1, PIN_HIGH);
        }
    }
    if((have_phone_[GRIPPER_1] == NO_PHONE)||(have_phone_[GRIPPER_2] == NO_PHONE)||(have_phone_[GRIPPER_3] == NO_PHONE)||(have_phone_[GRIPPER_4] == NO_PHONE)){
        osDelay(1000);

        for(gripper = 0; gripper < 4; gripper++){
            if(have_phone_[gripper] == NO_PHONE){
                index = get_servo1_index_from_gripper_index(gripper);
                // release motor 0, 2, 4, 6
                on_off_pwR(index, PIN_LOW);
                on_off_pwR(index+1, PIN_LOW);


                // update gripper process
                process_of_gripper[gripper].gripper = gripper;
                process_of_gripper[gripper].step = PROCESS_RELEASE_MAIN;
            }
        }
    }
}

/**
  * @brief  This function transmit angle to one servo which is one of 16 servos
  * @param  servo_no: is the servo will be controlled
  *         duty_cycle: servo's angle
  * @return None
  */

//bool ctrl_pwm_duty(uint8_t servo_no, uint16_t duty_cycle){
//    bool re = false;
//    switch (servo_no) {
//    case G1_SERVO_1:
//        if(htim1.Instance->CCR1 == duty_cycle){
//            send_debugging_data("G1_SERVO_1 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim1.Instance->CCR1 = duty_cycle;
//            re = true;
//        }
//        break;
//    case G1_SERVO_2:
//        if(htim1.Instance->CCR2 == duty_cycle){
//            send_debugging_data("G1_SERVO_2 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim1.Instance->CCR2 = duty_cycle;
//            re = true;
//        }
////        htim1.Instance->CCR2 = duty_cycle;
//        break;
//    case G2_SERVO_1:
//        if(htim3.Instance->CCR2 == duty_cycle){
//            send_debugging_data("G2_SERVO_1 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim3.Instance->CCR2 = duty_cycle;
//            re = true;
//        }
////        htim3.Instance->CCR2 = duty_cycle;
//        break;
//    case G2_SERVO_2:
//        if(htim3.Instance->CCR1 == duty_cycle){
//            send_debugging_data("G2_SERVO_2 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim3.Instance->CCR1 = duty_cycle;
//            re = true;
//        }
////        htim3.Instance->CCR1 = duty_cycle;
//        break;
//    case G3_SERVO_1:
//        if(htim4.Instance->CCR1 == duty_cycle){
//            send_debugging_data("G3_SERVO_1 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim4.Instance->CCR1 = duty_cycle;
//            re = true;
//        }
////        htim4.Instance->CCR1 = duty_cycle;
//        break;
//    case G3_SERVO_2:
//        if(htim4.Instance->CCR2 == duty_cycle){
//            send_debugging_data("G3_SERVO_2 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim4.Instance->CCR2 = duty_cycle;
//            re = true;
//        }
////        htim4.Instance->CCR2 = duty_cycle;
//        break;
//    case G4_SERVO_1:
//        if(htim1.Instance->CCR3 == duty_cycle){
//            send_debugging_data("G4_SERVO_1 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim1.Instance->CCR3 = duty_cycle;
//            re = true;
//        }
////        htim1.Instance->CCR3 = duty_cycle;
//        break;
//    case G4_SERVO_2:
//        if(htim1.Instance->CCR4 == duty_cycle){
//            send_debugging_data("G4_SERVO_2 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim1.Instance->CCR4 = duty_cycle;
//            re = true;
//        }
////        htim1.Instance->CCR4 = duty_cycle;
//        break;
//    case G1_LINEAR_SERVO_1:
//        if(htim8.Instance->CCR1 == duty_cycle){
//            send_debugging_data("G1_LINEAR_SERVO_1 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim8.Instance->CCR1 = duty_cycle;
//            re = true;
//        }
////        htim8.Instance->CCR1 = duty_cycle;
//        break;
//    case G1_LINEAR_SERVO_2:
//        if(htim8.Instance->CCR2 == duty_cycle){
//            send_debugging_data("G1_LINEAR_SERVO_2 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim8.Instance->CCR2 = duty_cycle;
//            re = true;
//        }
////        htim8.Instance->CCR2 = duty_cycle;
//        break;
//    case G2_LINEAR_SERVO_1:
//        if(htim8.Instance->CCR3 == duty_cycle){
//            send_debugging_data("G2_LINEAR_SERVO_1 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim8.Instance->CCR3 = duty_cycle;
//            re = true;
//        }
////        htim8.Instance->CCR3 = duty_cycle;
//        break;
//    case G2_LINEAR_SERVO_2:
//        if(htim8.Instance->CCR4 == duty_cycle){
//            send_debugging_data("G2_LINEAR_SERVO_2 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim8.Instance->CCR4 = duty_cycle;
//            re = true;
//        }
////        htim8.Instance->CCR4 = duty_cycle;
//        break;
//    case G3_LINEAR_SERVO_1:
//        if(htim9.Instance->CCR1 == duty_cycle){
//            send_debugging_data("G3_LINEAR_SERVO_1 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim9.Instance->CCR1 = duty_cycle;
//            re = true;
//        }
////        htim9.Instance->CCR1 = duty_cycle;
//        break;
//    case G3_LINEAR_SERVO_2:
//        if(htim9.Instance->CCR2 == duty_cycle){
//            send_debugging_data("G3_LINEAR_SERVO_2 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim9.Instance->CCR2 = duty_cycle;
//            re = true;
//        }
////        htim9.Instance->CCR2 = duty_cycle;
//        break;
//    case G4_LINEAR_SERVO_1:
//        if(htim4.Instance->CCR3 == duty_cycle){
//            send_debugging_data("G4_LINEAR_SERVO_1 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim4.Instance->CCR3 = duty_cycle;
//            re = true;
//        }
////        htim4.Instance->CCR3 = duty_cycle;
//        break;
//    case G4_LINEAR_SERVO_2:
//        if(htim4.Instance->CCR4 == duty_cycle){
//            send_debugging_data("G4_LINEAR_SERVO_2 == duty_cycle \n\r");
//            re = false;
//        }
//        else{
//            htim4.Instance->CCR4 = duty_cycle;
//            re = true;
//        }
////        htim4.Instance->CCR4 = duty_cycle;
//        break;
//    default:
//        break;
//    }
//	return re;
//}

void ctrl_pwm_duty(uint8_t servo_no, uint16_t duty_cycle){
    switch (servo_no) {
    case G1_SERVO_1:
        htim1.Instance->CCR1 = duty_cycle;
        break;
    case G1_SERVO_2:
        htim1.Instance->CCR2 = duty_cycle;
        break;
    case G2_SERVO_1:
        htim3.Instance->CCR2 = duty_cycle;
        break;
    case G2_SERVO_2:
        htim3.Instance->CCR1 = duty_cycle;
        break;
    case G3_SERVO_1:
        htim4.Instance->CCR1 = duty_cycle;
        break;
    case G3_SERVO_2:
        htim4.Instance->CCR2 = duty_cycle;
        break;
    case G4_SERVO_1:
        htim1.Instance->CCR3 = duty_cycle;
        break;
    case G4_SERVO_2:
        htim1.Instance->CCR4 = duty_cycle;
        break;
    case G1_LINEAR_SERVO_1:
        htim8.Instance->CCR1 = duty_cycle;
        break;
    case G1_LINEAR_SERVO_2:
        htim8.Instance->CCR2 = duty_cycle;
        break;
    case G2_LINEAR_SERVO_1:
        htim8.Instance->CCR3 = duty_cycle;
        break;
    case G2_LINEAR_SERVO_2:
        htim8.Instance->CCR4 = duty_cycle;
        break;
    case G3_LINEAR_SERVO_1:
        htim9.Instance->CCR1 = duty_cycle;
        break;
    case G3_LINEAR_SERVO_2:
        htim9.Instance->CCR2 = duty_cycle;
        break;
    case G4_LINEAR_SERVO_1:
        htim4.Instance->CCR3 = duty_cycle;
        break;
    case G4_LINEAR_SERVO_2:
        htim4.Instance->CCR4 = duty_cycle;
        break;
    default:
        break;
    }
}

void check_allow_ctr_motor_sub_linear(void){
    // check linear1 = sub-linear
    uint8_t index;
    for(index = 0; index < 8; index+=2){
        if((motor_infor[index].status == CLAMP)&&(motor_infor[index+1].status == CLAMP)){
           allow_control_motor[index+8] = ALLOW_CTRL;
        }
        else{
            allow_control_motor[index+8] = NOT_ALLOW_CTRL;
        }
    }
}

uint8_t check_is_motor_linear1(uint8_t index){
    if((index == 8)||(index == 10)||(index == 12)||(index == 14))
        return SUCCESS_;

    return FAILED_;

}

uint8_t check_is_motor_linear2(uint8_t index){
    if((index == 9)||(index == 11)||(index == 13)||(index == 15))
        return SUCCESS_;

    return FAILED_;

}
