#include "main.h"


UART_HandleTypeDef usart1_pc_handel, uart_debug;

uint8_t start_process, start_gohome, check_system_init;

uint8_t motor_step[PWM_NUM_CHANNEL];

uint8_t check_calib_gohome_ctrl_step;
uint8_t start_auto_process[4];  // dieu kien de start control gripper
uint8_t check_on_auto_process[4]; // su dung de phan biet control auto and control by hand

void SystemClock_Config(void);
void gpio_clock_init(void);

uint8_t current_index[PWM_NUM_CHANNEL] = {0};


uint32_t time_point_1[PWM_NUM_CHANNEL];

uint16_t duty_cycle_hold_clamp[PWM_NUM_CHANNEL] = {1350,1350,1350,1350,1350,1350,1350,1350,1350,1300,1350,1300,1350,1350,1350,1300};

uint8_t first_time[PWM_NUM_CHANNEL] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int main(void){
    // init thread
    osThreadDef(control_motor_Thread, osPriorityNormal, 1, 1024);

    osKernelInitialize();
    HAL_Init();
    SystemClock_Config();
    SystemCoreClockUpdate();
    gpio_clock_init();
    uart_pc_init();
    uart_debug_init();
    Robotic_ADC_Init();
    Robotic_gpio_input();
    MX_TIMER_Init();
    Init_GPIO_servo_pwr();
    MX_I2C1_Init();
//	settingThermalSensorDT32L01A();
    Is_phone_in_grippers();

    send_debugging_data("Application Init system done \n\r");
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)read_adc_value, NUMBER_ADC_CHANNEL);
    osThreadCreate(osThread(control_motor_Thread), NULL);
    osKernelStart();
    osDelay(500); // 500 ms
    machine_init();
    send_debugging_data("Application ___ \n\r");
	//getValueThermalSensorD6T32L01A();
    while(1){
        osDelay(10);
        if(start_process == 1){
            process_data_input();
            start_process = 0;
        }
        // check phone in all grippers
        Is_phone_in_grippers();

        if(start_auto_process[GRIPPER_1] == START_AUTO_PROCESS){
            auto_process_ctrl(get_servo1_index_from_gripper_index(GRIPPER_1));
        }
        if(start_auto_process[GRIPPER_2] == START_AUTO_PROCESS){
            auto_process_ctrl(get_servo1_index_from_gripper_index(GRIPPER_2));
        }
        if(start_auto_process[GRIPPER_3] == START_AUTO_PROCESS){
            auto_process_ctrl(get_servo1_index_from_gripper_index(GRIPPER_3));
        }
        if(start_auto_process[GRIPPER_4] == START_AUTO_PROCESS){
            auto_process_ctrl(get_servo1_index_from_gripper_index(GRIPPER_4));
        }
    }
}

void update_motor_current(void){
    uint8_t index;
    for(index = 0; index < PWM_NUM_CHANNEL; index++){
        if(motor_infor[index].status == RUNNING){
            if(motor_infor[index].current < adc_value[index]){
                motor_infor[index].current = adc_value[index];
            }
        }
        else{
            motor_infor[index].current = 0;
        }
    }
}

uint8_t status[16];

void process_ctrl_servo(void){
    uint8_t index;
    // control servo motor: 0 -> 7 ; 3 status: running, clamp, release
    // servo is not turn of power at all
    for(index = 0; index < 8; index+=2){
        if(motor_step[index] == UNKNOW_STEP){// step 4: do nothing
            current_index[index] = 0;
            status[index] = UNKNOW_;
        }
        else if(motor_step[index] == STEP_1){// step 1: power on
            if(motor_step[index+8] == UNKNOW_STEP){
                status[index] = UNKNOW_;
                motor_step[index] = STEP_2;
                current_index[index] = 0;
                on_off_pwR(index, PIN_HIGH);
                motor_infor[index].status = RUNNING;
                first_time[index] = 1;

                status[index+1] = UNKNOW_;
                motor_step[index+1] = STEP_2;
                current_index[index+1] = 0;
                on_off_pwR(index+1, PIN_HIGH);
                motor_infor[index+1].status = RUNNING;
            }
        }
        else if(motor_step[index] == STEP_2){// step 2: control pwm and check current
            // xac dinh trang thai chuyen dong
            if(first_time[index] == 1){
//                if(motor_infor[index].duty < pl_control_motor[index].duty){
//                    motor_infor[index].duty = pl_control_motor[index].duty;
//                    status[index] = CLAMP;
//                    motor_infor[index+1].duty = pl_control_motor[index+1].duty;
//                    status[index+1] = CLAMP;
//                }
//                else if(motor_infor[index].duty > pl_control_motor[index].duty){
//                    motor_infor[index].duty = pl_control_motor[index].duty;
//                    status[index] = RELEASE;
//                    motor_infor[index+1].duty = pl_control_motor[index].duty;
//                    status[index+1] = RELEASE;
//                }
                if(Get_ctrl_proces[get_Gr_index_from_Mo_index(index)].step == PROCESS_CLAMP_SUB){
                    status[index] = CLAMP;
                    status[index+1] = CLAMP;
                }
                else if(Get_ctrl_proces[get_Gr_index_from_Mo_index(index)].step == PROCESS_RELEASE_SUB){
                    status[index] = RELEASE;
                    status[index+1] = RELEASE;
                }
                ctrl_pwm_duty(index, pl_control_motor[index].duty);
                ctrl_pwm_duty(index+1, pl_control_motor[index+1].duty);
                motor_infor[index].duty = pl_control_motor[index].duty;
                motor_infor[index+1].duty = pl_control_motor[index+1].duty;
                time_point_1[index] = HAL_GetTick();
            }
            first_time[index] = 0;
            if(os_time - time_point_1[index] >= 700){
                motor_step[index] = STEP_3;
                motor_step[index+1] = STEP_3;
            }
            else if(os_time - time_point_1[index] >= 500){
                if(status[index] == RELEASE){
                    motor_infor[index].status = status[index]; // change motor status
                    motor_infor[index+1].status = status[index+1]; // change motor status
                }
            }
        }
        else{// if(motor_step[index] == STEP_3){{// step 3: power off
            motor_infor[index].status = status[index]; // change motor status
            motor_infor[index+1].status = status[index+1]; // change motor status
            on_off_pwR(index, PIN_LOW);
            on_off_pwR(index+1, PIN_LOW);
            motor_step[index] = UNKNOW_STEP;
            motor_step[index+1] = UNKNOW_STEP;
            motor_infor[index].duty = pl_control_motor[index].duty;
            motor_infor[index+1].duty = pl_control_motor[index+1].duty;
        }
    }
    // sub linear motor only run when 2 servo motor is "CLAMP"
    // control sub-linear motor: 8,10,12,14
    for(index = 8; index < 16; index+=2){
        if(motor_step[index] == UNKNOW_STEP){// step 4: do nothing
            current_index[index] = 0;
            status[index] = UNKNOW_;
        }
        else if(motor_step[index] == STEP_1){// step 1: power on
            if(motor_step[index-8] == UNKNOW_STEP){
                motor_step[index] = STEP_2;
                current_index[index] = 0;
                on_off_pwR(index, PIN_HIGH);
                motor_infor[index].status = RUNNING;
                first_time[index] = 1;
            }
        }
        else if(motor_step[index] == STEP_2){// step 2: control pwm and check duty + current
            // sub linear is CLAM
            if(first_time[index] == 1){
//                if(motor_infor[index].duty > pl_control_motor[index].duty){ // means motor is retracting => clamping
//                    motor_infor[index].duty = pl_control_motor[index].duty;
//                    status[index] = CLAMP;
//                }
//                else if(motor_infor[index].duty < pl_control_motor[index].duty){ // means motor is extending => release
//                    motor_infor[index].duty = pl_control_motor[index].duty;
//                    status[index] = RELEASE;
//                }
                if(Get_ctrl_proces[get_Gr_index_from_Mo_index(index)].step == PROCESS_CLAMP_SUB){
                    status[index] = CLAMP;
                }
                else if(Get_ctrl_proces[get_Gr_index_from_Mo_index(index)].step == PROCESS_RELEASE_SUB){
                    status[index] = RELEASE;
                }
                time_point_1[index] = HAL_GetTick();
                motor_infor[index].duty = pl_control_motor[index].duty;
                ctrl_pwm_duty(index, pl_control_motor[index].duty);
            }
            first_time[index] = 0;
            if(os_time - time_point_1[index] >= 1500){
                motor_step[index] = STEP_3;
            }
        }
        else{// if(motor_step[index] == STEP_3){{// step 3: power off  => means release done
            motor_infor[index].status = status[index];
            motor_step[index] = UNKNOW_STEP;
            motor_infor[index].duty = pl_control_motor[index].duty;
            if(status[index] == CLAMP){
                ctrl_pwm_duty(index, duty_cycle_hold_clamp[index]);
            }
            else{
                on_off_pwR(index, PIN_LOW);
            }
        }
    }
    // control main-linear motor: 9,11,13,15
    for(index = 9; index < 16; index+=2){
        if(motor_step[index] == UNKNOW_STEP){// step 4: do nothing
        }
        else if(motor_step[index] == STEP_1){// step 1: power on
            if((motor_step[index-1] == UNKNOW_STEP)&&(motor_step[index-9] == UNKNOW_STEP)){
                on_off_pwR(index, PIN_HIGH);
                motor_step[index] = STEP_2;
                motor_infor[index].status = RUNNING;
                current_index[index] = 0;
                status[index] = UNKNOW_;
                first_time[index] = 1;
            }
        }
        else if(motor_step[index] == STEP_2){// step 2: control pwm and check duty + current
            // sub linear is CLAM
            if(first_time[index] == 1){
//                if(motor_infor[index].duty > pl_control_motor[index].duty){ // means motor is retracting => clamping
//                    motor_infor[index].duty = pl_control_motor[index].duty;
//                    status[index] = CLAMP;
//                }
//                else if(motor_infor[index].duty < pl_control_motor[index].duty){ // means motor is extending => release
//                    motor_infor[index].duty = pl_control_motor[index].duty;
//                    status[index] = RELEASE;
//                }
                if(Get_ctrl_proces[get_Gr_index_from_Mo_index(index)].step == PROCESS_CLAMP_MAIN){
                    status[index] = CLAMP;
                }
                else if(Get_ctrl_proces[get_Gr_index_from_Mo_index(index)].step == PROCESS_RELEASE_MAIN){
                    status[index] = RELEASE;
                }
                time_point_1[index] = HAL_GetTick();
                motor_infor[index].duty = pl_control_motor[index].duty;
                ctrl_pwm_duty(index, pl_control_motor[index].duty);
            }
            first_time[index] = 0;

            if(os_time - time_point_1[index] >= 1500){
                motor_step[index] = STEP_3;
            }
            else if(os_time - time_point_1[index] >= 900){
                if(status[index] == RELEASE){
                    motor_infor[index].status = status[index];
                }
                if(status[index] == CLAMP){
                    if(motor_infor[index].current >= CURRENT_LINEAR_MAX){
                        motor_step[index] = STEP_3;
                    }
                }
            }
        }
        else{// if(motor_step[index] == STEP_3){{// step 3: power off  => means release done
            if(status[index] == CLAMP){
                ctrl_pwm_duty(index, duty_cycle_hold_clamp[index]);
            }
            else{
                on_off_pwR(index, PIN_LOW);
            }
            motor_infor[index].status = status[index];
            motor_step[index] = UNKNOW_STEP;
            motor_infor[index].duty = pl_control_motor[index].duty;
        }
    }
}

void control_motor_Thread(void const *argument){
    send_debugging_data(" control_motor_Thread \n\r");
//    int i;
    uint8_t index = 0, i, j, k;
    uint8_t a = 5;
    uint16_t adc_buff[NUMBER_ADC_CHANNEL][a];

    for(i = 0; i< 8; i ++){
        release_duty_cycle[i] = 1100;
        clamp_duty_cycle[i] = 1850;
    }
    for(i = 8; i< 16; i = i+2){
        release_duty_cycle[i] = 1940;
        clamp_duty_cycle[i] = 1060;
        release_duty_cycle[i+1] = 1800;
        clamp_duty_cycle[i+1] = 1060;
    }
    check_system_init = 0;

    for(i=0; i<16; i++){
        time_point_1[i] = HAL_GetTick();
        motor_step[index] = UNKNOW_STEP;
    }
    // init servo motors
//    init_servo_motors();
    check_calib_gohome_ctrl_step = CALIB;
    check_system_init = 1;

    for(i=0; i<NUMBER_ADC_CHANNEL; i++){
        for(j=0;j<a; j++){
            adc_buff[i][j]=0;
        }
        duty_cycle_hold_clamp[i] = 1260;
    }

    for(;;)
    {
        // process control all servos
        process_ctrl_servo();

        for(k = 0; k<10; k++){
            for(i = 0; i < NUMBER_ADC_CHANNEL; i++){ // each motor
                adc_buff[i][index] = read_adc_value[i];  // buffer
                adc_value[i] = 0;
                for(j = 0; j<a; j++){  // trung binh cong
                    adc_value[i] += adc_buff[i][j];
                }
                adc_value[i] = adc_value[i] / (a);
            }
            update_motor_current();
            osDelay(2);
            index++;
            if(index >= a)  index = 0;
        }
    }
}

uint32_t HAL_GetTick(void)
{
    return os_time;
}


void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /* Enable Power Control clock */
    __PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency (see datasheet). */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
            RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

}

void gpio_clock_init(void){

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
//    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

}

void system_resetMCU(void){
    NVIC_SystemReset();
    //USBD_Reset(0);
    osDelay(2800);
    while(1);
}

void get_hw_version(void){
    uint8_t hw1,hw2, hw3;

    hw1 = HAL_GPIO_ReadPin(HW_CONFIG_1_PORT, HW_CONFIG_1_PIN);
    hw2 = HAL_GPIO_ReadPin(HW_CONFIG_2_PORT, HW_CONFIG_2_PIN);
    hw3 = HAL_GPIO_ReadPin(HW_CONFIG_3_PORT, HW_CONFIG_3_PIN);

    if((hw3 == 0)&&(hw2 == 0)&&(hw1 == 0)){
        strncpy(Grippers_infor.hw_version, "1.00", 4);
    }
    else if((hw3 == 0)&&(hw2 == 0)&&(hw1 == 1)){
        strncpy(Grippers_infor.hw_version, "1.01", 4);
    }
    else if((hw3 == 0)&&(hw2 == 1)&&(hw1 == 0)){
        strncpy(Grippers_infor.hw_version, "1.02", 4);
    }
    else if((hw3 == 0)&&(hw2 == 1)&&(hw1 == 1)){
        strncpy(Grippers_infor.hw_version, "1.03", 4);
    }
    else if((hw3 == 1)&&(hw2 == 0)&&(hw1 == 0)){
        strncpy(Grippers_infor.hw_version, "1.04", 4);
    }
    else if((hw3 == 1)&&(hw2 == 0)&&(hw1 == 1)){
        strncpy(Grippers_infor.hw_version, "1.05", 4);
    }
    else if((hw3 == 1)&&(hw2 == 1)&&(hw1 == 0)){
        strncpy(Grippers_infor.hw_version, "1.06", 4);
    }
    else{// if((hw3 == 1)&&(hw2 == 1)&&(hw1 == 1)){
        strncpy(Grippers_infor.hw_version, "1.07", 4);
    }
}



