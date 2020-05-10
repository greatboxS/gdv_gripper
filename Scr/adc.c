#include "main.h"
#include "adc.h"
#include "protocol.h"
#include <stm32f4xx_hal.h>
#include "uart_debug.h"


ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

uint16_t adc_value[NUMBER_ADC_CHANNEL];
uint16_t read_adc_value[NUMBER_ADC_CHANNEL];

void Robotic_ADC_Init(void)
{
    int index;
    ADC_ChannelConfTypeDef sConfig;
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = NUMBER_ADC_CHANNEL;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = DISABLE;
    HAL_ADC_Init(&hadc1);

    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    sConfig.Rank = 0;
    for(index = 0; index < NUMBER_ADC_CHANNEL; index++){
        GPIO_InitStruct.Pin = r_init_adc[index].pin;
        HAL_GPIO_Init(r_init_adc[index].port, &GPIO_InitStruct);
        sConfig.Rank = index + 1;

        sConfig.Channel = r_init_adc[index].channel;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    }

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	hdma_adc1.Instance = DMA2_Stream0;
	hdma_adc1.Init.Channel = DMA_CHANNEL_0;
	hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc1.Init.Mode = DMA_CIRCULAR;
	hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
	hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_adc1);


	__HAL_LINKDMA(&hadc1,DMA_Handle,hdma_adc1);


}


void Robotic_gpio_input(void){
    uint8_t index;
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    for(index = 0; index < 4; index++){
        GPIO_InitStruct.Pin = sensor_check_phone[index].pin;
        HAL_GPIO_Init(sensor_check_phone[index].port, &GPIO_InitStruct);
    }

    GPIO_InitStruct.Pin = HW_CONFIG_1_PIN;
    HAL_GPIO_Init(HW_CONFIG_1_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = HW_CONFIG_2_PIN;
    HAL_GPIO_Init(HW_CONFIG_2_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = HW_CONFIG_3_PIN;
    HAL_GPIO_Init(HW_CONFIG_3_PORT, &GPIO_InitStruct);
}

void Is_phone_in_grippers(void){
//    Grippers_infor.gripper_No1 = HAL_GPIO_ReadPin(sensor_check_phone[0].port, sensor_check_phone[0].pin);
//    Grippers_infor.gripper_No2 = HAL_GPIO_ReadPin(sensor_check_phone[1].port, sensor_check_phone[1].pin);
//    Grippers_infor.gripper_No3 = HAL_GPIO_ReadPin(sensor_check_phone[2].port, sensor_check_phone[2].pin);
//    Grippers_infor.gripper_No4 = HAL_GPIO_ReadPin(sensor_check_phone[3].port, sensor_check_phone[3].pin);

   static uint32_t get_stick_1[4] = {0};

    if(HAL_GPIO_ReadPin(sensor_check_phone[GRIPPER_1].port, sensor_check_phone[GRIPPER_1].pin)){
        Grippers_infor.gripper_No1 = HAVE_PHONE;
        get_stick_1[GRIPPER_1] = os_time;
    }
    else{
        if(((os_time - get_stick_1[GRIPPER_1]) > 300) && (HAL_GPIO_ReadPin(sensor_check_phone[GRIPPER_1].port, sensor_check_phone[GRIPPER_1].pin) != HAVE_PHONE)){
            Grippers_infor.gripper_No1 = NO_PHONE;
            get_stick_1[GRIPPER_1] = os_time;
        }
    }

    if(HAL_GPIO_ReadPin(sensor_check_phone[GRIPPER_2].port, sensor_check_phone[GRIPPER_2].pin)){
        Grippers_infor.gripper_No2 = HAVE_PHONE;
        get_stick_1[GRIPPER_2] = os_time;
    }
    else{
        if(((os_time - get_stick_1[GRIPPER_2]) > 300) && (HAL_GPIO_ReadPin(sensor_check_phone[GRIPPER_2].port, sensor_check_phone[GRIPPER_2].pin) != HAVE_PHONE)){
            Grippers_infor.gripper_No2 = NO_PHONE;
            get_stick_1[GRIPPER_2] = os_time;
        }
    }

    if(HAL_GPIO_ReadPin(sensor_check_phone[GRIPPER_3].port, sensor_check_phone[GRIPPER_3].pin)){
        Grippers_infor.gripper_No3 = HAVE_PHONE;
        get_stick_1[GRIPPER_3] = os_time;
    }
    else{
        if(((os_time - get_stick_1[GRIPPER_3]) > 300) && (HAL_GPIO_ReadPin(sensor_check_phone[GRIPPER_3].port, sensor_check_phone[GRIPPER_3].pin) != HAVE_PHONE)){
            Grippers_infor.gripper_No3 = NO_PHONE;
            get_stick_1[GRIPPER_3] = os_time;
        }
    }

    if(HAL_GPIO_ReadPin(sensor_check_phone[GRIPPER_4].port, sensor_check_phone[GRIPPER_4].pin)){
        Grippers_infor.gripper_No4 = HAVE_PHONE;
        get_stick_1[GRIPPER_4] = os_time;
    }
    else{
        if(((os_time - get_stick_1[GRIPPER_4]) > 300) && (HAL_GPIO_ReadPin(sensor_check_phone[GRIPPER_4].port, sensor_check_phone[GRIPPER_4].pin) != HAVE_PHONE)){
            Grippers_infor.gripper_No4 = NO_PHONE;
            get_stick_1[GRIPPER_4] = os_time;
        }
    }
}

void print_phone_in_gripper(void){
    if(Grippers_infor.gripper_No1 == HAVE_PHONE)
        send_debugging_data("Grippers_infor.gripper_No1 = HAVE_PHONE_ \n\r");
    else
        send_debugging_data("Grippers_infor.gripper_No1 = NO_PHONE_ \n\r");
    if(Grippers_infor.gripper_No2 == HAVE_PHONE)
        send_debugging_data("Grippers_infor.gripper_No2 = HAVE_PHONE_ \n\r");
    else
        send_debugging_data("Grippers_infor.gripper_No2 = NO_PHONE_ \n\r");
    if(Grippers_infor.gripper_No3 == HAVE_PHONE)
        send_debugging_data("Grippers_infor.gripper_No3 = HAVE_PHONE_ \n\r");
    else
        send_debugging_data("Grippers_infor.gripper_No3 = NO_PHONE_ \n\r");
    if(Grippers_infor.gripper_No4 == HAVE_PHONE)
        send_debugging_data("Grippers_infor.gripper_No4 = HAVE_PHONE_ \n\r");
    else
        send_debugging_data("Grippers_infor.gripper_No4 = NO_PHONE_ \n\r");
}
