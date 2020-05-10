#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include "main.h"

// define for ADC (Analog Digital Conversion)
#define NUMBER_ADC_CHANNEL           16

#define R_ADC_DMA_CHANNEL                DMA_CHANNEL_0
#define R_ADC_DMA_STREAM                 DMA2_Stream0

// servo
#define R_ADC_CHANNEL_PIN_1                GPIO_PIN_5
#define R_ADC_CHANNEL_GPIO_PORT_1          GPIOC
#define R_ADC_CHANNEL_1                    ADC_CHANNEL_15

#define R_ADC_CHANNEL_PIN_2                GPIO_PIN_0
#define R_ADC_CHANNEL_GPIO_PORT_2          GPIOB
#define R_ADC_CHANNEL_2                    ADC_CHANNEL_8

#define R_ADC_CHANNEL_PIN_3                GPIO_PIN_1
#define R_ADC_CHANNEL_GPIO_PORT_3          GPIOB
#define R_ADC_CHANNEL_3                    ADC_CHANNEL_9

#define R_ADC_CHANNEL_PIN_4                GPIO_PIN_0
#define R_ADC_CHANNEL_GPIO_PORT_4          GPIOA
#define R_ADC_CHANNEL_4                    ADC_CHANNEL_0

#define R_ADC_CHANNEL_PIN_5                GPIO_PIN_1
#define R_ADC_CHANNEL_GPIO_PORT_5          GPIOA
#define R_ADC_CHANNEL_5                    ADC_CHANNEL_1

#define R_ADC_CHANNEL_PIN_6                GPIO_PIN_2
#define R_ADC_CHANNEL_GPIO_PORT_6          GPIOA
#define R_ADC_CHANNEL_6                    ADC_CHANNEL_2

#define R_ADC_CHANNEL_PIN_7                GPIO_PIN_3
#define R_ADC_CHANNEL_GPIO_PORT_7          GPIOA
#define R_ADC_CHANNEL_7                    ADC_CHANNEL_3

#define R_ADC_CHANNEL_PIN_8                GPIO_PIN_5
#define R_ADC_CHANNEL_GPIO_PORT_8          GPIOA
#define R_ADC_CHANNEL_8                    ADC_CHANNEL_5

// linear
#define R_ADC_CHANNEL_PIN_9                GPIO_PIN_0
#define R_ADC_CHANNEL_GPIO_PORT_9          GPIOC
#define R_ADC_CHANNEL_9                    ADC_CHANNEL_10

#define R_ADC_CHANNEL_PIN_10               GPIO_PIN_1
#define R_ADC_CHANNEL_GPIO_PORT_10         GPIOC
#define R_ADC_CHANNEL_10                   ADC_CHANNEL_11

#define R_ADC_CHANNEL_PIN_11               GPIO_PIN_2
#define R_ADC_CHANNEL_GPIO_PORT_11         GPIOC
#define R_ADC_CHANNEL_11                   ADC_CHANNEL_12

#define R_ADC_CHANNEL_PIN_12               GPIO_PIN_3
#define R_ADC_CHANNEL_GPIO_PORT_12         GPIOC
#define R_ADC_CHANNEL_12                   ADC_CHANNEL_13

#define R_ADC_CHANNEL_PIN_13               GPIO_PIN_4
#define R_ADC_CHANNEL_GPIO_PORT_13         GPIOA
#define R_ADC_CHANNEL_13                   ADC_CHANNEL_4

#define R_ADC_CHANNEL_PIN_14               GPIO_PIN_6
#define R_ADC_CHANNEL_GPIO_PORT_14         GPIOA
#define R_ADC_CHANNEL_14                   ADC_CHANNEL_6

#define R_ADC_CHANNEL_PIN_15               GPIO_PIN_7
#define R_ADC_CHANNEL_GPIO_PORT_15         GPIOA
#define R_ADC_CHANNEL_15                   ADC_CHANNEL_7

#define R_ADC_CHANNEL_PIN_16               GPIO_PIN_4
#define R_ADC_CHANNEL_GPIO_PORT_16         GPIOC
#define R_ADC_CHANNEL_16                   ADC_CHANNEL_14

// define for input GPIO - sensor detect phone
#define R_GPIO_INPUT_1_PIN                 GPIO_PIN_2
#define R_GPIO_INPUT_1_PORT                GPIOB

#define R_GPIO_INPUT_2_PIN                 GPIO_PIN_7
#define R_GPIO_INPUT_2_PORT                GPIOE

#define R_GPIO_INPUT_3_PIN                 GPIO_PIN_8
#define R_GPIO_INPUT_3_PORT                GPIOE

#define R_GPIO_INPUT_4_PIN                 GPIO_PIN_10
#define R_GPIO_INPUT_4_PORT                GPIOE

#define CURRENT_SERVO_MAX                       1500
#define CURRENT_SERVO_RUNNING_THREASHOD         50    // Neu dong dien motor lon hon => motor dang chay

#define CURRENT_LINEAR_MAX                      1200 //2100 // max = 550 mA <=> 1800
#define LINEAR_RUNNING_CURRENT_THREASHOD        50

typedef struct _GPIO_MODULE_{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint32_t channel;
}r_gpio_module;

typedef struct _R_GPIO_{
    GPIO_TypeDef *port;
    uint16_t      pin;
}r_gpio;

static const r_gpio_module r_init_adc[] = {
    {R_ADC_CHANNEL_GPIO_PORT_1, R_ADC_CHANNEL_PIN_1, R_ADC_CHANNEL_1},
    {R_ADC_CHANNEL_GPIO_PORT_2, R_ADC_CHANNEL_PIN_2, R_ADC_CHANNEL_2},
    {R_ADC_CHANNEL_GPIO_PORT_3, R_ADC_CHANNEL_PIN_3, R_ADC_CHANNEL_3},
    {R_ADC_CHANNEL_GPIO_PORT_4, R_ADC_CHANNEL_PIN_4, R_ADC_CHANNEL_4},
    {R_ADC_CHANNEL_GPIO_PORT_5, R_ADC_CHANNEL_PIN_5, R_ADC_CHANNEL_5},
    {R_ADC_CHANNEL_GPIO_PORT_6, R_ADC_CHANNEL_PIN_6, R_ADC_CHANNEL_6},
    {R_ADC_CHANNEL_GPIO_PORT_7, R_ADC_CHANNEL_PIN_7, R_ADC_CHANNEL_7},
    {R_ADC_CHANNEL_GPIO_PORT_8, R_ADC_CHANNEL_PIN_8, R_ADC_CHANNEL_8},
    {R_ADC_CHANNEL_GPIO_PORT_9, R_ADC_CHANNEL_PIN_9, R_ADC_CHANNEL_9},
    {R_ADC_CHANNEL_GPIO_PORT_10, R_ADC_CHANNEL_PIN_10, R_ADC_CHANNEL_10},
    {R_ADC_CHANNEL_GPIO_PORT_11, R_ADC_CHANNEL_PIN_11, R_ADC_CHANNEL_11},
    {R_ADC_CHANNEL_GPIO_PORT_12, R_ADC_CHANNEL_PIN_12, R_ADC_CHANNEL_12},
    {R_ADC_CHANNEL_GPIO_PORT_13, R_ADC_CHANNEL_PIN_13, R_ADC_CHANNEL_13},
    {R_ADC_CHANNEL_GPIO_PORT_14, R_ADC_CHANNEL_PIN_14, R_ADC_CHANNEL_14},
    {R_ADC_CHANNEL_GPIO_PORT_15, R_ADC_CHANNEL_PIN_15, R_ADC_CHANNEL_15},
    {R_ADC_CHANNEL_GPIO_PORT_16, R_ADC_CHANNEL_PIN_16, R_ADC_CHANNEL_16}
};

static const r_gpio sensor_check_phone[] = {
    {R_GPIO_INPUT_1_PORT, R_GPIO_INPUT_1_PIN},
    {R_GPIO_INPUT_2_PORT, R_GPIO_INPUT_2_PIN},
    {R_GPIO_INPUT_3_PORT, R_GPIO_INPUT_3_PIN},
    {R_GPIO_INPUT_4_PORT, R_GPIO_INPUT_4_PIN}
};
//(uint32_t *)
extern uint16_t adc_value[NUMBER_ADC_CHANNEL];
extern uint16_t read_adc_value[NUMBER_ADC_CHANNEL];
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

void Robotic_ADC_Init(void);
void Robotic_gpio_input(void);
void Is_phone_in_grippers(void);
void print_phone_in_gripper(void);

#endif // ADC_H
