#ifndef gripper_def_h
#define gripper_def_h
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

struct TIM_HandleTypeDef;
struct GPIO_TypeDef;
struct TIM_TypeDef;

#define SERVO_PWR_ON    GPIO_PIN_SET
#define SERVO_PWR_OFF   GPIO_PIN_RESET

#define PIN_HIGH        GPIO_PIN_SET
#define PIN_LOW         GPIO_PIN_RESET

// define for ADC (Analog Digital Conversion)
#define NUMBER_ADC_CHANNEL           16

#define R_ADC_DMA_CHANNEL                DMA_CHANNEL_0
#define R_ADC_DMA_STREAM                 DMA2_Stream0

//********************************************************
#define SERVO_1_TIM                 TIM1
#define SERVO_1_PWM_PIN             GPIO_PIN_9
#define SERVO_1_PWM_PORT            GPIOE
#define SERVO_1_TIM_CHANNEL			TIM_CHANNEL_1

#define SERVO_1_PWR_PIN             GPIO_PIN_2
#define SERVO_1_PWR_PORT            GPIOE

#define R_ADC_CHANNEL_PIN_1                GPIO_PIN_5
#define R_ADC_CHANNEL_GPIO_PORT_1          GPIOC
#define R_ADC_CHANNEL_1                    ADC_CHANNEL_15
//********************************************************

#define SERVO_2_TIM                 TIM1
#define SERVO_2_PWM_PIN             GPIO_PIN_11
#define SERVO_2_PWM_PORT            GPIOE
#define SERVO_2_TIM_CHANNEL			TIM_CHANNEL_2

#define SERVO_2_PWR_PIN             GPIO_PIN_3
#define SERVO_2_PWR_PORT            GPIOE

#define R_ADC_CHANNEL_PIN_2                GPIO_PIN_0
#define R_ADC_CHANNEL_GPIO_PORT_2          GPIOB
#define R_ADC_CHANNEL_2                    ADC_CHANNEL_8
//********************************************************

#define SERVO_3_TIM                 TIM3
#define SERVO_3_PWM_PIN             GPIO_PIN_5
#define SERVO_3_PWM_PORT            GPIOB
#define SERVO_3_TIM_CHANNEL			TIM_CHANNEL_2

#define SERVO_3_PWR_PIN             GPIO_PIN_4
#define SERVO_3_PWR_PORT            GPIOE

#define R_ADC_CHANNEL_PIN_3                GPIO_PIN_1
#define R_ADC_CHANNEL_GPIO_PORT_3          GPIOB
#define R_ADC_CHANNEL_3                    ADC_CHANNEL_9
//********************************************************

#define SERVO_4_TIM                 TIM3
#define SERVO_4_PWM_PIN             GPIO_PIN_4
#define SERVO_4_PWM_PORT            GPIOB
#define SERVO_4_TIM_CHANNEL			TIM_CHANNEL_1

#define SERVO_4_PWR_PIN             GPIO_PIN_13 // PC.13
#define SERVO_4_PWR_PORT            GPIOC

#define R_ADC_CHANNEL_PIN_4                GPIO_PIN_0
#define R_ADC_CHANNEL_GPIO_PORT_4          GPIOA
#define R_ADC_CHANNEL_4                    ADC_CHANNEL_0
//********************************************************

#define SERVO_5_TIM                 TIM4
#define SERVO_5_PWM_PIN             GPIO_PIN_12
#define SERVO_5_PWM_PORT            GPIOD
#define SERVO_5_TIM_CHANNEL			TIM_CHANNEL_1

#define SERVO_5_PWR_PIN             GPIO_PIN_7 // PD.7
#define SERVO_5_PWR_PORT            GPIOD

#define R_ADC_CHANNEL_PIN_5                GPIO_PIN_1
#define R_ADC_CHANNEL_GPIO_PORT_5          GPIOA
#define R_ADC_CHANNEL_5                    ADC_CHANNEL_1
//********************************************************

#define SERVO_6_TIM                 TIM4
#define SERVO_6_PWM_PIN             GPIO_PIN_13
#define SERVO_6_PWM_PORT            GPIOD
#define SERVO_6_TIM_CHANNEL			TIM_CHANNEL_2

#define SERVO_6_PWR_PIN             GPIO_PIN_6 // PD.6
#define SERVO_6_PWR_PORT            GPIOD

#define R_ADC_CHANNEL_PIN_6                GPIO_PIN_2
#define R_ADC_CHANNEL_GPIO_PORT_6          GPIOA
#define R_ADC_CHANNEL_6                    ADC_CHANNEL_2
//********************************************************

#define SERVO_7_TIM                 TIM1
#define SERVO_7_PWM_PIN             GPIO_PIN_13
#define SERVO_7_PWM_PORT            GPIOE
#define SERVO_7_TIM_CHANNEL			TIM_CHANNEL_3

#define SERVO_7_PWR_PIN             GPIO_PIN_5 // PD.5
#define SERVO_7_PWR_PORT            GPIOD

#define R_ADC_CHANNEL_PIN_7                GPIO_PIN_3
#define R_ADC_CHANNEL_GPIO_PORT_7          GPIOA
#define R_ADC_CHANNEL_7                    ADC_CHANNEL_3
//********************************************************

#define SERVO_8_TIM                 TIM1
#define SERVO_8_PWM_PIN             GPIO_PIN_14
#define SERVO_8_PWM_PORT            GPIOE
#define SERVO_8_TIM_CHANNEL			TIM_CHANNEL_4

#define SERVO_8_PWR_PIN             GPIO_PIN_4 // PD.4
#define SERVO_8_PWR_PORT            GPIOD

#define R_ADC_CHANNEL_PIN_8                GPIO_PIN_5
#define R_ADC_CHANNEL_GPIO_PORT_8          GPIOA
#define R_ADC_CHANNEL_8                    ADC_CHANNEL_5
//********************************************************

#define LINEAR_1_TIM                TIM8
#define LINEAR_1_PWM_PIN            GPIO_PIN_6
#define LINEAR_1_PWM_PORT           GPIOC
#define LINEAR_1_TIM_CHANNEL        TIM_CHANNEL_1

#define LINEAR_1_PWR_PIN             GPIO_PIN_1
#define LINEAR_1_PWR_PORT            GPIOE

#define R_ADC_CHANNEL_PIN_9                GPIO_PIN_0
#define R_ADC_CHANNEL_GPIO_PORT_9          GPIOC
#define R_ADC_CHANNEL_9                    ADC_CHANNEL_10
//********************************************************

#define LINEAR_2_TIM                TIM8
#define LINEAR_2_PWM_PIN            GPIO_PIN_7
#define LINEAR_2_PWM_PORT           GPIOC
#define LINEAR_2_TIM_CHANNEL        TIM_CHANNEL_2

#define LINEAR_2_PWR_PIN             GPIO_PIN_0
#define LINEAR_2_PWR_PORT            GPIOE

#define R_ADC_CHANNEL_PIN_10               GPIO_PIN_1
#define R_ADC_CHANNEL_GPIO_PORT_10         GPIOC
#define R_ADC_CHANNEL_10                   ADC_CHANNEL_11
//********************************************************

#define LINEAR_3_TIM                TIM8
#define LINEAR_3_PWM_PIN            GPIO_PIN_8
#define LINEAR_3_PWM_PORT           GPIOC
#define LINEAR_3_TIM_CHANNEL        TIM_CHANNEL_3

#define LINEAR_3_PWR_PIN             GPIO_PIN_9
#define LINEAR_3_PWR_PORT            GPIOD

#define R_ADC_CHANNEL_PIN_11               GPIO_PIN_2
#define R_ADC_CHANNEL_GPIO_PORT_11         GPIOC
#define R_ADC_CHANNEL_11                   ADC_CHANNEL_12
//********************************************************

#define LINEAR_4_TIM                TIM8
#define LINEAR_4_PWM_PIN            GPIO_PIN_9
#define LINEAR_4_PWM_PORT           GPIOC
#define LINEAR_4_TIM_CHANNEL        TIM_CHANNEL_4

#define LINEAR_4_PWR_PIN             GPIO_PIN_11
#define LINEAR_4_PWR_PORT            GPIOA

#define R_ADC_CHANNEL_PIN_12               GPIO_PIN_3
#define R_ADC_CHANNEL_GPIO_PORT_12         GPIOC
#define R_ADC_CHANNEL_12                   ADC_CHANNEL_13
//********************************************************

#define LINEAR_5_TIM                TIM9
#define LINEAR_5_PWM_PIN            GPIO_PIN_5
#define LINEAR_5_PWM_PORT           GPIOE
#define LINEAR_5_TIM_CHANNEL        TIM_CHANNEL_1

#define LINEAR_5_PWR_PIN             GPIO_PIN_8
#define LINEAR_5_PWR_PORT            GPIOA

#define R_ADC_CHANNEL_PIN_13               GPIO_PIN_4
#define R_ADC_CHANNEL_GPIO_PORT_13         GPIOA
#define R_ADC_CHANNEL_13                   ADC_CHANNEL_4
//********************************************************

#define LINEAR_6_TIM                TIM9
#define LINEAR_6_PWM_PIN            GPIO_PIN_6
#define LINEAR_6_PWM_PORT           GPIOE
#define LINEAR_6_TIM_CHANNEL        TIM_CHANNEL_2

#define LINEAR_6_PWR_PIN             GPIO_PIN_15
#define LINEAR_6_PWR_PORT            GPIOD

#define R_ADC_CHANNEL_PIN_14               GPIO_PIN_6
#define R_ADC_CHANNEL_GPIO_PORT_14         GPIOA
#define R_ADC_CHANNEL_14                   ADC_CHANNEL_6
//********************************************************

#define LINEAR_7_TIM                TIM4
#define LINEAR_7_PWM_PIN            GPIO_PIN_8
#define LINEAR_7_PWM_PORT           GPIOB
#define LINEAR_7_TIM_CHANNEL        TIM_CHANNEL_3

#define LINEAR_7_PWR_PIN             GPIO_PIN_11
#define LINEAR_7_PWR_PORT            GPIOD

#define R_ADC_CHANNEL_PIN_15               GPIO_PIN_7
#define R_ADC_CHANNEL_GPIO_PORT_15         GPIOA
#define R_ADC_CHANNEL_15                   ADC_CHANNEL_7
//********************************************************

#define LINEAR_8_TIM                TIM4
#define LINEAR_8_PWM_PIN            GPIO_PIN_9
#define LINEAR_8_PWM_PORT           GPIOB
#define LINEAR_8_TIM_CHANNEL        TIM_CHANNEL_4

#define LINEAR_8_PWR_PIN             GPIO_PIN_10
#define LINEAR_8_PWR_PORT            GPIOD

#define R_ADC_CHANNEL_PIN_16               GPIO_PIN_4
#define R_ADC_CHANNEL_GPIO_PORT_16         GPIOC
#define R_ADC_CHANNEL_16                   ADC_CHANNEL_14
//********************************************************

// define for input GPIO - sensor detect phone
#define R_GPIO_INPUT_1_PIN                 GPIO_PIN_2
#define R_GPIO_INPUT_1_PORT                GPIOB

#define R_GPIO_INPUT_2_PIN                 GPIO_PIN_7
#define R_GPIO_INPUT_2_PORT                GPIOE

#define R_GPIO_INPUT_3_PIN                 GPIO_PIN_8
#define R_GPIO_INPUT_3_PORT                GPIOE

#define R_GPIO_INPUT_4_PIN                 GPIO_PIN_10
#define R_GPIO_INPUT_4_PORT                GPIOE
//********************************************************

typedef struct servo_peripheral_block{
    // pwm block
    TIM_TypeDef* tim;

    GPIO_TypeDef* pwm_port;

    uint16_t pwm_pin;

    uint32_t tim_channel;

    // pwr block
    GPIO_TypeDef* pwr_port;

    uint16_t pwr_pin;

    // adc block
    GPIO_TypeDef* adc_port;

    uint16_t adc_pin;

    uint32_t adc_channel;

}servo_peripheral_block;

typedef struct sensor_block{
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_pin;
}sensor_block;
//********************************************************
static const servo_peripheral_block servo_peripheral_block_def[] = {
    {SERVO_1_TIM, SERVO_1_PWM_PORT,SERVO_1_PWM_PIN,SERVO_1_TIM_CHANNEL,
     SERVO_1_PWR_PORT, SERVO_1_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_1, R_ADC_CHANNEL_PIN_1, R_ADC_CHANNEL_1
    },
    {SERVO_2_TIM, SERVO_2_PWM_PORT,SERVO_2_PWM_PIN,SERVO_2_TIM_CHANNEL,
     SERVO_2_PWR_PORT, SERVO_2_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_2, R_ADC_CHANNEL_PIN_2, R_ADC_CHANNEL_2
    },
    {SERVO_3_TIM, SERVO_3_PWM_PORT,SERVO_3_PWM_PIN,SERVO_3_TIM_CHANNEL,
     SERVO_3_PWR_PORT, SERVO_3_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_3, R_ADC_CHANNEL_PIN_3, R_ADC_CHANNEL_3
    },
    {SERVO_4_TIM, SERVO_4_PWM_PORT,SERVO_4_PWM_PIN,SERVO_4_TIM_CHANNEL,
     SERVO_4_PWR_PORT, SERVO_4_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_4, R_ADC_CHANNEL_PIN_4, R_ADC_CHANNEL_4
    },
    {SERVO_5_TIM, SERVO_5_PWM_PORT,SERVO_5_PWM_PIN,SERVO_5_TIM_CHANNEL,
     SERVO_5_PWR_PORT, SERVO_5_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_5, R_ADC_CHANNEL_PIN_5, R_ADC_CHANNEL_5
    },
    {SERVO_6_TIM, SERVO_6_PWM_PORT,SERVO_6_PWM_PIN,SERVO_6_TIM_CHANNEL,
     SERVO_6_PWR_PORT, SERVO_6_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_6, R_ADC_CHANNEL_PIN_6, R_ADC_CHANNEL_6
    },
    {SERVO_7_TIM, SERVO_7_PWM_PORT,SERVO_7_PWM_PIN,SERVO_7_TIM_CHANNEL,
     SERVO_7_PWR_PORT, SERVO_7_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_7, R_ADC_CHANNEL_PIN_7, R_ADC_CHANNEL_7
    },
    {SERVO_8_TIM, SERVO_8_PWM_PORT,SERVO_8_PWM_PIN,SERVO_8_TIM_CHANNEL,
     SERVO_8_PWR_PORT, SERVO_8_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_8, R_ADC_CHANNEL_PIN_8, R_ADC_CHANNEL_8
    },
    {LINEAR_1_TIM, LINEAR_1_PWM_PORT, LINEAR_1_PWM_PIN,  LINEAR_1_TIM_CHANNEL,
     LINEAR_1_PWR_PORT, LINEAR_1_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_9, R_ADC_CHANNEL_PIN_9, R_ADC_CHANNEL_9
    },
    {LINEAR_2_TIM, LINEAR_2_PWM_PORT, LINEAR_2_PWM_PIN,  LINEAR_2_TIM_CHANNEL,
     LINEAR_2_PWR_PORT, LINEAR_2_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_10, R_ADC_CHANNEL_PIN_10, R_ADC_CHANNEL_10
    },
    {LINEAR_3_TIM, LINEAR_3_PWM_PORT, LINEAR_3_PWM_PIN,  LINEAR_3_TIM_CHANNEL,
     LINEAR_3_PWR_PORT, LINEAR_3_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_11, R_ADC_CHANNEL_PIN_11, R_ADC_CHANNEL_11
    },
    {LINEAR_4_TIM, LINEAR_4_PWM_PORT, LINEAR_4_PWM_PIN,  LINEAR_4_TIM_CHANNEL,
     LINEAR_4_PWR_PORT, LINEAR_4_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_12, R_ADC_CHANNEL_PIN_12, R_ADC_CHANNEL_12
    },
    {LINEAR_5_TIM, LINEAR_5_PWM_PORT, LINEAR_5_PWM_PIN,  LINEAR_5_TIM_CHANNEL,
     LINEAR_5_PWR_PORT, LINEAR_5_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_13, R_ADC_CHANNEL_PIN_13, R_ADC_CHANNEL_13
    },
    {LINEAR_6_TIM, LINEAR_6_PWM_PORT, LINEAR_6_PWM_PIN,  LINEAR_6_TIM_CHANNEL,
     LINEAR_6_PWR_PORT, LINEAR_6_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_13, R_ADC_CHANNEL_PIN_13, R_ADC_CHANNEL_13
    },
    {LINEAR_7_TIM, LINEAR_7_PWM_PORT, LINEAR_7_PWM_PIN,  LINEAR_7_TIM_CHANNEL,
     LINEAR_7_PWR_PORT, LINEAR_7_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_14, R_ADC_CHANNEL_PIN_14, R_ADC_CHANNEL_14
    },
    {LINEAR_8_TIM, LINEAR_8_PWM_PORT, LINEAR_8_PWM_PIN,  LINEAR_8_TIM_CHANNEL,
     LINEAR_8_PWR_PORT, LINEAR_8_PWR_PIN,
     R_ADC_CHANNEL_GPIO_PORT_15, R_ADC_CHANNEL_PIN_15, R_ADC_CHANNEL_15
    },
};

static const sensor_block slot_sensor_block[] = {
    {R_GPIO_INPUT_1_PORT, R_GPIO_INPUT_1_PIN},
    {R_GPIO_INPUT_2_PORT, R_GPIO_INPUT_2_PIN},
    {R_GPIO_INPUT_3_PORT, R_GPIO_INPUT_3_PIN},
    {R_GPIO_INPUT_4_PORT, R_GPIO_INPUT_4_PIN}
};

typedef enum GripperSlot_Def{
    GRIPPER_SLOT_1,
    GRIPPER_SLOT_2,
    GRIPPER_SLOT_3,
    GRIPPER_SLOT_4,

}GripperSlot_Def;

typedef enum SlotServo_Def{
    SLOT1_N_SERVO1 = 0,
    SLOT1_N_SERVO2 = 1,
    SLOT1_L_SERVO_MAIN = 8,
    SLOT1_L_SERVO_SUB = 9,

    SLOT2_N_SERVO1 = 2,
    SLOT2_N_SERVO2 = 3,
    SLOT2_L_SERVO_MAIN = 10,
    SLOT2_L_SERVO_SUB = 11,

    SLOT3_N_SERVO1 = 4,
    SLOT3_N_SERVO2 = 5,
    SLOT3_L_SERVO_MAIN = 12,
    SLOT3_L_SERVO_SUB = 13,

    SLOT4_N_SERVO1 = 6,
    SLOT4_N_SERVO2 = 7,
    SLOT4_L_SERVO_MAIN = 14,
    SLOT4_L_SERVO_SUB = 15,
}SlotServo_Def;

typedef enum ProtocolCmd_Def{
    CMD_READ_VERSION			=0x02,
    CMD_JUMP_BOOTLOADER         =0x15,
    CMD_START_UPGRADE           =0x16,
    CMD_UPGRADE_RUNNING         =0x17,
    CMD_UPGRADE_FINNIHED        =0x18,
    CMD_TRANSFER_TESTING        =0x19,
    CMD_CTRL_GOHOME             =0x1A,
    CMD_GET_MOTOR_STATUS        =0x1B,
    CMD_CHECK_PHONE_IN          =0x1C,
    CMD_CTRL_MOTOR              =0x1D,
    CMD_CTRL_AUTO_PROCESS       =0x1E,
    CMD_CTRL_CALIB              =0x1F,
    CMD_SW_WRITE_CALIB          =0x20,
    CMD_SPECIAL_RELEASE_FUNC    =0x21,
    CMD_READ_RANGE_SENSOR       =0x22,
    CMD_CTRL_ALL_SUB_LINEAR     =0x23,
    CMD_CTRL_SENSOR_POWER       =0x24,
    CMD_READ_THERMAL_SENSOR     =0x25,

}ProtocolCmd_Def;

typedef enum ProtocolResponse_Def{
    RESPOND_SUCESS              =0x01,
    RESPOND_FAILED              =0x02,
    RESPOND_UNKNOW              =0x03,    // Command is not supported
    RESPOND_CHECKSUM_ERROR      =0x04,    // Checksum error

    RESPOND_CANNOT_GOHOME       =0x05,
    RESPOND_CANNOT_CALIB        =0x06,
    RESPOND_CANNOT_CONTROL      =0x07,
    RESPOND_SYSTEM_INIT         =0x08,
    RESPOND_RUNNING             =0x09,
}ProtocolResponse_Def;


typedef enum ServoControl_Def{
    MOTOR_OFF,
    MOTOR_ON,
    MOTOR_SET_PWM,
    MOTOR_RUNNING,
    MOTOR_CLAMP,
    MOTOR_RELEASE,
    MOTOR_SPECIAL_RELEASE,
    MOTOR_AUTO,
    MOTOR_MANUAL
}ServoContrl_Def;

typedef enum GrippderSlotProcess_Def{
    PROCESS_CLAMP_MAIN = 1,
    PROCESS_CLAMP_SUB = 2,
    PROCESS_RELEASE_MAIN = 3,
    PROCESS_RELEASE_SUB = 4
}GrippderSlotProcess_Def;

typedef enum GripperSlotControl_Def{
    SLOT_CTRL_CLAMP_MAIN_1,
    SLOT_CTRL_CLAMP_SUB_1,
    SLOT_CTRL_RELEASE_MAIN_1,
    SLOT_CTRL_RELEASE_SUB_1,

    SLOT_CTRL_CLAMP_MAIN_2,
    SLOT_CTRL_CLAMP_SUB_2,
    SLOT_CTRL_RELEASE_MAIN_2,
    SLOT_CTRL_RELEASE_SUB_2,

}GripperSlotControl_Def;

typedef struct servo_control_block{
    uint8_t servo_id;

    uint8_t motor_status;

    ServoContrl_Def motor_control;

    uint8_t alow_ctrl;

    uint16_t pwm_duty_val;

    uint16_t pwm_calib_val;

    uint16_t* adc_value_ptr;

    servo_peripheral_block peripheral_block;

}servo_control_block;

typedef struct gripper_slot_control_block{

    GripperSlot_Def slot_id;

    GrippderSlotProcess_Def slot_process;

    uint8_t slot_status;

    uint8_t phone_sensor_status;

    sensor_block slot_sensor_block;

    servo_control_block linear_main_servo;

    servo_control_block linear_sub_servo;

    servo_control_block normal_servo1;

    servo_control_block normal_servo2;

}gripper_slot_control_block;

extern gripper_slot_control_block gripper_slot_block[4];

void gripper_init();
uint8_t gripper_slot_init(gripper_slot_control_block* slot);
void servo_init(servo_control_block* servo);
void servo_set_pwm_duty_cycle(servo_control_block* servo_ctrl_block, uint16_t duty_cycle);
uint8_t servo_set_pwm_duty_cycle(uint8_t servo_id, uint16_t duty_cycle);
uint8_t servo_ctrl_pwr_pin(uint8_t servo_id, int pin_state );
void servo_ctrl_pwr_pin(servo_control_block* servo, uint16_t pin_state);
void process_control_servo(servo_control_block* servo);
void process_control_servo(gripper_slot_control_block* gripper_slot);

#endif
