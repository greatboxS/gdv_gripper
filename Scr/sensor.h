#ifndef SENSOR_H
#define SENSOR_H

//#include "adc.h"
#include "main.h"
#include "uart_debug.h"
#include "protocol.h"
#include <stdbool.h>

#define SCL_ULTRASONIC_PORT GPIOB
#define SCL_ULTRASONIC_PIN  GPIO_PIN_6

#define SDA_ULTRASONIC_PORT GPIOB
#define SDA_ULTRASONIC_PIN  GPIO_PIN_7

#define PWR_ULTRASONIC_PORT GPIOE
#define PWR_ULTRASONIC_PIN  GPIO_PIN_12


extern I2C_HandleTypeDef i2c_ultralsonic_handel;

void MX_I2C1_Init(void);
int settingThermalSensorDT32L01A(uint8_t doSetting, uint8_t valueReg00, uint8_t valueReg01, uint8_t valueReg02);
int getValueThermalSensorD6T32L01A(void);
void MX_GPIO_I2C_Init(void);
uint16_t read_ultrasonic_value(void);
void check_sensor_address(void);
void OnOffSensorPower(GPIO_PinState state);

#endif
