#include "sensor.h"

I2C_HandleTypeDef i2c_ultralsonic_handel;

uint8_t i2c_address;
uint8_t Tx_buff[2];
uint8_t Rx_buff[2];
uint8_t data[2051]={0};
uint16_t i = 0, j=0;
uint16_t tp[1024]={0};
uint16_t tPTAT = 0;
uint16_t tPEC = 0;
uint8_t buffer[2];

#define SLAVE_ADDR 0x0A // 7-bit, right aligned

void MX_I2C1_Init(void)
{
    i2c_ultralsonic_handel.Instance = I2C1;
    i2c_ultralsonic_handel.Init.ClockSpeed = 400000;
    i2c_ultralsonic_handel.Init.DutyCycle = I2C_DUTYCYCLE_2;
    i2c_ultralsonic_handel.Init.OwnAddress1 = 0;
    i2c_ultralsonic_handel.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c_ultralsonic_handel.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c_ultralsonic_handel.Init.OwnAddress2 = 0;
    i2c_ultralsonic_handel.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c_ultralsonic_handel.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&i2c_ultralsonic_handel) != HAL_OK){                                 // Config I2C
        send_debugging_data("init I2C failed \n\r");
    }
    else{
        send_debugging_data("init I2C okkkk\n\r");
    }

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin = PWR_ULTRASONIC_PIN;
    HAL_GPIO_Init(PWR_ULTRASONIC_PORT, &GPIO_InitStruct);


    HAL_GPIO_WritePin(PWR_ULTRASONIC_PORT, PWR_ULTRASONIC_PIN, GPIO_PIN_SET);
    osDelay(500);

}

bool I2C1_waitForReady(I2C_HandleTypeDef* hi2c, uint32_t timeout_ms)
{
    uint32_t tickstart = HAL_GetTick();
    while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
    {
        if((timeout_ms==0) || ((HAL_GetTick() - tickstart) > timeout_ms))  return false;
    }

    return true;
}
int settingThermalSensorDT32L01A(uint8_t doSetting, uint8_t valueReg00, uint8_t valueReg01, uint8_t valueReg02)
{

	if (doSetting != 1) return SUCCESS_;
    if(!I2C1_waitForReady(&i2c_ultralsonic_handel, 1000)) {
		return FAILED_;
	}

    buffer[0]= 0x00;
    buffer[1]= valueReg00;

    if(HAL_I2C_Master_Transmit(&i2c_ultralsonic_handel,(uint16_t)(0xA<<1),(uint8_t *)buffer,2,100) != HAL_OK){
		return FAILED_;
    }

    buffer[0]= 0x01;
    buffer[1]= valueReg01;
    if(HAL_I2C_Master_Transmit(&i2c_ultralsonic_handel,(uint16_t)(0xA<<1),(uint8_t *)buffer,2,100) != HAL_OK){
		return FAILED_;
    }

    buffer[0]= 0x02;
    buffer[1]= valueReg02;
    if(HAL_I2C_Master_Transmit(&i2c_ultralsonic_handel,(uint16_t)(0xA<<1),(uint8_t *)buffer,2,100) != HAL_OK){
		send_debugging_data("valueReg02 failed\n\r");
		return FAILED_;
    }
 	osDelay(300);

	return SUCCESS_;
}
int getValueThermalSensorD6T32L01A(void)
{
	if(!I2C1_waitForReady(&i2c_ultralsonic_handel, 1000)) {
		                        send_debugging_data("111valueReg02 failed\n\r");
return FAILED_;
	}
	
	uint8_t res;
	memset(thermal_sensor.Rxbuf,0, 2051);	
	memset(data,0, 2051);	

	res = HAL_I2C_Mem_Read(&i2c_ultralsonic_handel,(uint16_t)(0xA<<1),(uint16_t)(0x4D),I2C_MEMADD_SIZE_8BIT, data,2051,1000) ;
    if( res!= HAL_OK){
		                        send_debugging_data("i2c_ultralsonic_handel failed\n\r");
return FAILED_;
    }
    else{

//        tPTAT = 256*data[1] + data[0];
//        for( j = 0; j< 1024; j++){
//            tp[j] = 256*data[j*2+3] + data[j*2+2];
//        }
//        tPEC 	 = 	   data[2050];
//		sprintf(str__, "tPTAT = %04d \n\r", tPTAT );
//        send_debugging_data(str__);
//        for( j = 0; j< 1024; j++){
////			if((j+1)%32==0)
////			{
////				sprintf(str__, "\n\r");
////			}
////			else 
//				sprintf(str__, "%04d ", tp[j]);
//            send_debugging_data(str__);
//        }
//        sprintf(str__, "tPEC = %04d \n\r", tPEC );
//        send_debugging_data(str__);
//        for( i = 0; i< 2051; i++){            
//			sprintf(str__, "%02x ", data[i]);
//            send_debugging_data(str__);
//        }
//		memcpy(thermal_sensor.Rxbuf, data, 2051);
//		send_response_data(data,2051);
		memcpy(thermal_sensor.Rxbuf, data, 2051);			
    }

	return SUCCESS_;
}



/*
 * read on this webside for more information:
 * https://www.maxbotix.com/articles/095.htm
 */
//float LPF_Beta = 0.35;
uint16_t read_ultrasonic_value(void){
    //    static float range_smooth;
    uint16_t range, result;
    uint8_t res;
    uint8_t i;
    Tx_buff[0] = 81;
    Rx_buff[0] = Rx_buff[1] = 0;
    i2c_address = 224;
    result = 0;

    //    HAL_GPIO_WritePin(PWR_ULTRASONIC_PORT, PWR_ULTRASONIC_PIN, GPIO_PIN_SET);
    //    osDelay(500);

    if(!I2C1_waitForReady(&i2c_ultralsonic_handel, 1000))
        return 0;// as failed

    // start sensor
    //    Tx_buff[0] = 224;
    //    res = HAL_I2C_Master_Transmit(&i2c_ultralsonic_handel, (uint16_t)i2c_address, (uint8_t *)Tx_buff, 0, 10000);
    //    send_debugging_data(str__);
    //    if(res != HAL_OK){
    //        sprintf(str__, "FAILED start sensor with address = %d \n\r", i2c_address);
    //        send_debugging_data(str__);
    //    }
    //// wait 100ms
    //    osDelay(110);

    // send cmd read range
    Tx_buff[0] = 81;
    res = HAL_I2C_Master_Transmit(&i2c_ultralsonic_handel, (uint16_t)i2c_address, (uint8_t *)Tx_buff, 1, 1000);
    send_debugging_data(str__);
    if(res != HAL_OK){
        sprintf(str__, "FAILED sent cmd read sensor with address = %d \n\r", i2c_address);
        send_debugging_data(str__);
    }
    osDelay(150);

    for(i = 0; i <2; i++){
        // read data
        res = HAL_I2C_Master_Receive(&i2c_ultralsonic_handel, (uint16_t)(i2c_address), (uint8_t *)Rx_buff, 2, 1000);
        send_debugging_data(str__);
        if(res != HAL_OK){
            sprintf(str__, "get data i2c failed = %d with address = %d \n\r", res, i2c_address);
            send_debugging_data(str__);
            //            continue;
            return 1;
        }

        range = Rx_buff[0]*256 + Rx_buff[1];
        Rx_buff[0] = Rx_buff[1] = 0;
        //    range_smooth = range_smooth - (LPF_Beta * (range_smooth - range));

        sprintf(str__, "read_ultrasonic_value = %d cm \n\r", range);
        send_debugging_data(str__);
        osDelay(100);

        result += range;
    }
    result = result/2;

    sprintf(str__, "read_ultrasonic_value final = %d cm \n\r", result);
    send_debugging_data(str__);

    //    HAL_GPIO_WritePin(PWR_ULTRASONIC_PORT, PWR_ULTRASONIC_PIN, GPIO_PIN_RESET);

    return result;
}


void check_sensor_address(void){
    uint16_t i;
    for( i = 0; i< 255; i++){
        if(HAL_I2C_IsDeviceReady(&i2c_ultralsonic_handel, (uint16_t)i, 1, 1000) == HAL_OK){
            break;
        }
        HAL_Delay(30);
    }
    sprintf(str__, "check_sensor_address = %d \n\r --- \n\r", i);
    send_debugging_data(str__);
}


void OnOffSensorPower(GPIO_PinState state){
    HAL_GPIO_WritePin(PWR_ULTRASONIC_PORT, PWR_ULTRASONIC_PIN, state);
}
void I2CWriteBuffer(uint8_t I2C_ADDRESS, uint8_t *aTxBuffer, uint8_t TXBUFFERSIZE)
{
    /* -> Start the transmission process */
    /* While the I2C in reception process, user can transmit data through "aTxBuffer" buffer */
    while(HAL_I2C_Master_Transmit(&i2c_ultralsonic_handel, (uint16_t)I2C_ADDRESS<<1, (uint8_t*)aTxBuffer, (uint16_t)TXBUFFERSIZE, (uint32_t)1000)!= HAL_OK)
    {
        /*
         * Error_Handler() function is called when Timeout error occurs.
         * When Acknowledge failure occurs (Slave don't acknowledge it's address)
         * Master restarts communication
         */

        if (HAL_I2C_GetError(&i2c_ultralsonic_handel) != HAL_I2C_ERROR_AF)
        {
            //DEBUG(3, "In I2C::WriteBuffer -> error");
            //Error_Handler(3);
        }

    }

    /* -> Wait for the end of the transfer */
    /* Before starting a new communication transfer, you need to check the current
     * state of the peripheral; if it’s busy you need to wait for the end of current
     * transfer before starting a new one.
     * For simplicity reasons, this example is just waiting till the end of the
     * transfer, but application may perform other tasks while transfer operation
     * is ongoing.
     */
    while (HAL_I2C_GetState(&i2c_ultralsonic_handel) != HAL_I2C_STATE_READY)
    {
    }
}
void I2CReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE)
{
    /* -> Lets ask for register's address */
    //WriteBuffer(I2C_ADDRESS, &RegAddr, 1);

    /* -> Put I2C peripheral in reception process */
    while(HAL_I2C_Master_Receive(&i2c_ultralsonic_handel, (uint16_t)I2C_ADDRESS<<1, aRxBuffer, (uint16_t)RXBUFFERSIZE, (uint32_t)1000) != HAL_OK)
    {
        /* Error_Handler() function is called when Timeout error occurs.
         * When Acknowledg".\Objects\robotic_gripper.axf" - 3 Error(s), 5 Warning(s).e failure occurs (Slave don't acknowledge it's address)
         * Master restarts communication
         */
        if (HAL_I2C_GetError(&i2c_ultralsonic_handel) != HAL_I2C_ERROR_AF)
        {
            //DEBUG(3, "In I2C::WriteBuffer -> error");
            //Error_Handler(4);
        }
    }

    /* -> Wait for the end of the transfer */
    /* Before starting a new communication transfer, you need to check the current
     * state of the peripheral; if it’s busy you need to wait for the end of current
     * transfer before starting a new one.
     * For simplicity reasons, this example is just waiting till the end of the
     * transfer, but application may perform other tasks while transfer operation
     * is ongoing.
     **/
    while (HAL_I2C_GetState(&i2c_ultralsonic_handel) != HAL_I2C_STATE_READY)
    {
    }
}
