/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::USB:Device
 * Copyright (c) 2004-2014 ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    HID.c
 * Purpose: USB Device Human Interface Device example program
 *----------------------------------------------------------------------------*/

#include <string.h>
#include <stdio.h>
#include <cmsis_os.h>
#include "main.h"
#include "uart_debug.h"
#include "protocol.h"
#define debugOverUART 1


uint8_t receive_buffer[512];

/* UART init function */
void uart_pc_init(void)
{
    usart1_pc_handel.Instance = USART1;
    usart1_pc_handel.Init.BaudRate = 57600;
    usart1_pc_handel.Init.WordLength = UART_WORDLENGTH_8B;
    usart1_pc_handel.Init.StopBits = UART_STOPBITS_1;
    usart1_pc_handel.Init.Parity = UART_PARITY_NONE;
    usart1_pc_handel.Init.Mode = UART_MODE_TX_RX;
    usart1_pc_handel.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    usart1_pc_handel.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&usart1_pc_handel);
	
}

void send_response_data(uint8_t *str_, uint16_t len)
{
	HAL_UART_Transmit_IT(&usart1_pc_handel, (uint8_t *)str_, len);
}

//------------------------------------------------------------------------------
// @ Function: Printf_Debug()
// @ Purpose: this function is used to print the buffer into console as a hexa
//		table, it is use to debug.
// @ Inputs:
//			- *data: the data buffer to print.
//			- size: the number of bytes to print
// @ Return: no return
//------------------------------------------------------------------------------
void Printf_Debug(unsigned char *data,int size)
{
    unsigned int i,j = 0;
    unsigned int temp;
    char str_[16];
    char str_2[2048];

    //return;
    
    sprintf(str_2,"\n\r Size: %d \n\r", size);
    if (size <= 16)
    {
        for (i = 0;i < size;i++)
        {
            sprintf(str_, "%02X ",data[i]);
            strcat(str_2, str_);
        }
        for (i = 0;i < 16 - size;i++)
        {
            strcat(str_2, "   ");
        }
        strcat(str_2, "	");
        for (i = 0;i < size;i++)
        {
            if(data[i] <= 31)
            {
                strcat(str_2, ".");
            }
            else
            {
                sprintf(str_, "%c",data[i]);
                strcat(str_2, str_);
            }
        }
    }
    else
    {
        temp = size - size % 16;
        while (j < temp)
        {
            for (i = 0;i < 16;i++)
            {
                sprintf(str_, "%02X ",data[j + i]);
                strcat(str_2, str_);
            }
            strcat(str_2, "	");
            for (i = 0;i < 16;i++)
            {
                if(data[j + i] <= 31)
                {
                    strcat(str_2, ".");
                }
                else
                {
                    sprintf(str_, "%c",data[j + i]);
                    strcat(str_2, str_);
                }
            }
            strcat(str_2,"\n\r");
            j += 16;
        }

        for (i = 0;i < (size % 16);i++)
        {
            sprintf(str_, "%02X ",data[j + i]);
            strcat(str_2, str_);
        }
        for (i = 0;i < (16 - size % 16);i++)
        {
            strcat(str_2, "   ");
        }
        strcat(str_2, "	");
        for (i = 0;i < (size % 16);i++)
        {
            if (data[j + i] <= 31)
            {
                strcat(str_2, ".");
            }
            else
            {
                sprintf(str_, "%c",data[j + i]);
                strcat(str_2, str_);
            }
        }
    }
    strcat(str_2,"\n\n\r \0");
//    debug(str_2);
}

void uart_debug_init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    uart_debug.Instance = UART5;
    uart_debug.Init.BaudRate = 57600;
    uart_debug.Init.WordLength = UART_WORDLENGTH_8B;
    uart_debug.Init.StopBits = UART_STOPBITS_1;
    uart_debug.Init.Parity = UART_PARITY_NONE;
    uart_debug.Init.Mode = UART_MODE_TX_RX;
    uart_debug.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_debug.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&uart_debug);
}

void send_debugging_data(char *str_)
{
    if (debugOverUART)
	{
		HAL_UART_Transmit_IT(&uart_debug, (uint8_t *)str_, strlen(str_));
		osDelay(10);
//		HAL_UART_Transmit(&uart_debug, (uint8_t *)str_, strlen(str_),10);
	}
}
