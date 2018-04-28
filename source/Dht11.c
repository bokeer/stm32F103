/**
  ******************************************************************************
  * @file    Dht11.h
  * @author  bokeer
  * @version V1.0
  * @date    11-March-2018
  * @brief   This file contains all the functions prototypes for the dht11 temperature	
  *          and humidity sensor.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
#include "Dht11.h"
#include "delay.h"

#define dht11_port GPIOB
#define dht11_pin GPIO_Pin_10
#define dht11_port_colck RCC_APB2Periph_GPIOB

void dht11_gpio_input(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(dht11_port_colck, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = dht11_pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //????
    
    GPIO_Init(dht11_port,&GPIO_InitStructure);
}

void dht11_gpio_output(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(dht11_port_colck, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = dht11_pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP; 
    
    GPIO_Init(dht11_port,&GPIO_InitStructure);
}
void dht11_reset(void)
{
    
    dht11_gpio_output();
    GPIO_ResetBits(dht11_port, dht11_pin);
    delay_us(19000);
    GPIO_SetBits(dht11_port, dht11_pin);
    delay_us(30);
    dht11_gpio_input();
}
u16 dht11_scan(void)
{
    return GPIO_ReadInputDataBit(dht11_port, dht11_pin);
}
u16 dht11_read_bit(void)
{
    while (dht11_scan() == RESET);
    delay_us(40);
    if (dht11_scan() == SET)
    {
        while (dht11_scan() == SET);
        return 1;
    }
    else
    {
        return 0;
    }
}
u16 dht11_read_byte(void)
{
    u16 i;
    u16 data = 0;
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        data |= dht11_read_bit();
    }
    return data;
}
u16 dht11_read_data(u8 buffer[5])
{
    u16 i = 0;
    
    dht11_reset();
    if (dht11_scan() == RESET)
    {
        //检测到DHT11响应
        while (dht11_scan() == RESET);
        while (dht11_scan() == SET);
        for (i = 0; i < 5; i++)
        {
            buffer[i] = dht11_read_byte();
        }
        
        while (dht11_scan() == RESET);
        dht11_gpio_output();
        GPIO_SetBits(dht11_port, dht11_pin);
        
        u8 checksum = buffer[0] + buffer[1] + buffer[2] + buffer[3];
        if (checksum != buffer[4])
        {
            // checksum error
            return 1;
        }
    }
    
    return 0;
}
