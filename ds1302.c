/**
 ******************************************************************************
 * @file         ds1302.c
 * @author       Aaron Escoboza
 * @brief        Source file for DS1302 RTC driver
 * @link         GitGub : https://github.com/aaron-ev
 ******************************************************************************
 */

#include "ds1302.h"

#define DS1302_DATA_SIZE           (8)
#define DS1302_ADDR_SIZE           (8)
#define DS1302_PACKAGE_SIZE        (DS1302_DATA_SIZE + DS1302_ADDR_SIZE)

void DS1302_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = DS1302_PIN_SCLK | DS1302_PIN_SDA | DS1302_PIN_RST;
    GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(DS1302_GPIO_PORT, &GPIO_InitStructure);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_RST, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SCLK, GPIO_PIN_RESET);
}

void DS1302_writeBit(uint8_t bit)
{
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_SDA, (bit == 1) ?  GPIO_PIN_SET :  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_SCLK,  GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_SCLK,  GPIO_PIN_RESET);
    HAL_Delay(1);
}

static void DS1302_writeByte(uint8_t addr, uint8_t data)
{
    uint8_t i;
    uint16_t  package = (uint16_t)data << 8 | addr;

    /* RST high to start communication */
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_RST, GPIO_PIN_SET);

    /* write address/cmd and then the data */
    for (i = 0; i < DS1302_PACKAGE_SIZE; i ++) 
    {
        DS1302_writeBit(package & 1);
        package >>= 1;
    }

    /* Go back to default state*/
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_RST, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_SDA, GPIO_PIN_RESET);
}

static void DS1302_setSDAasInput(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = DS1302_PIN_SDA;
    GPIO_InitStructure.Mode =  GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(DS1302_GPIO_PORT, &GPIO_InitStructure);	
}

static void DS1302_setSDAasOutput(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = DS1302_PIN_SDA;
    GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(DS1302_GPIO_PORT, &GPIO_InitStructure);
}

static uint8_t DS1302_readByte(uint8_t addr)
{
    uint8_t i; 
    uint8_t data = 0;

    /* RST high to start communication */
    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_PIN_RST, GPIO_PIN_SET);

    /* Make sure LSB bit is high for read */
    addr |= 0x1;
    DS1302_writeByte(addr);

    DS1302_setSDAasInput();
    for (i = 0; i < DS1302_DATA_SIZE; i ++) 
    {
        if (HAL_GPIO_ReadPin(DS1302_GPIO, DS1302_PIN_SDA))
        {
            data |= 0x80;
        }
        HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK,  GPIO_PIN_SET);
        Hal_delay(1);
        HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK,  GPIO_PIN_RESET);
        Hal_delay(1);
        data >>= 1;
    }
    DS1302_setSDAasOutput();

    /* Go back to default state*/
    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_PIN_RST, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SDA, GPIO_PIN_RESET);

    return data;
}
