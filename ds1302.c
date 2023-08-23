
#include "ds1302.h"

#define MAX_DATA_SIZE           8

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


void DS1302_sendBit(uint8_t bit)
{
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_SDA, (bit == 1) ?  GPIO_PIN_SET :  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_SCLK,  GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_SCLK,  GPIO_PIN_RESET);
    HAL_Delay(1);
}

#define BYTE_SIZE           8

void DS1302_writeByte(uint8_t data)
{
    for (i = 0; i < BYTE_SIZE; i ++) 
    {
        DS1302_sendBit(data & 1);
        data >>= 1;
    }
}

static void DS1302_write(uint8_t addr, uint8_t data)
{
    uint8_t i;

    /* RST high to start communication */
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_RST, GPIO_PIN_SET);
    /* Send address/cmd */
    DS1302_writeByte(addr);
    /* Send data */
    DS1302_writeByte(data);
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

static uint8_t DS1302_readByte(void)
{
    uint8_t i;
    uint8_t data = 0;

    for (i = 0; i < 8; i ++) 
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
    return data;
}

static uint8_t DS1302_read(uint8_t addr)
{
    uint8_t data;

    /* RST high to start communication */
    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_PIN_RST, GPIO_PIN_SET);
    /* Make sure LSB bit is high for read */
    addr |= 0x1;
    DS1302_writeByte(addr);

    DS1302_setSDAasInput();
    data = DS1302_readByte();
    DS1302_setSDAasOutput();

    /* Go back to default state*/
    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_PIN_RST, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SDA, GPIO_PIN_RESET);

    return data;
}
