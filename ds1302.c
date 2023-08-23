
#include "ds1302.h"

#define MAX_CMD_SIZSE           8

void DS1302_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = DS1302_PIN_SCLK | DS1302_PIN_SDA | DS1302_PIN_RST;
    GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(DS1302_GPIO_PORT, &GPIO_InitStructure);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_RST,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SCLK,  GPIO_PIN_RESET);
}

/* Sends an address or command */
static void DS1302_SendCmd(Cmd cmd) 
{
    uint8_t i;
    for (i = 0; i < MAX_CMD_SIZE; i ++) 
    {
        HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SDA, (cmd & 1) ?  GPIO_PIN_SET :  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK,  GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK,  GPIO_PIN_RESET);
        HAL_Delay(1);
        cmd >>= 1;
    }
}
