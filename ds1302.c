/**
 ******************************************************************************
 * @file         ds1302.c
 * @author       Aaron Escoboza
 * @brief        Source file for DS1302 RTC driver
 * @link         GitGub : https://github.com/aaron-ev
 ******************************************************************************
 */
#include "ds1302.h"

/* Definition of sizes */
#define DS1302_DATA_SIZE           (8)
#define DS1302_ADDR_SIZE           (8)
#define DS1302_PACKAGE_SIZE        (DS1302_DATA_SIZE + DS1302_ADDR_SIZE)
#define DS1302_RAM_ADDR_START      (0xC0)
#define DS1302_RAM_SIZE            (31)

/* Register definition according to the spec */
#define DS1302_REG_SEC              0x80
#define DS1302_REG_MIN              0x82
#define DS1302_REG_HOUR             0x84
#define DS1302_REG_DATE             0x86
#define DS1302_REG_MONTH            0x88
#define DS1302_REG_DAY              0x8a
#define DS1302_REG_YEAR             0x8c
#define DS1302_REG_CONTROL          0x8e

#define BCD_TO_DEC(val)            ((val/16*10) + (val%16))
#define DEC_TO_BCD(val)            ((val/10*16) + (val%10))

/**
 * @brief Initialize the DS1302 device
 *
 * @param void
 * @return void
 */
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

/**
 * @brief write a bit following serial communication specified in the spec
 *
 * @param bit you being written
 * @return void
 */
static void DS1302_writeBit(uint8_t bit)
{
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SDA, (bit == 1) ?  GPIO_PIN_SET :  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SCLK,  GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SCLK,  GPIO_PIN_RESET);
    HAL_Delay(1);
}

/**
 * @brief write a byte following serial communication specified in the spec
 *
 * @param addr where you want write to
 * @param data that will write to addr
 * @return void
 */
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
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SDA, GPIO_PIN_RESET);
}

/**
 * @brief read a byte following serial communication specified in the spec
 *
 * @param addr address you want to read from
 * @return byte read
 */
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
        HAL_GPIO_WritePin(DS1302_GPIO, DS1302_PIN_SCLK,  GPIO_PIN_SET);
        Hal_delay(1);
        HAL_GPIO_WritePin(DS1302_GPIO, DS1302_PIN_SCLK,  GPIO_PIN_RESET);
        Hal_delay(1);
        data >>= 1;
    }
    DS1302_setSDAasOutput();

    /* Go back to default state*/
    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_PIN_RST, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_GPIO, DS1302_PIN_SDA, GPIO_PIN_RESET);

    return data;
}

/**
 * @brief Get time in calendar registers
 *
 * @param *time pointer to Time structure
 * @return void
 */
void DS1302_getTime(Time *time)
{
    if (time == NULL)
        return;

    time->year = BCD_TO_DEC(DS1302_readByte(DS1302_REG_YEAR));
    time->month = BCD_TO_DEC(DS1302_readByte(DS1302_REG_MONTH));
    time->date = BCD_TO_DEC(DS1302_readByte(DS1302_REG_DATE));
    time->hour = BCD_TO_DEC(DS1302_readByte(DS1302_REG_HOUR));
    time->min = BCD_TO_DEC(DS1302_readByte(DS1302_REG_MIN));
    time->sec = BCD_TO_DEC(DS1302_readByte(DS1302_REG_SEC));
    time->day = BCD_TO_DEC(DS1302_readByte(DS1302_REG_DAY));
}

/**
 * @brief set time into calendar registers
 *
 * @param *time pointer to Time structure
 * @return void
 */
void DS1302_setTime(Time *time)
{

    if (time == NULL)
        return;

    /* Enable write by driving protected bit to 0*/
    DS1302_writeByte(DS1302_REG_CONTROL, 0);
    HAL_delay(1);

    /* Write time to into registers in BCD format*/
    DS1302_writeByte(DS1302_REG_YEAR, DEC_TO_BCD(time->year));
    DS1302_writeByte(DS1302_REG_MONTH, DEC_TO_BCD(time->month));
    DS1302_writeByte(DS1302_REG_DATE, DEC_TO_BCD(time->date));
    DS1302_writeByte(DS1302_REG_HOUR, DEC_TO_BCD(time->hour));
    DS1302_writeByte(DS1302_REG_MIN, DEC_TO_BCD(time->min));
    DS1302_writeByte(DS1302_REG_SEC, DEC_TO_BCD(time->sec));

    /* Disable write by driving protected bit to 1 */
    DS1302_writeByte(DS1302_REG_CONTROL,0x80);
    HAL_delay(1);
}

/**
 * @brief Write to RAM, valid addresses 0 - 30
 * @param addr address you want to write to
 * @param data data to write into the address
 * @return void
 */
void DS1302_writeToRAM(uint8_t addr, uint8_t data)
{
    /* Check for valid addr */
    if ( (addr >= 0) && (addr <= 30) )
        return;

    /* Enable write by driving protected bit to 0*/
    DS1302_writeByte(DS1302_REG_CONTROL, 0);
    HAL_delay(1);
    /* Write addresses for RAM are multiple of 2 */
    DS1302_writeByte(DS1302_RAM_ADDR_START + (2 * addr), data);
    /* Disable write by driving protected bit to 1 */
    DS1302_WriteByte(DS1302_REG_CONTROL,0x80);
    delayUS_DWT(1);
}

/**
 * @brief Read from RAM, valid addresses 0 - 30
 * @param addr address you want to read from
 * @return data read
 */
uint8_t DS1302_readFromRAM(uint8_t addr)
{
    /* Check for valid addr */
    if ( (addr >= 0) && (addr <= 30) )
        return;

    return DS1302_readByte(DS1302_RAM_ADDR_START + (2 * addr));
}

/**
 * @brief Clear the entire RAM addresses (0 - 30)
 * @param void
 * @return void
 */
void DS1302_clearRAM(void)
{
    int i;
    for (i = 0; i < DS1302_RAM_SIZE; i++)
    {
        DS1302_writeToRAM(i, 0);
    }
}
