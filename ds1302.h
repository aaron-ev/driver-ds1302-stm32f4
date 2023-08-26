/**
 ******************************************************************************
 * @file         ds1302.c
 * @author       Aaron Escoboza
 * @brief        header file for DS1302 RTC driver
 * @link         GitGub : https://github.com/aaron-ev
 ******************************************************************************
 */
#ifndef __DS1302__H
#define __DS1302__H

#include "stm32f4xx.h"
#include "stdint.h"

/************* Configuration section **************/
#define DS1302_GPIO_PORT            GPIOB
#define DS1302_PIN_SCLK             GPIO_PIN_7
#define DS1302_PIN_SDA              GPIO_PIN_6
#define DS1302_PIN_RST              GPIO_PIN_5
/************* Configuration section **************/

/**
 * Enumeration for days of the week
 */
typedef enum
{
    MONDAY,
    TUESDAY,
    WEDNESDAY,
    THURSDAY,
    FRIDAY,
    SATURDAY,
    SUNDAY
}Day;

/**
 * Structure for holding time data
 */
typedef struct
{
    Day day;
    uint8_t year;
    uint8_t month;
    uint8_t date;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
}Time;

/* Initialize DS1302 device */
void ds1302_init(void);

/* APIs for calendar*/
void ds1302_get_time(Time *time);
void ds1302_setTime(Time *time);

/* APIs for RAM*/
void ds1302_clear_ram(void);
void ds1302_write_ram(uint8_t addr, uint8_t data);
uint8_t ds1302_read_ram(uint8_t addr);

#endif
