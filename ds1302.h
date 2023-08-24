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

#define DS1302_PIN_SCLK
#define DS1302_PIN_SDA
#define DS1302_PIN_RST
#define DS1302_GPIO_PORT

/* Initialize DS1302 device */
void DS1302_init(void);

/* APIs for RAM*/
void DS1302_readRam(void);
void DS1302_writeRam(void);
void DS1302_clearRam(void);

/* APIs for the calendar*/
void DS1302_readTime(void);
void DS1302_writeTime(void);

#endif
