
#ifndef __ DS1302__
#define __ DS1302__

#include "stm32f4xx.h"

#define DS1302_PIN_SCLK
#define DS1302_PIN_SDA
#define DS1302_PIN_RST

typedef enum {


} Cmd;

void DS1302_init(void);
void DS1302_send_cmd(Cmd cmd);

#endif
