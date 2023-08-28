# Driver for real time clock DS1202 based on STM32F4xx using HAL layer.
# What is DS1302 ?

The DS1302 trickle-charge timekeeping chip contains a real-time clock/calendar and 31 bytes of static RAM. It communicates with a microprocessor via a simple serial interface. The real-time clock/calendar provides seconds, minutes, hours, day, date, month, and year information.

![ds1302 block diagram](/docs/img/blockDiagram.png)

Source: [DS1302 device](https://www.analog.com/en/products/ds1302.html#:~:text=The%20DS1302%20trickle%2Dcharge%20timekeeping,%2C%20month%2C%20and%20year%20information.)

# Configuration 

The first thing you need to do is changing the current microcontroller settings 
based on your needs in ds1302.h under configuration
section. The 3 lines should be on the same GPIO port.

![configuration](/docs/img/configuration.png)

# Example

The following pice of code shows you how to:

* initialize the device lines (DATA, CLK and RESET)
* Set a specific time
* Get current time
* Write to specific address in RAM
* Read entire RAM
* Clear entire RAM

```c

#include "ds1302.h"

int main(void)
{
    Time_s time = {0};

    /* Initialize microcontroller configuration */
    ds1302_init();

    /* Setting Sunday 30/02/2023 8:1:0 PM */
    time.day = SUNDAY;
    time.year = 23;
    time.month = 2;
    time.date = 30;
    time.clockSystem = DS1302_CLK_SYSTEM_12;
    time.clockPeriod = DS1302_CLK_PM_PERIOD;
    time.hour = 8;
    time.min = 1;
    time.sec = 0;
    ds1302_set_time(&time);

    /*Getting current time*/
    Time_s newTime = {0};
    ds1302_get_time(&newTime); 

    /* Writing 10 to address 8 in RAM */
    ds1302_write_ram(8, 10);

    /*Reading the entire RAM, 0 - 30 valid addresses */
    int i;
    char ram[DS1302_RAM_SIZE];
    for( i = 0; i < DS1302_RAM_SIZE; i++)
    {
        ram[i] = ds1302_read_ram(i);
    }

    /* Clearing the entire RAM */
    ds1302_clear_ram();

    return 0;
}
```
