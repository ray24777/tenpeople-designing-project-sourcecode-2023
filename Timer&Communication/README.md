## Timmer（第一阶段代码）

#### DS3231 module（stm32L432kc）：
#### Attention：Mbed OS5平台代码：

```c
#include "mbed.h"
#include "ds3231.h"
#include <cstdio>
#include <string>


#define TERM_CLEAR_EOL ("\033[K")

#define DS3231_WRITE_ADRS (0x68 << 1)
#define DS3231_READ_ADRS ((0x68 << 1) | 1)
#define STATUS_REG_ADRS (0x0F)
#define MAXIMUM_BUFFER_SIZE   32 

I2C i2c(D4, D5);

Serial hc(D1,D0); // tx, rx


void rtc_isr()
{
    char data[2];
    uint8_t status;
    data[0] = STATUS_REG_ADRS;

    //Set pointer in DS3231
    i2c.write(DS3231_WRITE_ADRS, data, 1);
    //Read status register
    i2c.read(DS3231_READ_ADRS, (data + 1), 1);
    //Save status register in status var
    status = data[1];
    //Clear data[1] for clearing flags in DS3231
    data[1] = 0;
    i2c.write(DS3231_WRITE_ADRS, data, 2);
}


int main()
{   
    hc.baud(9600);

    //DS3231 rtc
    Ds3231 rtc(D4, D5);
    
    ds3231_cntl_stat_t rtc_control_status;
    //Bit2 - !INT/SQW pin used for Interrupts
    //Bit1 - A2IE set to allow for Alarm2 interrupt
    //Bit0 - A1IE set to allow for Alarm1 interrupt
    rtc_control_status.control =  0x07;
    rtc_control_status.status = 0;
    
    //write control, control/status registers
    rtc.set_cntl_stat_reg(rtc_control_status);
    
    //Set time
    ds3231_time_t rtc_time;
    rtc_time.seconds = 30;
    rtc_time.minutes = 51;
    rtc_time.hours = 20;
    rtc_time.mode = false; //24Hr mode
    
    //write timming registers
    rtc.set_time(rtc_time);
    
    //Set date
    ds3231_calendar_t rtc_calendar;
    rtc_calendar.day = 2;
    rtc_calendar.date = 14;
    rtc_calendar.month = 3;
    rtc_calendar.year = 23;
    
    //write calendar registers
    rtc.set_calendar(rtc_calendar);

    
    char buffer[32];
    time_t epoch_time;

    char buf[MAXIMUM_BUFFER_SIZE]={0};
    char c;

    while(1){

       if (hc.getc()==1){
        
        epoch_time = rtc.get_epoch();
        printf("\nTeam number is 07 \n");
        printf("\nNow Time is = %s\n", ctime(&epoch_time));
 
        strftime(buffer, 32, "%I:%M %p\n", localtime(&epoch_time));

        }


        wait(5);
    
        
    
}
}

```

## Communication

#### HC12 module（stm32L432kc）：
```c
#include "mbed.h"

Serial hc(D1,D0); // tx, rx
int num=1;
AnalogIn control(A4);

int main() {
    hc.baud(9600);
   while(1){
    if(control>0.8){
    hc.putc(num);
    wait(5);
       }
    if(control<0.5){
    hc.putc(0);
    wait(5);
    }
   }
}
```
