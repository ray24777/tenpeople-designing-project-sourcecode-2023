# Wiring
Following the table for wiring.

## Nucleo-G474RE

### Power
|G474 Port  | PCB Port  |
|-----------|-----------|
|E5V |5V|
|GND |GND|

### GPIO

|G474 GPIO Port  | Output Device  | Output Device Port  |
|-----------|-----------|-----------|
|RESET|Push Button|GND|
|PC1 |Metal Servo|PWM|
|PC2 |Blue Servo A|PWM|
|PC3 |Blue Servo B|PWM|
|PC0 |Ultrasonic sensors (All)|Trig|
|PA0 |Ultrasonic sensor northeast|Echo |
|PA1 |Ultrasonic sensors southeast|Echo|
|PA11 |Ultrasonic sensors north|Echo|
|PC12(UART5) |HC12|RX|
|PD2 (UART5) |HC12|TX|
|PC10 (UART4) |H7|B15(RX)|
|PC11 (UART4) |H7|TX|
|PC4(USART1)  |ATK-MS901M|RX|
|PC5(USART1)  |ATK-MS901M|TX|
|PB10(USART3) |OpenMV|RX|
|PB11(USART3) |OpenMV|TX|

## H7

### GPIO

|H7 GPIO Port  | Output Device  | Output Device Port  |
|-----------|-----------|-----------|
|B15 |G474|PC10(TX)|
|A6|Motor Contorl 2nd layer|PWMA|
|A7|Motor Contorl 2nd layer|AIN2|
|C4|Motor Contorl 2nd layer|AIN1|
|5V **(ON PCB)**|Motor Contorl 2nd layer|STBY|
|C5|Motor Contorl 2nd layer|BIN1|
|B0|Motor Contorl 2nd layer|BIN2|
|B1|Motor Contorl 2nd layer|PWMB|
|--|Motor Contorl 2nd layer|5V|
|--|Motor Contorl 2nd layer|GND|
|--|Motor Contorl 2nd layer|ADC|
|B6|Motor Contorl 2nd layer|E1A|
|B7|Motor Contorl 2nd layer|E1B|
|E1|Motor Contorl 2nd layer|E2A|
|E0|Motor Contorl 2nd layer|E2B|
|A0|Motor Contorl 1st layer|PWMA|
|A2|Motor Contorl 1st layer|AIN2|
|A1|Motor Contorl 1st layer|AIN1|
|5V **(ON PCB)**|Motor Contorl 1st layer|STBY|
|A3|Motor Contorl 1st layer|BIN1|
|A4|Motor Contorl 1st layer|BIN2|
|A9|Motor Contorl 1st layer|PWMB|
|--|Motor Contorl 1st layer|5V|
|--|Motor Contorl 1st layer|GND|
|--|Motor Contorl 1st layer|ADC|
|D3|Motor Contorl 1st layer|E1A|
|D4|Motor Contorl 1st layer|E1B|
|D0|Motor Contorl 1st layer|E2A|
|C12|Motor Contorl 1st layer|E2B|
