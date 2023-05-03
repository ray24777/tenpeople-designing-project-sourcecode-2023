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
|PC1 |Metal Servo|PWM|
|PC2 |Blue Servo A|PWM|
|PC3 |Blue Servo B|PWM|
|PC0 |Ultrasonic sensors (All)|Trig|
|PA0 |Ultrasonic sensor northeast|Echo |
|PA1 |Ultrasonic sensors southeast|Echo|
|PA11 |Ultrasonic sensors north|Echo|
|PC12 |HC12|RX|
|PD2  |HC12|TX|
|PC10 |H7|B15(RX)|
|PC11 |H7|TX|
|PC4  |ATK-MS901M|RX|
|PC5  |ATK-MS901M|TX|
|PB10 |OpenMV|RX|
|PB11 |OpenMV|TX|

## H7

### Power
|H7 Port  | PCB Port  |
|-----------|-----------|
|E5V |5V|
|GND |GND|

### GPIO

|H7 GPIO Port  | Output Device  | Output Device Port  |
|-----------|-----------|-----------|
|B15 |G474|PC10(TX)|
