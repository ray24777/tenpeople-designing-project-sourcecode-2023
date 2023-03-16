## Machine_Arm

#### 测速用代码（arduino）：

```c
/* This example Arduino Sketch controls the complete rotation of
 *  SG995 Servo motor by using its PWM and Pulse width modulation technique
 */

#include <Servo.h> // include servo library to use its related functions
#define Servo_PWM 6 // A descriptive name for D6 pin of Arduino to provide PWM signal
Servo MG995_Servo;  // Define an instance of of Servo with the name of "MG995_Servo"
  

void setup() {
  MG995_Servo.attach(Servo_PWM);  // Connect D6 of Arduino with PWM signal pin of servo motor

}

void loop() {
  MG995_Servo.write(0); //Turn clockwise at high speed
  delay(3000);
  MG995_Servo.detach();//Stop. You can use deatch function or use write(x), as x is the middle of 0-180 which is 90, but some lack of precision may change this value
  delay(2000);
  MG995_Servo.attach(Servo_PWM);//Always use attach function after detach to re-connect your servo with the board
  // MG995_Servo.write(180);
  // delay(3000);
  // MG995_Servo.detach();//Stop
  // delay(2000);
  // MG995_Servo.attach(Servo_PWM);

      
}
```



#### 机械设计 v1:

<img src="pic\image-20230316100444417.png" alt="image-20230316100444417" style="zoom:33%;" /><img src="pic\image-20230316100512331.png" alt="image-20230316100512331" style="zoom:33%;" />

舵机——机械臂链接件

通过舵机圆盘，用螺丝链接



<img src="pic\image-20230316100620498.png" alt="image-20230316100620498" style="zoom:33%;" /><img src="pic\image-20230316100607029.png" alt="image-20230316100607029" style="zoom:33%;" />

机械臂传动杆，左边为机械臂圆盘，3d 打印时组合；右边连接撞球框，且为通过螺丝螺母连接



<img src="pic\image-20230316100751081.png" alt="image-20230316100751081" style="zoom:33%;" /><img src="pic\image-20230316100806749.png" alt="image-20230316100806749" style="zoom:33%;" />

球筐与连接件，以网球尺寸前后分别冗余 3mm，避免卡住和保证无阻尼



#### 整体效果

![image-20230316100909854](pic\image-20230316100909854.png)



#### 计划：

舵机旋转通过传动杆带动球筐旋转 180 度，理想情况下球会在前方落下



#### 实体：

![IMG_20230316_101011](pic\IMG_20230316_101011.jpg)



#### 第一版实现情况：

优点：

​	螺丝打孔无问题，连接稳固

​	网球实现无摩擦，不会卡住

​	总体重量较轻，无负担



缺点：

​	网球大约 90 度后会落出，落点太近

​	传动杆硬度不足



解决方案：

​	重新设计球筐

​	传动杆填充增加



**TODO**

- [ ] 待部件到了连接舵机实现测试
- [ ] 第二版 3D 打印