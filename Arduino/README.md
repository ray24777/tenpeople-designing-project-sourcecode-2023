[TOC]



# **底盘**

**PID库有改动，请使用本库中附带的PID库**

## 基本过程

用STM32和主控（OpenMV）进行UART串口通信，通过接收到的信息对应的控制四个麦克纳姆轮的速度和方向

## **1.电机控制**

### 1) 输出控制

这几个引脚全是输出模式

两个IN1，IN2引脚为GPIO输出,控制麦轮方向；ENA引脚为PWM输出，控制麦轮速度

```c++
/*定义电机驱动接线引脚*/
int L1_IN1 = 4;     int L1_IN2 = 3;     int L1_ENA = 2;//左前轮
int R1_IN1 = 5;     int R1_IN2 = 6;     int R1_ENA = 7;//右前轮
int L2_IN1 = 10;    int L2_IN2 = 9;     int L2_ENA = 8;//左后轮
int R2_IN1 = 11;    int R2_IN2 = 12;    int R2_ENA = 13;//右后轮
```

```c++
 void setup()
{
  pinMode(L1_IN1, OUTPUT); pinMode(L1_IN2, OUTPUT); pinMode(L1_ENA, OUTPUT);
  pinMode(R1_IN1, OUTPUT); pinMode(R1_IN2, OUTPUT); pinMode(R1_ENA, OUTPUT);
  pinMode(L2_IN1, OUTPUT); pinMode(L2_IN2, OUTPUT); pinMode(L2_ENA, OUTPUT);
  pinMode(R2_IN1, OUTPUT); pinMode(R2_IN2, OUTPUT); pinMode(R2_ENA, OUTPUT);
 }
```

### 2) 反馈接收

这几个引脚全是输入模式

利用中断，50ms检测一次速度，每个电机霍尔编码器的上升沿计次，从而获取每个轮子的速度

其中变量V代表目标速度，V_NOW代表目标速度，V_ROS代表轮子转速

最后生成的是linear_x, linear_y, linear_w，分别代表前后实际速度（cm/s），左右实际速度（cm/s），逆时针转动角速度，计算过程利用了线性代数，具体计算过程参见网站

[(41条消息) ROS麦克纳姆轮底盘制作（上）_熊猫飞天的博客-CSDN博客](https://blog.csdn.net/crp997576280/article/details/102026459)

```c++
/*****底盘返回参数**********/
double K4_1 = 1.0 / (4.0 * 18.1); //底盘K值,X+Y=18.1cm
double linear_x = 0;
double linear_y = 10;
double linear_w = 0; //转动角速度

/*定义中断(霍尔返回值)引脚*/
int MotorLA1count=42; 
int MotorLA2count=43;//左前轮
int MotorRA1count=44;
int MotorRA2count=45;//右前轮
int MotorLB1count=30;
int MotorLB2count=31;//左后轮 
int MotorRB1count=34;
int MotorRB2count=35;//右后轮

volatile double motorLA = 0; //中断变量，左前轮脉冲计数
volatile double motorRA = 0; //中断变量，右前轮脉冲计数
volatile double motorLB = 0; //中断变量，左后轮脉冲计数
volatile double motorRB = 0; //中断变量，右后轮脉冲计数

double V_LA, V_LA_NOW, V_LAROS = 0; //左前轮速度 单位cm/s
double V_LB, V_LB_NOW, V_LBROS = 0; //左后轮速度 单位cm/s
double V_RA, V_RA_NOW, V_RAROS = 0; //右前轮速度 单位cm/s
double V_RB, V_RB_NOW, V_RBROS = 0; //右后轮速度 单位cm/s
```

```c++
void setup()
{
  pinMode(MotorLA1count, INPUT); pinMode(MotorLA2count, INPUT);
  pinMode(MotorRA1count, INPUT); pinMode(MotorRA2count, INPUT);
  pinMode(MotorLB1count, INPUT); pinMode(MotorLB2count, INPUT);
  pinMode(MotorRB1count, INPUT); pinMode(MotorRB2count, INPUT);
}
```

```c++
void Read_Moto_V()
{
  unsigned long nowtime = 0;
  nowtime = millis() + 50; //读50毫秒

  attachInterrupt(digitalPinToInterrupt(MotorLA1count), READ_ENCODER_LA, RISING); //左前轮
  //参数含义：第一个是中断引脚，第二个是中断时执行函数，第三个RISING代表上升沿执行中断
  attachInterrupt(digitalPinToInterrupt(MotorLB1count), READ_ENCODER_LB, RISING); //左后轮
  attachInterrupt(digitalPinToInterrupt(MotorRA1count), READ_ENCODER_RA, RISING); //右前轮
  attachInterrupt(digitalPinToInterrupt(MotorRB1count), READ_ENCODER_RB, RISING); //右后轮
  while (millis() < nowtime); //达到50毫秒关闭中断
  detachInterrupt(digitalPinToInterrupt(MotorLA1count)); //左前轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorLB1count)); //左后轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorRA1count)); //右前轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorRB1count)); //右后轮脉冲关中断计数

  // motorLA/390/0.05
  V_LA_NOW = ((motorLA / 390) * 7.5 * 3.141) / (0.05 * 1.414); //左前轮返回速度
  V_RA_NOW = ((motorRA / 390) * 7.5 * 3.141) / (0.05 * 1.414); //右前轮返回速度
  V_LB_NOW = ((motorLB / 390) * 7.5 * 3.141) / (0.05 * 1.414); //左后轮返回速度
  V_RB_NOW = ((motorRB / 390) * 7.5 * 3.141) / (0.05 * 1.414); //右后轮返回速度

  V_LAROS = linear_x - linear_y - K4_1 * linear_w;
  V_LBROS = linear_x + linear_y - K4_1 * linear_w;
  V_RBROS = linear_x - linear_y + K4_1 * linear_w;
  V_RAROS = linear_x + linear_y + K4_1 * linear_w;

  V_LA = V_LAROS / 1.414;
  V_RA = V_RAROS / 1.414;
  V_LB = V_LBROS / 1.414;
  V_RB = V_RBROS / 1.414;

  motorLA = motorLB = motorRA = motorRB = 0; //一个周期后计数置零
}
```

```c++
//这些函数是中断时触发函数，用于中断计数，正向转动加，反向转动减
void READ_ENCODER_LA()
{
  // motorLA++;
  if (digitalRead(MotorLA1count) == HIGH)
  {
    if (digitalRead(MotorLA2count) == LOW)
      motorLA++;  //根据另外一相电平判定方向
    else
      motorLA--;
  }

  else
  {
    if (digitalRead(MotorLA2count) == LOW)
      motorLA--; //根据另外一相电平判定方向
    else
      motorLA++;
  }

}
void READ_ENCODER_LB()
{
  // motorLB++;
  if (digitalRead(MotorLB1count) == HIGH)
  {
    if (digitalRead(MotorLB2count) == LOW)
      motorLB++;  //根据另外一相电平判定方向
    else
      motorLB--;
  }

  else
  {
    if (digitalRead(MotorLB2count) == LOW)
      motorLB--; //根据另外一相电平判定方向
    else
      motorLB++;
  }
}
void READ_ENCODER_RA()
{
  // motorRA++;
  if (digitalRead(MotorRA1count) == HIGH)
  {
    if (digitalRead(MotorRA2count) == LOW)
      motorRA++;  //根据另外一相电平判定方向
    else
      motorRA--;
  }

  else
  {
    if (digitalRead(MotorRA2count) == LOW)
      motorRA--; //根据另外一相电平判定方向
    else
      motorRA++;
  }
}
void READ_ENCODER_RB()
{
  // motorRB++;
  if (digitalRead(MotorRB1count) == HIGH)
  {
    if (digitalRead(MotorRB2count) == LOW)
      motorRB++;  //根据另外一相电平判定方向
    else
      motorRB--;
  }

  else
  {
    if (digitalRead(MotorRB2count) == LOW)
      motorRB--; //根据另外一相电平判定方向
    else
      motorRB++;
  }

}

```

## 2. PID调节

PID调节用于保证实际输出等于目标输出，具体原理参见网站

[一文搞懂PID控制算法 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/347372624)

我这里PID算法专门用了一个库，引用的时候用PID创建实例对象，输入参数即可

**示例：**

```c++
PID myPIDLA(&InputLA, &OutputLA, &SetpointLA, 0.1, 8, 0.1, DIRECT); 
//&Input代表输入（调整前），Output输出（调整后），Setpoint代表目标值，后面三个数字代表参参数大小，顺序:Ki,Kp,Kd，DIRECT代表正向调节
```



```c++
#include <PID_v1.h>

/*******初始化pid算法*******/
double InputLA, OutputLA, SetpointLA;
double InputLB, OutputLB, SetpointLB;
double InputRA, OutputRA, SetpointRA;
double InputRB, OutputRB, SetpointRB;

PID myPIDLA(&InputLA, &OutputLA, &SetpointLA, 0.1, 8, 0.1, DIRECT); //顺序:Ki,Kp,Kd
PID myPIDLB(&InputLB, &OutputLB, &SetpointLB, 0.1, 8, 0.1, DIRECT); //顺序:Ki,Kp,Kd
PID myPIDRA(&InputRA, &OutputRA, &SetpointRA, 0.1, 8, 0.1, DIRECT); //顺序:Ki,Kp,Kd
PID myPIDRB(&InputRB, &OutputRB, &SetpointRB, 0.1, 8, 0.1, DIRECT); //顺序:Ki,Kp,Kd
```

```c++
void setup()
{
  myPIDLA.SetMode(AUTOMATIC);//自动开始运行
  myPIDLB.SetMode(AUTOMATIC);
  myPIDRA.SetMode(AUTOMATIC);
  myPIDRB.SetMode(AUTOMATIC);
  myPIDLA.SetOutputLimits(-255, 255);//输出控制在-255到255之间，用于和PWM范围对应，负代表反向
  myPIDLB.SetOutputLimits(-255, 255);
  myPIDRA.SetOutputLimits(-255, 255);
  myPIDRB.SetOutputLimits(-255, 255);
  myPIDLA.SetSampleTime(50);//采样计算时间和电机采样时间控制一致
  myPIDLB.SetSampleTime(50);
  myPIDRA.SetSampleTime(50);
  myPIDRB.SetSampleTime(50);
}
```

```c++
void Read_Moto_V()
{
	SetpointLA = V_LA;
  SetpointLB = V_LB;
  SetpointRA = V_RA;
  SetpointRB = V_RB;

  InputLA = V_LA_NOW; InputRA = V_RA_NOW; InputLB = V_LB_NOW; InputRB = V_RB_NOW;
  myPIDLA.Compute(); myPIDLB.Compute(); myPIDRA.Compute(); myPIDRB.Compute();
  //compute让其开始运算
  
  if (int(OutputLA) >= 0)//正代表正向，负代表反向
  {
    digitalWrite(L1_IN1, LOW);
    digitalWrite(L1_IN2, HIGH);
    analogWrite(L1_ENA, (int)OutputLA);
  }
  else
  {
    digitalWrite(L1_IN1, HIGH);
    digitalWrite(L1_IN2, LOW);
    analogWrite(L1_ENA, -(int)OutputLA);
  }

  if (int(OutputLB) >= 0)
  {
    digitalWrite(L2_IN1, HIGH);
    digitalWrite(L2_IN2, LOW);
    analogWrite(L2_ENA, (int)OutputLB);
  }
  else
  {
    digitalWrite(L2_IN1, LOW);
    digitalWrite(L2_IN2, HIGH);
    analogWrite(L2_ENA, -(int)OutputLB);
  }

  if (int(OutputRA) >= 0)
  {
    digitalWrite(R1_IN1, HIGH);
    digitalWrite(R1_IN2, LOW);
    analogWrite(R1_ENA, (int)OutputRA);
  }
  else
  {
    digitalWrite(R1_IN1, LOW);
    digitalWrite(R1_IN2, HIGH);
    analogWrite(R1_ENA, -(int)OutputRA);
  }

  if (int(OutputRB) >= 0)
  {
    digitalWrite(R2_IN1, HIGH);
    digitalWrite(R2_IN2, LOW);
    analogWrite(R2_ENA, (int)OutputRB);
  }
  else
  {
    digitalWrite(R2_IN1, LOW);
    digitalWrite(R2_IN2, HIGH);
    analogWrite(R2_ENA, -(int)OutputRB);
  }
}
```

## 3. UART串口通信

UART串口通信用于接收上位机发送的消息，由10位数组成，按照3,3,4分段，其中每段的第一位是符号位，1代表反向，2代表正向，串口通信相关知识参考网站：

[(41条消息) Arduino串口通信_xiaoshihd的博客-CSDN博客](https://blog.csdn.net/xiaoshihd/article/details/108805015)

**eg. 接收1232341234**

**代表linear_x = -23, linear_y = 34, linear_w = -234**

```c++
void setup()
{
	Serial1.begin(115200);
}
```

```c++
//这段代码用于将每段的符号位提取，然后变为正负号
int JudgeNum1(int n) //用于判断接收参数的符号
{
  int sign = n / 100; // get the first digit
  int num = n % 100; // get the last two digits
  if (sign == 1) // if sign bit is 1, the number is negative
  {
    n = -n;
    n = n % 100;
  }
  else // if sign bit is 2, the number is positive
  {
    n = n;
    n = n % 100;
  }
  return n;
}

int JudgeNum2(int n) //用于判断接收参数的符号
{
  int sign = n / 1000; // get the first digit
  int num = n % 1000; // get the three two digits
  if (sign == 1) // if sign bit is 1, the number is negative
  {
    n = -n;
    n = n % 1000;
  }
  else // if sign bit is 2, the number is positive
  {
    n = n;
    n = n % 1000;
  }
  return n;
}
```

```c++
void Read_Moto_V()
{
  int a, b, c; // the two 3-digit numbers and one 4-digit number
  
  if (Serial1.available() > 0)
  {
    Serial1.println("####-------------------------------###");
    data = Serial1.parseInt(); //parseInt()函数用于读取串口整数
    int data43 = Serial1.parseInt();//读空白符，垃圾数据，不要删
    Serial1.print("data is : ");
    Serial1.println(data); 

    a = data / 10000000; // get the first 3 digits of n
    b = (data / 10000) % 1000; // get the second 3 digits of n
    c = data % 10000; // get the last 4 digits of n

    linear_x = JudgeNum1(a); // print a with sign
    linear_y = JudgeNum1(b); // print b with sign
    linear_w = JudgeNum2(c); // print c with sign

    Serial1.print("a is :");
    Serial1.println(linear_x);
    Serial1.print("b is :");
    Serial1.println(linear_y);
    Serial1.print("c is :");
    Serial1.println(linear_w);
  }

  Serial1.println("last loop");
  Serial1.println(linear_x);
  Serial1.println(linear_y);
  Serial1.println(linear_w);
}
```



# 附加

## Pid.h

```c++
#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.1.1

class PID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1

  //commonly used functions **************************************************************************
    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here
	
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(double, double,       // * overload for specifying proportional mode
                    double, int);         	  

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	double GetKp();						  // These functions query the pid for interal values.
	double GetKi();						  //  they were created mainly for the pid front-end,
	double GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

  private:
	void Initialize();
	
	double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	double dispKi;				//   format for display purposes
	double dispKd;				//
    
	double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the 
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	double outputSum, lastInput;

	unsigned long SampleTime;
	double outMin, outMax;
	bool inAuto, pOnE;
};
#endif


```

## Pid.cpp

```c++
/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_v1.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, POn);

    lastTime = millis()-SampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
    :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)
{

}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double input = *myInput;
      double error = *mySetpoint - input;
      double dInput = (input - lastInput);
      outputSum+= (ki * error);

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!pOnE) outputSum-= kp * dInput;

      if(outputSum > outMax) outputSum= outMax;
      else if(outputSum < outMin) outputSum= outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
	    double output;
      // if(pOnE) output = kp * error;
      // else output = 0;

      /*Compute Rest of PID Output*/
      output += outputSum - kd * dInput;

	    if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	    *myOutput = output;

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
	    return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd){
    SetTunings(Kp, Ki, Kd, pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(outputSum > outMax) outputSum= outMax;
	   else if(outputSum < outMin) outputSum= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}


```

