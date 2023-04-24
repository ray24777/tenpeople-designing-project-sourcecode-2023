#include <PID_v1.h>
#include <stdio.h>
// #include <HardwareSerial.h>

HardwareSerial Serial1(PB15, PB14);

// void setRx(uint32_t _rx);
// void setTx(uint32_t _tx);
// void setRx(PB14 _rx);
// void setTx(PB15 _tx);

/*******初始化pid算法*******/
double InputLA, OutputLA, SetpointLA;
double InputLB, OutputLB, SetpointLB;
double InputRA, OutputRA, SetpointRA;
double InputRB, OutputRB, SetpointRB;

PID myPIDLA(&InputLA, &OutputLA, &SetpointLA, 0.1, 8, 0.1, DIRECT); // 顺序:Ki,Kp,Kd
PID myPIDLB(&InputLB, &OutputLB, &SetpointLB, 0.1, 8, 0.1, DIRECT); // 顺序:Ki,Kp,Kd
PID myPIDRA(&InputRA, &OutputRA, &SetpointRA, 0.1, 8, 0.1, DIRECT); // 顺序:Ki,Kp,Kd
PID myPIDRB(&InputRB, &OutputRB, &SetpointRB, 0.1, 8, 0.1, DIRECT); // 顺序:Ki,Kp,Kd

/*定义电机驱动接线引脚*/
int L1_IN1 = PA2;
int L1_IN2 = PA1;
int L1_ENA = PA0; // 左前轮
int R1_IN1 = PA3;
int R1_IN2 = PA4;
int R1_ENA = PA9; // 右前轮
// int R1_IN1 = PA3;
// int R1_IN2 = PA4;
// int R1_ENA = PA5; // 右前轮
int L2_IN1 = PC4;
int L2_IN2 = PA7;
int L2_ENA = PA6; // 左后轮
int R2_IN1 = PC5;
int R2_IN2 = PB0;
int R2_ENA = PB1; // 右后轮

/*定义中断(霍尔返回值)引脚*/
int MotorLA1count = PD3;
int MotorLA2count = PD4; // 左前轮
// int MotorRA1count = PD1;
// int MotorRA2count = PD2; // 右前轮
int MotorLB1count = PB6;
int MotorLB2count = PB7; // 左后轮
// int MotorRB1count = PB8;
// int MotorRB2count = PB9; // 右后轮

int MotorRA1count = PC12;
int MotorRA2count = PD0; // 右前轮

int MotorRB1count = PE0;
int MotorRB2count = PE1; // 右后轮

volatile double motorLA = 0; // 中断变量，左前轮脉冲计数
volatile double motorRA = 0; // 中断变量，右前轮脉冲计数
volatile double motorLB = 0; // 中断变量，左后轮脉冲计数
volatile double motorRB = 0; // 中断变量，右后轮脉冲计数

double V_LA, V_LA_NOW, V_LAROS = 0; // 左前轮速度 单位cm/s
double V_LB, V_LB_NOW, V_LBROS = 0; // 左后轮速度 单位cm/s
double V_RA, V_RA_NOW, V_RAROS = 0; // 右前轮速度 单位cm/s
double V_RB, V_RB_NOW, V_RBROS = 0; // 右后轮速度 单位cm/s

/*****底盘返回参数**********/
double K4_1 = 1.0 / (4.0 * 18.1); // 底盘K值,X+Y=18.1cm
double linear_x = 0;
double linear_y = 10;
double linear_w = 0; // 转动角速度
long long data = 2002002000;
// long long data = 2202001000;
// SoftwareSerial1 softSerial1(PC_0,PC_1);

void setup()
{
  pinMode(L1_IN1, OUTPUT);
  pinMode(L1_IN2, OUTPUT);
  pinMode(L1_ENA, OUTPUT);
  pinMode(R1_IN1, OUTPUT);
  pinMode(R1_IN2, OUTPUT);
  pinMode(R1_ENA, OUTPUT);
  pinMode(L2_IN1, OUTPUT);
  pinMode(L2_IN2, OUTPUT);
  pinMode(L2_ENA, OUTPUT);
  pinMode(R2_IN1, OUTPUT);
  pinMode(R2_IN2, OUTPUT);
  pinMode(R2_ENA, OUTPUT);
  pinMode(MotorLA1count, INPUT);
  pinMode(MotorLA2count, INPUT);
  pinMode(MotorRA1count, INPUT);
  pinMode(MotorRA2count, INPUT);
  pinMode(MotorLB1count, INPUT);
  pinMode(MotorLB2count, INPUT);
  pinMode(MotorRB1count, INPUT);
  pinMode(MotorRB2count, INPUT);

  myPIDLA.SetMode(AUTOMATIC);
  myPIDLB.SetMode(AUTOMATIC);
  myPIDRA.SetMode(AUTOMATIC);
  myPIDRB.SetMode(AUTOMATIC);
  myPIDLA.SetOutputLimits(-255, 255);
  myPIDLB.SetOutputLimits(-255, 255);
  myPIDRA.SetOutputLimits(-255, 255);
  myPIDRB.SetOutputLimits(-255, 255);
  myPIDLA.SetSampleTime(50);
  myPIDLB.SetSampleTime(50);
  myPIDRA.SetSampleTime(50);
  myPIDRB.SetSampleTime(50);

  Serial1.begin(115200);
}

void loop()
{
  Read_Moto_V();
}

int JudgeNum1(int n) // 用于判断接收参数的符号
{
  int sign = n / 100; // get the first digit
  int num = n % 100;  // get the last two digits
  if (sign == 1)      // if sign bit is 1, the number is negative
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

int JudgeNum2(int n) // 用于判断接收参数的符号
{
  int sign = n / 1000; // get the first digit
  int num = n % 1000;  // get the three two digits
  if (sign == 1)       // if sign bit is 1, the number is negative
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

void L1_forward(int sp) // 左前轮前进
{
  digitalWrite(L1_IN1, LOW);
  digitalWrite(L1_IN2, HIGH);
  analogWrite(L1_ENA, sp);
}
void R1_forward(int sp) // 右前轮前进
{
  digitalWrite(R1_IN1, HIGH);
  digitalWrite(R1_IN2, LOW);
  analogWrite(R1_ENA, sp);
}
void L2_forward(int sp) // 左后轮前进
{
  digitalWrite(L2_IN1, HIGH);
  digitalWrite(L2_IN2, LOW);
  analogWrite(L2_ENA, sp);
}
void R2_forward(int sp) // 右后轮前进
{
  digitalWrite(R2_IN1, HIGH);
  digitalWrite(R2_IN2, LOW);
  analogWrite(R2_ENA, sp);
}
void allstop()
{
  digitalWrite(L1_IN1, LOW);
  digitalWrite(L1_IN2, LOW);
  digitalWrite(R1_IN1, LOW);
  digitalWrite(R1_IN2, LOW);
  digitalWrite(L2_IN1, LOW);
  digitalWrite(L2_IN2, LOW);
  digitalWrite(R2_IN1, LOW);
  digitalWrite(R2_IN2, LOW);
}
void L1_backward(int sp) // 左前轮后退
{
  digitalWrite(L1_IN1, HIGH);
  digitalWrite(L1_IN2, LOW);
  analogWrite(L1_ENA, sp);
}
void R1_backward(int sp) // 右前轮后退
{
  digitalWrite(R1_IN1, LOW);
  digitalWrite(R1_IN2, HIGH);
  analogWrite(R1_ENA, sp);
}
void L2_backward(int sp) // 左后轮后退
{
  digitalWrite(L2_IN1, LOW);
  digitalWrite(L2_IN2, HIGH);
  analogWrite(L2_ENA, sp);
}
void R2_backward(int sp) // 右后轮后退
{
  digitalWrite(R2_IN1, LOW);
  digitalWrite(R2_IN2, HIGH);
  analogWrite(R2_ENA, sp);
}

void Forward(int sp) // 前进函数封装
{
  L1_forward(sp);
  R1_forward(sp);
  L2_forward(sp);
  R2_forward(sp);
}

void Backward(int sp) // 返回函数封装
{
  L1_backward(sp);
  R1_backward(sp);
  L2_backward(sp);
  R2_backward(sp);
}

void Left(int sp)
{
  L1_backward(sp);
  R1_forward(sp);
  L2_forward(sp);
  R2_backward(sp);
}

void Right(int sp)
{
  L1_forward(sp);
  R1_backward(sp);
  L2_backward(sp);
  R2_forward(sp);
}

void Turn_Left(int sp)
{
  L1_backward(sp);
  R1_forward(sp);
  L2_backward(sp);
  R2_forward(sp);
}

void Turn_Right(int sp)
{
  L1_forward(sp);
  R1_backward(sp);
  L2_forward(sp);
  R2_backward(sp);
}

void Read_Moto_V()
{

  unsigned long nowtime = 0;
  //  unsigned long nowtime_read = 0;
  nowtime = millis() + 50; // 读50毫秒

  attachInterrupt(digitalPinToInterrupt(MotorLA1count), READ_ENCODER_LA, RISING); // 左前轮
  attachInterrupt(digitalPinToInterrupt(MotorLB1count), READ_ENCODER_LB, RISING); // 左后轮
  attachInterrupt(digitalPinToInterrupt(MotorRA1count), READ_ENCODER_RA, RISING); // 右前轮
  attachInterrupt(digitalPinToInterrupt(MotorRB1count), READ_ENCODER_RB, RISING); // 右后轮
  while (millis() < nowtime)
    ;           
  // delay(80);                                         // 达到50毫秒关闭中断
  detachInterrupt(digitalPinToInterrupt(MotorLA1count)); // 左前轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorLB1count)); // 左后轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorRA1count)); // 右前轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorRB1count)); // 右后轮脉冲关中断计数

  int a, b, c; // the two 3-digit numbers and one 4-digit number

  if (Serial1.available() > 0)
  {
    // Serial1.println("####-------------------------------###");
    while (1)
    {
      delay(50);
      if (Serial1.read() == 0x90)
      {
        data = 0;
        break;
      }
    }
    //    nowtime_read = millis() + 20; //读20毫秒
    for (int i = 0; i < 10; i++)
    {
      data = Serial1.read() + 10 * data;
    }
  }

  //    int data43 = Serial1.parseInt();//读空白符，垃圾数据，不要删
  //    Serial1.print("data is : ");
  //    Serial1.println(data);

  a = data / 10000000;       // get the first 3 digits of n
  b = (data / 10000) % 1000; // get the second 3 digits of n
  c = data % 10000;          // get the last 4 digits of n

  linear_x = JudgeNum1(a); // print a with sign
  linear_y = JudgeNum1(b); // print b with sign
  linear_w = JudgeNum2(c); // print c with sign

  // Serial1.print("a is :");
  // Serial1.println(linear_x);
  // Serial1.print("b is :");
  // Serial1.println(linear_y);
  // Serial1.print("c is :");
  // Serial1.println(linear_w);

  // Serial1.println("last loop");
  // Serial1.println(linear_x);
  // Serial1.println(linear_y);
  // Serial1.println(linear_w);

  // motorLA/390/0.05
  V_LA_NOW = ((motorLA / 390) * 7.5 * 3.141) / (0.05 * 1.414); // 左前轮返回速度
  V_RA_NOW = ((motorRA / 390) * 7.5 * 3.141) / (0.05 * 1.414); // 右前轮返回速度
  V_LB_NOW = ((motorLB / 390) * 7.5 * 3.141) / (0.05 * 1.414); // 左后轮返回速度
  V_RB_NOW = ((motorRB / 390) * 7.5 * 3.141) / (0.05 * 1.414); // 右后轮返回速度

  // 测试用
  // V_RB_NOW = (V_LB_NOW + V_RA_NOW + V_LA_NOW)/3;
  // 测试用 

  V_LAROS = linear_x - linear_y - K4_1 * linear_w;
  V_LBROS = linear_x + linear_y - K4_1 * linear_w;
  V_RBROS = linear_x - linear_y + K4_1 * linear_w;
  V_RAROS = linear_x + linear_y + K4_1 * linear_w;

  V_LA = V_LAROS / 1.414;
  V_RA = V_RAROS / 1.414;
  V_LB = V_LBROS / 1.414;
  V_RB = V_RBROS / 1.414;

  SetpointLA = V_LA;
  SetpointLB = V_LB;
  SetpointRA = V_RA;
  SetpointRB = V_RB;

  // V_RB_NOW = (V_LB_NOW + V_LA_NOW + V_RA_NOW)/3;

  InputLA = V_LA_NOW;
  InputRA = V_RA_NOW;
  InputLB = V_LB_NOW;
  InputRB = V_RB_NOW;

  
  // Serial1.print("V_LA_NOW is :");
  // Serial1.println(V_LA_NOW);
  // Serial1.print("V_RA_NOW is :");
  // Serial1.println(V_RA_NOW);
  // Serial1.print("V_LB_NOW is :");
  // Serial1.println(V_LB_NOW);
  // Serial1.print("V_RB_NOW is :");
  // Serial1.println(V_RB_NOW);

  myPIDRA.Compute();
  myPIDLA.Compute();
  myPIDLB.Compute();
  myPIDRB.Compute();

  motorLA = motorLB = motorRA = motorRB = 0; // 一个周期后计数置零
  // OutputRA = 20;
  // OutputLA = 20;
  // OutputLB = OutputRB = 0;
  // OutputRB = 20;
  // Serial1.print("\r\nOutputLA is :");
  // Serial1.println(OutputLA);
  // Serial1.print("OutputLB is :");
  // Serial1.println(OutputLB);
  // Serial1.print("OutputRA is :");
  // Serial1.println(OutputRA);
  // Serial1.print("OutputRB is :");
  // Serial1.println(OutputRB);
  
  if (int(OutputLA) >= 0)
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
  // delay(500);
}

void READ_ENCODER_LA()
{
  // motorLA++;
  if (digitalRead(MotorLA1count) == HIGH)
  {
    if (digitalRead(MotorLA2count) == LOW)
      motorLA++; // 根据另外一相电平判定方向
    else
      motorLA--;
  }

  else
  {
    if (digitalRead(MotorLA2count) == LOW)
      motorLA--; // 根据另外一相电平判定方向
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
      motorLB++; // 根据另外一相电平判定方向
    else
      motorLB--;
  }

  else
  {
    if (digitalRead(MotorLB2count) == LOW)
      motorLB--; // 根据另外一相电平判定方向
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
      motorRA++; // 根据另外一相电平判定方向
    else
      motorRA--;
  }

  else
  {
    if (digitalRead(MotorRA2count) == LOW)
      motorRA--; // 根据另外一相电平判定方向
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
      motorRB++; // 根据另外一相电平判定方向
    else
      motorRB--;
  }

  else
  {
    if (digitalRead(MotorRB2count) == LOW)
      motorRB--; // 根据另外一相电平判定方向
    else
      motorRB++;
  }
}
          