#include <PID_v1.h>
#define Trigleft 50 
#define Echoleft 51 
#define Trigright 52
#define Echoright 53

/*******初始化pid算法*******/
double InputLA,OutputLA,SetpointLA;
double InputLB,OutputLB,SetpointLB;
double InputRA,OutputRA,SetpointRA;
double InputRB,OutputRB,SetpointRB;
double Inputultra, Outputultra, Setpointultra;
double Inputdistance, Outputdistance, Setpointdistance;

/*******超声波微对准参数*****/
volatile double cmleft = 0; //距离变量
volatile double templeft = 0;
volatile double cmright = 0;
volatile double tempright = 0;
volatile double distance = 0;

PID myPIDLA(&InputLA, &OutputLA, &SetpointLA, 0.1, 8, 0.1, DIRECT); //顺序:Ki,Kp,Kd
PID myPIDLB(&InputLB, &OutputLB, &SetpointLB, 0.1, 8, 0.1, DIRECT); //顺序:Ki,Kp,Kd
PID myPIDRA(&InputRA, &OutputRA, &SetpointRA, 0.1, 8, 0.1, DIRECT); //顺序:Ki,Kp,Kd
PID myPIDRB(&InputRB, &OutputRB, &SetpointRB, 0.1, 8, 0.1, DIRECT); //顺序:Ki,Kp,Kd
PID myPIDultra(&Inputultra, &Outputultra, &Setpointultra, 0, 8, 3, DIRECT);
PID myPIDdistance(&Inputdistance, &Outputdistance, &Setpointdistance, 0.1, 8, 0.1, DIRECT);

/*定义电机驱动接线引脚*/
int L1_IN1 = 4;     int L1_IN2 = 3;     int L1_ENA = 2;//左前轮
int R1_IN1 = 5;     int R1_IN2 = 6;     int R1_ENA = 7;//右前轮
int L2_IN1 = 10;    int L2_IN2 = 9;     int L2_ENA = 8;//左后轮
int R2_IN1 = 11;    int R2_IN2 = 12;    int R2_ENA = 13;//右后轮

/*定义中断(霍尔返回值)引脚*/
int MotorLA1count=42; 
int MotorLA2count=43;//左前轮
int MotorRA1count=44;
int MotorRA2count=45;//右前轮
int MotorLB1count=30;
int MotorLB2count=31;//左后轮 
int MotorRB1count=34;
int MotorRB2count=35;//右后轮

volatile double motorLA=0;//中断变量，左前轮脉冲计数
volatile double motorRA=0;//中断变量，右前轮脉冲计数
volatile double motorLB=0;//中断变量，左后轮脉冲计数
volatile double motorRB=0;//中断变量，右后轮脉冲计数

double V_LA,V_LAROS=0; //左前轮速度 单位cm/s
double V_LB,V_LBROS=0; //左后轮速度 单位cm/s
double V_RA,V_RAROS=0; //右前轮速度 单位cm/s
double V_RB,V_RBROS=0; //右后轮速度 单位cm/s

/*****底盘返回参数**********/
double K4_1=1.0/(4.0*18.1);  //底盘K值,X+Y=18.1cm
double linear_x;
double linear_y;
double linear_w;  //转动角速度

void setup() 
{
  pinMode(L1_IN1, OUTPUT);pinMode(L1_IN2, OUTPUT);pinMode(L1_ENA, OUTPUT);
  pinMode(R1_IN1, OUTPUT);pinMode(R1_IN2, OUTPUT);pinMode(R1_ENA, OUTPUT);
  pinMode(L2_IN1, OUTPUT);pinMode(L2_IN2, OUTPUT);pinMode(L2_ENA, OUTPUT);
  pinMode(R2_IN1, OUTPUT);pinMode(R2_IN2, OUTPUT);pinMode(R2_ENA, OUTPUT);
  pinMode(MotorLA1count,INPUT);pinMode(MotorLA2count,INPUT);
  pinMode(MotorRA1count,INPUT);pinMode(MotorRA2count,INPUT);
  pinMode(MotorLB1count,INPUT);pinMode(MotorLB2count,INPUT);
  pinMode(MotorRB1count,INPUT);pinMode(MotorRB2count,INPUT);

  pinMode(Trigleft, OUTPUT);  pinMode(Echoleft, INPUT);  //超声波对准引脚
  pinMode(Trigright, OUTPUT); pinMode(Echoright, INPUT);

  pinMode(A0,OUTPUT);  //提供转压的3.3V
  digitalWrite(A0,HIGH);

  SetpointLA=10;
  SetpointLB=10;
  SetpointRA=10;
  SetpointRB=10;
  myPIDLA.SetMode(AUTOMATIC);
  myPIDLB.SetMode(AUTOMATIC);
  myPIDRA.SetMode(AUTOMATIC);
  myPIDRB.SetMode(AUTOMATIC);
  myPIDLA.SetOutputLimits(-255,255);
  myPIDLB.SetOutputLimits(-255,255);
  myPIDRA.SetOutputLimits(-255,255);
  myPIDRB.SetOutputLimits(-255,255);
  myPIDLA.SetSampleTime(50);
  myPIDLB.SetSampleTime(50);
  myPIDRA.SetSampleTime(50);
  myPIDRB.SetSampleTime(50);

  Setpointultra = 0;  //超声波对准PID参数
  myPIDultra.SetMode(AUTOMATIC);
  myPIDultra.SetOutputLimits(-30, 30);
  myPIDultra.SetSampleTime(50);

  Setpointdistance = 9.5;  //要求距离
  myPIDdistance.SetMode(AUTOMATIC);
  myPIDdistance.SetOutputLimits(-30, 30);
  myPIDdistance.SetSampleTime(50);

  Serial.begin(115200);

  delay(10000);
}

void loop() 
{ 
  Read_Moto_V();
  //Alignment();
}

void L1_forward(int sp)//左前轮前进
{
  digitalWrite(L1_IN1,LOW);
  digitalWrite(L1_IN2,HIGH);
  analogWrite(L1_ENA,sp);
}
void R1_forward(int sp)//右前轮前进
{
  digitalWrite(R1_IN1,HIGH);
  digitalWrite(R1_IN2,LOW);
  analogWrite(R1_ENA,sp);
}
void L2_forward(int sp)//左后轮前进
{
  digitalWrite(L2_IN1,HIGH);
  digitalWrite(L2_IN2,LOW);
  analogWrite(L2_ENA,sp);
}
void R2_forward(int sp)//右后轮前进
{
  digitalWrite(R2_IN1,HIGH);
  digitalWrite(R2_IN2,LOW);
  analogWrite(R2_ENA,sp);
}
void allstop()
{
  digitalWrite(L1_IN1,LOW);
  digitalWrite(L1_IN2,LOW);  
  digitalWrite(R1_IN1,LOW);
  digitalWrite(R1_IN2,LOW);
  digitalWrite(L2_IN1,LOW);
  digitalWrite(L2_IN2,LOW);
  digitalWrite(R2_IN1,LOW);
  digitalWrite(R2_IN2,LOW);
}
void L1_backward(int sp)//左前轮后退
{
  digitalWrite(L1_IN1,HIGH);
  digitalWrite(L1_IN2,LOW);
  analogWrite(L1_ENA,sp);
}
void R1_backward(int sp)//右前轮后退
{
  digitalWrite(R1_IN1,LOW);
  digitalWrite(R1_IN2,HIGH);
  analogWrite(R1_ENA,sp);
}
void L2_backward(int sp)//左后轮后退
{
  digitalWrite(L2_IN1,LOW);
  digitalWrite(L2_IN2,HIGH);
  analogWrite(L2_ENA,sp);
}
void R2_backward(int sp)//右后轮后退
{
  digitalWrite(R2_IN1,LOW);
  digitalWrite(R2_IN2,HIGH);
  analogWrite(R2_ENA,sp);
}

void Forward(int sp)//前进函数封装
{
  L1_forward(sp);
  R1_forward(sp);
  L2_forward(sp);
  R2_forward(sp);
}

void Backward(int sp)//返回函数封装
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
  unsigned long nowtime=0;
  nowtime=millis()+50;//读50毫秒

  attachInterrupt(digitalPinToInterrupt(MotorLA1count), READ_ENCODER_LA, RISING); //左前轮
  attachInterrupt(digitalPinToInterrupt(MotorLB1count), READ_ENCODER_LB, RISING); //左后轮
  attachInterrupt(digitalPinToInterrupt(MotorRA1count), READ_ENCODER_RA, RISING); //右前轮
  attachInterrupt(digitalPinToInterrupt(MotorRB1count), READ_ENCODER_RB, RISING); //右后轮
  while(millis()<nowtime); //达到50毫秒关闭中断
  detachInterrupt(digitalPinToInterrupt(MotorLA1count)); //左前轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorLB1count)); //左后轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorRA1count)); //右前轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorRB1count)); //右后轮脉冲关中断计数

  // motorLA/390/0.05
  V_LA=((motorLA/390)*7.5*3.141)/(0.05*1.414);   //左前轮返回速度
  V_RA=((motorRA/390)*7.5*3.141)/(0.05*1.414);   //右前轮返回速度
  V_LB=((motorLB/390)*7.5*3.141)/(0.05*1.414);   //左后轮返回速度
  V_RB=((motorRB/390)*7.5*3.141)/(0.05*1.414);   //右后轮返回速度

  V_LAROS=((motorLA/390)*7.5*3.141)/(0.05);   //给ROS使用的参数
  V_RAROS=((motorRA/390)*7.5*3.141)/(0.05);   
  V_LBROS=((motorLB/390)*7.5*3.141)/(0.05);   
  V_RBROS=((motorRB/390)*7.5*3.141)/(0.05);   

  linear_x=0.25*V_LAROS+0.25*V_LBROS+0.25*V_RBROS+0.25*V_RAROS;
  linear_y=-0.25*V_LAROS+0.25*V_LBROS-0.25*V_RBROS+0.25*V_RAROS;
  linear_w=-K4_1*V_LAROS-K4_1*V_LBROS+K4_1*V_RBROS+K4_1*V_RAROS;  //K4也是用的cm,所以综合下来单位就是rad/s

  motorLA=motorLB=motorRA=motorRB=0;  //一个周期后计数置零

  InputLA=V_LA;InputRA=V_RA;InputLB=V_LB;InputRB=V_RB;
  myPIDLA.Compute();myPIDLB.Compute();myPIDRA.Compute();myPIDRB.Compute();

  if(int(OutputLA)>=0)
  {
    digitalWrite(L1_IN1,LOW);
    digitalWrite(L1_IN2,HIGH);
    analogWrite(L1_ENA,(int)OutputLA);
  }
  else
  {
    digitalWrite(L1_IN1,HIGH);
    digitalWrite(L1_IN2,LOW);
    analogWrite(L1_ENA,-(int)OutputLA);
  }

  if(int(OutputLB)>=0)
  {
    digitalWrite(L2_IN1,HIGH);
    digitalWrite(L2_IN2,LOW);
    analogWrite(L2_ENA,(int)OutputLB);
  }
  else
  {
    digitalWrite(L2_IN1,LOW);
    digitalWrite(L2_IN2,HIGH);
    analogWrite(L2_ENA,-(int)OutputLB);
  }

  if(int(OutputRA)>=0)
  {
    digitalWrite(R1_IN1,HIGH);
    digitalWrite(R1_IN2,LOW);
    analogWrite(R1_ENA,(int)OutputRA);
  }
  else
  {
    digitalWrite(R1_IN1,LOW);
    digitalWrite(R1_IN2,HIGH);
    analogWrite(R1_ENA,-(int)OutputRA);
  }

  if(int(OutputRB)>=0)
  {
    digitalWrite(R2_IN1,HIGH);
    digitalWrite(R2_IN2,LOW);
    analogWrite(R2_ENA,(int)OutputRB);
  }
  else
  {
    digitalWrite(R2_IN1,LOW);
    digitalWrite(R2_IN2,HIGH);
    analogWrite(R2_ENA,-(int)OutputRB);
  }

  Serial.print("LeftA:");
  Serial.println(V_LA);
  Serial.print("LeftB:");
  Serial.println(V_LB);
  Serial.print("RightA:");
  Serial.println(V_RA);
  Serial.print("RightB:");
  Serial.println(V_RB);
  Serial.print("Angular velocity:");
  Serial.println(linear_w);
}

void Alignment()
{
  cmleft = READ_LEFT_DIS();
  cmright = READ_RIGHT_DIS();

  Inputultra = cmleft - cmright;
  Inputdistance = (cmleft + cmright) / 2;

  myPIDdistance.Compute();

  if(Outputdistance < 0)
  {
    if(Inputdistance < 11)
    {
      Serial.println(Inputdistance);
      Serial.println(Outputdistance);
      allstop();
    }
    else
    {
      Backward(-(int)Outputdistance);
      Serial.println(Inputdistance);
      Serial.println(Outputdistance);
    }
  }
  else
  {
    if(Inputdistance > 9)
    {
      Serial.println(Inputdistance);
      Serial.println(Outputdistance);
      allstop();
    }
    else
    {
      Forward(-(int)Outputdistance);
      Serial.println(Inputdistance);
      Serial.println(Outputdistance);
    }
  }

  myPIDultra.Compute();

  if (Outputultra >= 0)
  {
    if(Inputultra < 2)
    {
      Serial.print(Inputultra);
    }
    else
    {
      Turn_Left((int)Outputultra);
      Serial.print(Inputultra);
    }
  }
  else
  {
    if(Inputultra > -2)
    {
      Serial.print(Inputultra);
    }
    else
    {
      Turn_Right(-(int)Outputultra);
      Serial.print(Inputultra);
    }
  }
  Serial.print("Echoleft,right =");
  Serial.print(templeft);//串口输出等待时间的原始数据
  Serial.print(",");
  Serial.print(tempright);
  Serial.print(" | | Distanceleft,right = ");
  Serial.print(cmleft);//串口输出距离换算成cm的结果
  Serial.print(",");
  Serial.print(cmright);
  Serial.println("cm");
}

double READ_LEFT_DIS()
{
   //给Trig发送一个低高低的短时间脉冲,触发测距
  digitalWrite(Trigleft, LOW);
  delayMicroseconds(2);    //等待2微秒
  digitalWrite(Trigleft,HIGH);
  delayMicroseconds(10);   //等待10微秒
  digitalWrite(Trigleft, LOW); 

  templeft = double(pulseIn(Echoleft, HIGH)); //存储回波等待时间,
  //pulseIn函数会等待引脚变为HIGH,开始计算时间,再等待变为LOW并停止计时
  //返回脉冲的长度
  
  //声速是:340m/1s 换算成 34000cm / 1000000μs => 34 / 1000
  //因为发送到接收,实际是相同距离走了2回,所以要除以2
  //距离(厘米)  =  (回波时间 * (34 / 1000)) / 2
  //简化后的计算公式为 (回波时间 * 17)/ 1000
  double disleft;
  disleft = (templeft * 17 ) / 1000; //把回波时间换算成cm
  delay(10);
  return disleft;
}

double READ_RIGHT_DIS()
{
  digitalWrite(Trigright, LOW); //给Trig发送一个低电平
  delayMicroseconds(2);    //等待2微秒
  digitalWrite(Trigright, HIGH);//给Trig发送一个高电平
  delayMicroseconds(10);    //等待10微秒
  digitalWrite(Trigright, LOW); //给Trig发送一个低电平

  tempright = double(pulseIn(Echoright, HIGH)); //存储回波等待时间,
  //pulseIn函数会等待引脚变为HIGH,开始计算时间,再等待变为LOW并停止计时
  //返回脉冲的长度
  
  //声速是:340m/1s 换算成 34000cm / 1000000μs => 34 / 1000
  //因为发送到接收,实际是相同距离走了2回,所以要除以2
  //距离(厘米)  =  (回波时间 * (34 / 1000)) / 2
  //简化后的计算公式为 (回波时间 * 17)/ 1000
  double disright;
  disright = (tempright * 17 ) / 1000; //把回波时间换算成cm
  delay(10);
  return disright;
}

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
