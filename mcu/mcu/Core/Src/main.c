/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// for ATK process
#define UART_ENABLE_RE(USARTx) USARTx.Instance->CR1 |= (uint32_t)0x0004
#define UART_DISABLE_RE(USARTx) USARTx.Instance->CR1 &= (~(uint32_t)0x0004)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
// varables for the timer
uint8_t timel_flag = 0; // 0: first capture, 1: second capture
uint8_t timel_fin = 0;  // 0: not finished, 1: finished
uint64_t timel_1 = 0;
uint64_t timel_2 = 0; // left sensor
double cml = 0;       // cm of left sensor

uint8_t timer_flag = 0; // 0: first capture, 1: second capture
uint8_t timer_fin = 0;  // 0: not finished, 1: finished
uint64_t timer_1 = 0;
uint64_t timer_2 = 0; // right sensor
double cmr = 0;       // cm of right sensor

uint8_t timef_flag = 0;
uint8_t timef_fin = 0; // 0: not finished, 1: finished
uint32_t timef_1 = 0;
uint32_t timef_2 = 0; // right sensor
double cmf = 0;       // cm of right sensor

uint8_t ultraerrorcount_f=0;
uint8_t ultraerrorcount_l=0;
uint8_t ultraerrorcount_r=0;
uint8_t lflag=0;
uint8_t rflag=0;

double Inputultra, Outputultra;
double Setpointultra = 0;
double Inputdistance, Outputdistance;
double Setpointdistance = 15;

double Inputopenmv, Outputopenmv;
double Setpointopenmv = 0;

double Inputgyro, Outputgyro;
double Setpointgyro = 0;

PID_TypeDef myPIDultra;    // PID structure
PID_TypeDef myPIDdistance; // PID structure

PID_TypeDef myPIDopenmv; // PID structure

PID_TypeDef myPIDgyro; // PID structure

int xspeed = 0;    // speed of x axis
uint8_t xflag = 0; // 2: forward, 1: backward

int yspeed = 0;    // speed of y axis
uint8_t yflag = 0; // 2: right, 1: left

uint16_t wspeed = 0; // anguler speed
uint8_t wflag = 0;   // 2: counter-clock, 1: clock

uint8_t openmv_instrction[7] = {0}; // instruction from openmv

float roll, pitch, yaw; // ATK Return Value
int selfAngelint;
float initial_Pitch; // ATK Return Value
int initial_selfAngelint;
int openmvAngle=0;

uint8_t turnLeftCounter=0;
uint8_t turnRightCounter=0;

uint32_t timecount=200;

char buf[2]; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
// Redirect printf to UART
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

void afterArrowIdent(uint8_t angle)
{
  turn_Angle(angle, 1);
  // get initial value

  ATKPrcess();
  initial_selfAngelint = selfAngelint;

  while (1)
  {
    if (cmf < 40)
      break;

    Forward(15);
    walkStraight();
    drive();
    togglewalk();

  }
  Forward(20);
  drive();
  HAL_Delay(2500);

  Backward(20);
  drive();
  HAL_Delay(2500);
}
//toggle rg led
void togglewalk()
{
  HAL_GPIO_TogglePin(ldr_GPIO_Port,ldr_Pin);
  HAL_GPIO_TogglePin(ldg_GPIO_Port,ldg_Pin);
  HAL_Delay(5);

  HAL_GPIO_TogglePin(ldr_GPIO_Port,ldr_Pin);
  HAL_GPIO_TogglePin(ldg_GPIO_Port,ldg_Pin);
  HAL_Delay(5);

  HAL_GPIO_TogglePin(ldr_GPIO_Port,ldr_Pin);
  HAL_GPIO_TogglePin(ldg_GPIO_Port,ldg_Pin);
  HAL_Delay(5);

  HAL_GPIO_TogglePin(ldr_GPIO_Port,ldr_Pin);
  HAL_GPIO_TogglePin(ldg_GPIO_Port,ldg_Pin);
  HAL_Delay(5);

  HAL_Delay(30);
}
// define a function to toggle the LD2 LED in a certain pattern
void toggleLD2(uint32_t delay)
{
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_Delay(delay);
  // printf("Working\r\n");
}

// interrupt handler for the timer
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)
  {
    switch (htim->Channel)
    {
    case HAL_TIM_ACTIVE_CHANNEL_2: // right sensor
      if (timer_flag == 0)
      {
        timer_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        // printf("Echo right counter tr1= %d\r\n",timer_1);
        timer_flag = 1;
        timer_fin = 0;
      }
      else
      {
        timer_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        // printf("Echo right counter tr2= %d\r\n",timer_2);
        timer_flag = 0;
        timer_fin = 1;
        if (timer_1 < timer_2) // if the timer is not overflowed
        {
          cmr = (double)(timer_2 - timer_1) * 0.017; // 340*100/1e6/2
        }
        else
        {
          cmr = (double)(timer_2 + 0xffffffff - timer_1) * 0.017; // 340*100/1e6/2
        }
        timer_1 = 0;
        // printf("Distance right = %.3f cm. \r\n",  cmr);
      }
      // printf("Echo right: t1= %.3f us,  t2= %.3f us\r",timer_1*10, timer_2*10);
      HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
      break;

    case HAL_TIM_ACTIVE_CHANNEL_1: // left sensor
      if (timel_flag == 0)
      {
        timel_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        // printf("Echo left counter tl1= %d\r\n",timel_1);
        timel_flag = 1;
        timel_fin = 0;
      }
      else
      {
        timel_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        // printf("Echo left counter tl2= %d\r\n",timel_2);
        timel_flag = 0;
        timel_fin = 1;
        if (timel_1 < timel_2) // if the timer is not overflowed
        {
          cml = (double)(timel_2 - timel_1) * 0.017; // 340*100/1e6/2
        }
        else
        {
          cml = (double)(timel_2 + 0xffffffff - timel_1) * 0.017; // 340*100/1e6/2
        }
        timel_1 = 0;
      }
      // printf("Echo left: t1= %.3f us,  t2= %.3f us\r",timel_1*10, timel_2*10);
      // printf("Distance left = %.3f cm. \r\n",  cml);
      HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
      break;
    default:
      printf("Error timer channel.\r\n");
      break;
    }
  }
  if (htim->Instance == TIM4)
  {
    switch (htim->Channel)
    {
    case HAL_TIM_ACTIVE_CHANNEL_1: // right sensor
      if (timef_flag == 0)
      {
        timef_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        // printf("Echo front: tf1= %d us\r",timef_1*10);
        timef_flag = 1;
        timef_fin = 0;
      }
      else
      {
        timef_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        timef_flag = 0;
        timef_fin = 1;
        if (timef_1 < timef_2) // if the timer is not overflowed
        {
          cmf = (double)(timef_2 - timef_1) * 3.4; // 340*100/5e3/2
        }
        else
        {
          cmf = (double)(timef_2 + 0xffff - timef_1) * 3.4; // 340*100/5e3/2
        }
        timef_1 = 0;
      }
      // printf("Echo front: t= %.3f us\r\n",timel_1*10, timer_2*10);
      // printf("Distance front = %.3f cm. \r\n",  cmf);
      HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
      break;
    default:
      printf("Error timer channel.\r\n");
      break;
    }
  }
}
void toggleldr(uint32_t delay)
{
  HAL_GPIO_WritePin(ldr_GPIO_Port, ldr_Pin, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(ldr_GPIO_Port, ldr_Pin, GPIO_PIN_RESET);
  HAL_Delay(delay);
}

void toggleldg(uint32_t delay)
{
  HAL_GPIO_WritePin(ldg_GPIO_Port, ldg_Pin, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(ldg_GPIO_Port, ldg_Pin, GPIO_PIN_RESET);
  HAL_Delay(delay);
}

void drive()
{
  // Transfer int to char for output
  char outputstr[11];
  outputstr[0] = 0x90;
  outputstr[1] = xflag;
  outputstr[2] = xspeed / 10;
  outputstr[3] = xspeed % 10;
  outputstr[4] = yflag;
  outputstr[5] = yspeed / 10;
  outputstr[6] = yspeed % 10;
  outputstr[7] = wflag;
  outputstr[8] = '0';
  outputstr[9] = '1';
  outputstr[10] = '2';
  // Transmit the instruction to the motor driver
  printf("Speed Left=%d, Speed Right=%d\r\n", xspeed, yspeed);
  HAL_UART_Transmit(&huart4, (uint8_t *)outputstr, 11, 100);
}
// Redirect arduino print to UART
void print(double c)
{
  printf("%.3f", c);
}
// Redirect arduino println to UART
void println(double str)
{
  printf("%.3f\r\n", str);
}
// Redifine output functions

void Right(uint8_t speed)
{
  yspeed = speed;
  yflag = 2;
}

void Left(uint8_t speed)
{
  yspeed = speed;
  yflag = 1;
}

void Forward(uint8_t speed)
{
  xspeed = speed;
  yspeed = speed;
  xflag = 2;
  yflag = 2;
}

void Backward(uint8_t speed)
{
  xspeed = speed;
  yspeed = speed;
  xflag = 1;
  yflag = 1;
}

void Turn_Left(uint8_t speed)
{
  xspeed = xspeed - speed;
  yspeed = yspeed + speed;
  wflag = 0;
  if (xspeed < 0)
  {
    xflag = 1;
  }
  else
  {
    xflag = 2;
  }
  if (yspeed < 0)
  {
    yflag = 1;
  }
  else
  {
    yflag = 2;
  }
}

void Turn_Right(uint8_t speed)
{
  xspeed = xspeed + speed;
  yspeed = yspeed - speed;
  wflag = 1;
  if (xspeed < 0)
  {
    xflag = 1;
  }
  else
  {
    xflag = 2;
  }
  if (yspeed < 0)
  {
    yflag = 1;
  }
  else
  {
    yflag = 2;
  }
}

void gyroAlignment(double input)
{
  Inputgyro = input;
  if (PID_Compute(&myPIDgyro)==_FALSE)
      printf("PID_Compute for gyro error\r\n");
    printf("Outputgyro = %.3f\r\n", Outputgyro);

  if (Outputgyro > 1)
  {

    Turn_Left((uint8_t)Outputgyro);
  }
  if (Outputgyro < -1)
  {
    Turn_Right((uint8_t)(-Outputgyro));
  }
  if ((Inputgyro <= 1) && (Inputgyro >= -1))
  {
    Turn_Left(0);
  }
}
/***example***/
//in task function
//Forward(10);
//gyroAlignment(angle);
//drive();
//toggleld2(50);
/***example***/

void Alignment(double cmleft, double cmright)
{
  printf("Distance left = %.3f cm, Distance right = %.3f cm.\r\n", cmleft, cmright);
  Inputultra = cmleft-cmright;

  //When previous process have not been finished, do nothing, just align
  if ((cmleft-cmright>2)||(cmleft-cmright<-2))
  {
    if (PID_Compute(&myPIDultra)==_FALSE)
      printf("PID_Compute for ultra error\r\n");
    printf("Outputultra = %.3f\r\n", Outputultra);

    if (Outputultra >= 0)
      Turn_Left((uint8_t)Outputultra);
    else
      Turn_Right((uint8_t)(-Outputultra));
    return;
  }

  // if((cmleft < 15)&&(cmright < 15))
  // {
  //   //go left
  //   printf("Too close to wall\r\n");
  //   HAL_GPIO_WritePin(ldr_GPIO_Port,ldr_Pin,GPIO_PIN_SET);
  //   for(uint8_t i =0; i<=9;i++)
  //   {
  //     HAL_GPIO_TogglePin(ldr_GPIO_Port,ldr_Pin);
  //     Forward(10);
  //     Turn_Left(8);
  //     drive();
  //     HAL_Delay(100);
  //   }
  //   HAL_GPIO_TogglePin(ldr_GPIO_Port,ldr_Pin);
  //   Forward(10);
  //   Turn_Right(0);
  //   drive();
  //   HAL_Delay(1000);
  //   HAL_GPIO_WritePin(ldr_GPIO_Port,ldr_Pin,GPIO_PIN_RESET);
  //   return;
  // }
  // else
  // {
  //   if((cmleft > 30)&&(cmright > 30))
  //   {
  //     //go right
  //     printf("Too far from wall\r\n");
  //     HAL_GPIO_WritePin(ldg_GPIO_Port,ldg_Pin,GPIO_PIN_SET);
  //     for(uint8_t i =0; i<=9;i++)
  //     {
  //       HAL_GPIO_TogglePin(ldg_GPIO_Port,ldg_Pin);
  //       Forward(10);
  //       Turn_Right(8);
  //       drive();
  //       HAL_Delay(100);
  //     }
  //     HAL_GPIO_TogglePin(ldg_GPIO_Port,ldg_Pin);
  //     Forward(10);
  //     Turn_Left(0);
  //     drive();
  //     HAL_Delay(1000);
  //     HAL_GPIO_WritePin(ldg_GPIO_Port,ldg_Pin,GPIO_PIN_RESET);
  //     return;

  //   }
  //   else
  //   {
  //     //do nothing
  //     Turn_Left(0);
  //   }   
  // } 
    
  

  if (PID_Compute(&myPIDultra)==_FALSE)
    printf("PID_Compute for ultra error\r\n");
  printf("Outputultra = %.3f\r\n", Outputultra);

  if (Outputultra >= 0)
  {
    Turn_Left((uint8_t)Outputultra);
  }
  else
  {
    Turn_Right((uint8_t)(-Outputultra));
  }
}

void superAlignment(double precision)
{
  printf("Super Alignment Started.\r\n");
  
  while (1)
  {
    while((timel_fin==1 && timer_fin ==1)!=1)
      {
        HAL_Delay(1);
      }
      //printf("cm left = %.3f, cm right = %.3f\r\n", cml, cmr);
      //cml+=1;
      

    if((cml-cmr<precision) && (cml-cmr)>(-1*precision))
    {
      //double check
      Forward(0);
      drive();
      HAL_Delay(500);

      printf("Super Alignment Finished.\r\n");
      Forward(0);
      drive();
      if((cml-cmr<precision) && (cml-cmr)>(-1*precision))
      {
        HAL_GPIO_WritePin(ldr_GPIO_Port,ldr_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ldg_GPIO_Port,ldg_Pin,GPIO_PIN_RESET);
        toggleLD2(500);
        return;
      }
      else
      {
        continue;
      }
    }
    else
    {
      Inputultra = cml-cmr;
      if (PID_Compute(&myPIDultra)==_FALSE)
        printf("PID_Compute for ultra error\r\n");
      printf("Outputultra = %.3f\r\n", Outputultra);
  
      if (Outputultra > 0)
      {
        Forward((uint8_t)(Outputultra));
        xflag=1;
      }
      else
      {
        Forward((int)(-1*Outputultra));
        yflag=1;
      }
      drive();

      HAL_GPIO_TogglePin(ldr_GPIO_Port,ldr_Pin);
      HAL_GPIO_TogglePin(ldg_GPIO_Port,ldg_Pin);
      HAL_Delay(300);
    }
  }
}

uint8_t hc12send(uint8_t data)
{
  return HAL_UART_Transmit(&huart5, &data, 1, 100);
}

uint8_t openmvreceive(void)
{
  return HAL_UART_Receive(&huart3, &openmv_instrction, 7, 100);
}

int atkAngleRound(int a)
{
  return ((a + 360) % 360);
}

void ATKPrcess() // update ATK value
{
  int r = 0;
  char ATKbuf[100];

  UART_ENABLE_RE(huart1);
  if (HAL_UART_Receive(&huart1, (uint8_t *)ATKbuf, 60, HAL_MAX_DELAY) == HAL_ERROR) // Read frames from ATK
  {
    UART_DISABLE_RE(huart1); // error
    ATKPrcess();
    return;
  }
  UART_DISABLE_RE(huart1);

  // char pp[] = "\x90\x90\x90\x90";
  // SendPC(pp, 4);
  // SendPC(ATKbuf, 30);

  char ATKframes[10];

  for (r = 2; r < 60; r++)
  { // Find the Report message
    if ((ATKbuf[r - 2] == 0x55 && ATKbuf[r - 1] == 0x53))
    {
      int i = 0, N = 8;
      char *tmp = &ATKbuf[r];

      for (i = 0; i < N; i++)
      {
        ATKframes[i] = tmp[i];
      }
      break;
    }
  }

  if (r == 60) // Do not find the correct reply
  {
    ATKPrcess();
    return;
  }

  roll = (float)((int16_t)(ATKframes[1] << 8) | ATKframes[0]) / 32768 * 180;
  pitch = (float)((int16_t)(ATKframes[3] << 8) | ATKframes[2]) / 32768 * 180;
  yaw = (float)((int16_t)(ATKframes[5] << 8) | ATKframes[4]) / 32768 * 180;
  selfAngelint = ((int)yaw + 180) % 360;
  // printf("yaw=%d\r\n", (int)yaw);
  // selfAngelint = ((int)pitch + 180) % 360;
  // SendPC("Roll = ", 8);
  // SendPCint(selfAngelint);
  // char qwq[4];
  // qwq[0] = (int)(selfAngelint/100) + '0';
  // qwq[1] = (int)(selfAngelint/10)%10 + '0';
  // qwq[2] = selfAngelint%10 + '0';
  // qwq[3] = '\n';
  // HAL_UART_Transmit(&huart5, qwq, 4, HAL_MAX_DELAY);
}

void Set_angle(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle, uint32_t countPeriod, uint32_t CycleTime)
{
  uint16_t compare_value = 0;
  if (angle <= 180)
  {
    compare_value = 0.5 * countPeriod / CycleTime + angle * countPeriod / CycleTime / 90; // compute the compare_value
    __HAL_TIM_SET_COMPARE(htim, Channel, compare_value);
  }
}

int GetOpemMv() // Return Turn Angle
{
  printf("IN GetOpenmv\r\n");
  uint8_t r = 0;
  char Mvbuf[10];
  char *frame;
  int TurnAngle = 0;

  UART_ENABLE_RE(huart3);
  if (HAL_UART_Receive(&huart3, (uint8_t *)Mvbuf, 10, HAL_MAX_DELAY) == HAL_ERROR) // Read frames from ATK
  {
    UART_DISABLE_RE(huart3); // error
    return HAL_ERROR;
  }
  UART_DISABLE_RE(huart3);

  for (r = 0; r < 5; r++)
  {
    if (Mvbuf[r] == 'a')
    {
      frame = Mvbuf + r;
      // printf("%s\r\n", frame);
      if (frame[1] == '1')
      {
        TurnAngle = (frame[2] - '0') * 100 + (frame[3] - '0') * 10 + (frame[4] - '0');
        break;
      }
      else if (frame[1] == '2')
      {
        TurnAngle = -((frame[2] - '0') * 100 + (frame[3] - '0') * 10 + (frame[4] - '0'));
        break;
      }
    }
  }
  if (Mvbuf[r] == 'a')
    return TurnAngle;
  return HAL_ERROR;
}

char GetOpemMvArrow() // Return Turn Angle
{
  uint8_t r = 0;
  char Mvbuf[10];

  UART_ENABLE_RE(huart3);
  if (HAL_UART_Receive(&huart3, (uint8_t *)Mvbuf, 2, HAL_MAX_DELAY) == HAL_ERROR) 
  {
    UART_DISABLE_RE(huart3); // error
    return HAL_ERROR;
  }
  UART_DISABLE_RE(huart3);

  if (Mvbuf[0] == 'b' && Mvbuf[1] == '1')
    return '1';
  if (Mvbuf[0] == 'b' && Mvbuf[1] == '2')
    return '2';
  if (Mvbuf[0] == 'b' && Mvbuf[1] == '3')
    return '3';
}

void walkStraight() // Return Turn Angle
{
  ATKPrcess();
  int relativeAngle = 0;

  if(atkAngleRound(selfAngelint - initial_selfAngelint) > 180)
    relativeAngle = -1 * atkAngleRound(initial_selfAngelint - selfAngelint);
  else
    relativeAngle =  atkAngleRound((selfAngelint - initial_selfAngelint));

  gyroAlignment(relativeAngle);
}

void turn_Angle(int angle, int direction)
{
  int aimAngle = 0;
  int iniAngle = 0;
  int comAngle[3];
  int tolAngle = 0;
  int avgAngle = 0;
  int n = 0;
  uint8_t flag = 1;
  ATKPrcess();
  iniAngle = selfAngelint;

  if (direction == 1)
  {
    Forward(0);
    // Left(0);
    // Turn_Left(300); //增大目前角度
    // drive();

    Forward(7);
    xflag=1;
    drive();

    ATKPrcess();
    iniAngle = selfAngelint;
    comAngle[n] = atkAngleRound(selfAngelint - iniAngle);
    tolAngle += comAngle[n];
    n = (n + 1) % 3;

    ATKPrcess();
    comAngle[n] = atkAngleRound(selfAngelint - iniAngle);
    tolAngle += comAngle[n];
    n = (n + 1) % 3;

    ATKPrcess();
    comAngle[n] = atkAngleRound(selfAngelint - iniAngle);
    tolAngle += comAngle[n];
    n = (n + 1) % 3;

    while (1)
    {
      // toggleLD2(10);
      ATKPrcess();
      tolAngle -= comAngle[n];
      comAngle[n] = atkAngleRound(selfAngelint - iniAngle);
      tolAngle += comAngle[n];
      avgAngle = (int)(tolAngle / 3);
      avgAngle = comAngle[n];
      n = (n + 1) % 3;
      if (avgAngle >= (angle - 2) && avgAngle <= (angle + 2))
        break;

      if (avgAngle >= (angle / 2) && avgAngle <= (angle + 3) && flag == 1)
      {
        flag = 2;
        Forward(7);
        xflag=1;
        drive();
        // Left(0);
        // Turn_Left(300); //增大目前角度
        // drive();
      }

      if (avgAngle >= (angle + 2) && flag == 2)
      {
        // flag = 0;
        Forward(0);
        // Left(0);
        // Turn_Right(200); //增大目前角度
        // drive();
        break;
      }
      printf("diff=%d, selfangle=%d, avgAngle=%d \r\n", atkAngleRound(selfAngelint - iniAngle), selfAngelint, avgAngle);
      // SendPCint(aimAngle);
    }

    // Left(0);
    // Turn_Left(0);
    // drive();
    // toggleLD2(100);

    // ATKPrcess();
    // if (atkAngleRound(selfAngelint - iniAngle) >= (angle-1) && atkAngleRound(selfAngelint - iniAngle) <= (angle+1))
    //   return;

    // Left(0);
    // Turn_Right(150); //减小目前角度
    // drive();

    Forward(0);
    drive();
    HAL_Delay(500);

    Forward(4);
    yflag=1;
    drive();
    while (1)
    {
      // toggleLD2(5);
      if (atkAngleRound(selfAngelint - iniAngle) >= (angle-3) && atkAngleRound(selfAngelint - iniAngle) <= (angle+3))
        break;
      ATKPrcess();

      // SendPCint(aimAngle);
    }
  }
  else if (direction == 2)
  {
    Forward(0);
    // Left(0);
    // Turn_Right(300);//减小目前角度
    // drive();
    Forward(7);
    yflag=1;
    drive();

    ATKPrcess();
    iniAngle = selfAngelint;
    comAngle[n] = atkAngleRound(iniAngle - selfAngelint);
    tolAngle += comAngle[n];
    n = (n + 1) % 3;

    ATKPrcess();
    comAngle[n] = atkAngleRound(iniAngle - selfAngelint);
    tolAngle += comAngle[n];
    n = (n + 1) % 3;

    ATKPrcess();
    comAngle[n] = atkAngleRound(iniAngle - selfAngelint);
    tolAngle += comAngle[n];
    n = (n + 1) % 3;

    while (1)
    {
      // toggleLD2(10);
      ATKPrcess();
      tolAngle -= comAngle[n];
      comAngle[n] = atkAngleRound(iniAngle - selfAngelint);
      tolAngle += comAngle[n];
      // avgAngle = (int)(tolAngle / 3);
      avgAngle = comAngle[n];
      n = (n + 1) % 3;
      if (avgAngle >= (angle - 3) && avgAngle <= (angle + 7))
        break;
      if (avgAngle >= (angle / 2) && avgAngle <= (angle + 7) && flag == 1)
      {
        flag = 2;
        Forward(7);
        yflag=1;
        drive();
        // Left(0);
        // Turn_Right(400); //增大目前角度
        // drive();
      }

      if (avgAngle >= (angle + 2) && flag == 2)
      {

        //        flag = 0;
        Forward(0);
        //        Left(0);
        //        Turn_Left(200); //增大目前角度
        //        drive();
        break;
      }
      printf("diff=%d, selfangle=%d, avgAngle=%d \r\n", atkAngleRound(iniAngle - selfAngelint), selfAngelint, avgAngle);
      // printf("diff=%d, selfangle=%d \r\n", atkAngleRound(selfAngelint - iniAngle), selfAngelint);
    }

    Forward(0);
    drive();
    
    HAL_Delay(500);
    
    Forward(4);
    xflag=1;
    drive();
    while (1)
    {
      if (atkAngleRound(iniAngle - selfAngelint) >= (angle-2) && atkAngleRound(iniAngle - selfAngelint) <= (angle+2))
        break;
      ATKPrcess();
    }
  }

  Forward(0);
  // Left(0);
  // Turn_Left(0);
  drive();
  toggleLD2(100);
  return;
}
void celebrate()
{
  while (1)
  {
    HAL_GPIO_TogglePin(ldr_GPIO_Port,ldr_Pin);
    toggleLD2(100);
    HAL_GPIO_TogglePin(ldr_GPIO_Port,ldr_Pin);
    toggleLD2(100);
    HAL_GPIO_TogglePin(ldg_GPIO_Port,ldg_Pin);
    toggleLD2(100);
    HAL_GPIO_TogglePin(ldg_GPIO_Port,ldg_Pin);
    toggleLD2(100);
  }
}
void task (uint8_t numberoftask)
{
  switch (numberoftask)
  {
  case 1:
    /****************TASK 1******************/
    printf("task 1 begin\r\n");
    
    while(((timel_fin==1 && timer_fin ==1)&& timef_fin==1)!=1)
    {
      //waiting for the counting to finish
      toggleLD2(10);
    }

    if (cml<20 && cmr<20)
    {//alignment 
      printf("Found the bridge\r\n");
      Forward(0);
      drive();
      HAL_Delay(1000);
      superAlignment(1);
      
      Forward(0);
      drive();
      HAL_Delay(1000);

      //turn right
      Forward(10);
      printf("Turning right\r\n");
      HAL_GPIO_WritePin(ldr_GPIO_Port, ldr_Pin,GPIO_PIN_SET);

      for(uint8_t i = 0 ;i<4;i++)
      {
        Forward(15);
        drive();
        toggleLD2(400);
      }

      Forward(0);
      drive();
      toggleLD2(500);

      turn_Angle(70,2);

      //finish the turning
      for(uint8_t i = 0 ;i<5;i++)
      {
        Forward(15);
        drive();
        toggleLD2(400);
      }
      HAL_GPIO_WritePin(ldr_GPIO_Port, ldr_Pin,GPIO_PIN_RESET);

      //alignment
      Forward(0);
      drive();
      HAL_Delay(1000);
      superAlignment(1);

      Forward(0);
      drive();
      HAL_Delay(1000);

      //get initial value
      
      ATKPrcess();
      initial_selfAngelint= selfAngelint;

      while (1)
      {
        if(cmf<19)
          break;

        Forward(15);
        walkStraight();
        drive();
        togglewalk();

      }

      Forward(0);
      drive();
      HAL_Delay(1000);

      turn_Angle(80,1);

      Forward(0);
      drive();
      HAL_Delay(1000);
      
      //trace the white line
      superAlignment(1);
      
      Forward(0);
      drive();
      HAL_Delay(1000);

      // UART_ENABLE_RE(huart3);
      // HAL_UART_Transmit(&huart3, "task3", 5, HAL_MAX_DELAY);
      // UART_DISABLE_RE(huart3);
      uint32_t time=0;
      ATKPrcess();
      initial_selfAngelint= selfAngelint;
      while (1)
      {
        // if((cml<20||cmr<20)&& time >30)
        //   break;

        // time++;

        // openmvAngle = GetOpemMv();
        // // openmvAngle=100;

        // Inputopenmv = openmvAngle;
        // if (openmvAngle != HAL_ERROR)
        // {
        //   printf("openmvangle=%d\r\n", openmvAngle);

        //   if (PID_Compute(&myPIDopenmv)==_FALSE)
        //     printf("PID_Compute for OpenMV error\r\n");

        //   printf("Outputopenmv = %.3f\r\n", Outputopenmv);

        //   if(Outputopenmv > 0)
        //   {
        //     Forward(15);
        //     Turn_Left((int)Outputopenmv);
        //     drive();
        //   }
        //   else
        //   {
        //     Forward(15);
        //     Turn_Right((int)((-1) * Outputopenmv));
        //     drive();
        //   }
        //   toggleLD2(250);
        // }
        
        
        if((cml<20||cmr<20)&& time >30)
          break;

        
        Forward(15);
        walkStraight();
        drive();
        togglewalk();

        time++;
      }
      Forward(0);
      drive();
      celebrate();
    }
    openmvAngle = GetOpemMv();
    // openmvAngle=100;

    Inputopenmv = openmvAngle;
    if (openmvAngle != HAL_ERROR)
    {
      printf("openmvangle=%d\r\n", openmvAngle);

      if (PID_Compute(&myPIDopenmv)==_FALSE)
        printf("PID_Compute for OpenMV error\r\n");

      printf("Outputopenmv = %.3f\r\n", Outputopenmv);

      if(Outputopenmv > 0)
      {
        Forward(15);
        Turn_Left((int)Outputopenmv);
        drive();
      }
      else
      {
        Forward(15);
        Turn_Right((int)((-1) * Outputopenmv));
        drive();
      }
      toggleLD2(25);
      return;
    }
    /****************TASK 1******************/
    break;
  case 2:
    /****************TASK 2******************/
    printf("task 2 begin\r\n");

    if (turnLeftCounter==2 && turnRightCounter==2)
    {
      printf("Going to the busket.\r\n");

      superAlignment(1);

      for (int i = 0; i < 6; i++)
      {
        Forward(15);
        drive();
        toggleLD2(500);
      }
      //stop 
      Forward(0);
      drive();
      toggleLD2(500);
      //throw the ball
      Set_angle(&htim2,TIM_CHANNEL_1, 150,20000,20);
      toggleLD2(1000);
      Set_angle(&htim2,TIM_CHANNEL_1, 0,20000,20);

      //go back
      for (int i = 0; i < 4; i++)
      {
        Backward(15);
        drive();
        toggleLD2(500);
      }
      superAlignment(1);
      

      //turn right
        //turn on the red led
        HAL_GPIO_WritePin(ldr_GPIO_Port, ldr_Pin,GPIO_PIN_SET);

        turn_Angle(90,2);

        //finish the turning
        HAL_GPIO_WritePin(ldr_GPIO_Port, ldr_Pin,GPIO_PIN_RESET);

      Backward(15);
      drive();

      while (1)//detect the planter
      {
        while(((timel_fin==1 && timer_fin ==1)&& timef_fin==1)!=1)
        {
          //waiting for the counting to finish
          toggleLD2(20);
        }
        if(cml<80||cmr<80)
        {
          Forward(0);
          drive();
          toggleLD2(500);
          break;
        }
        Backward(15);
        drive();
        toggleLD2(500);
      }
      hc12send('a');
      toggleLD2(500);

      //FINISH THE TASK

      for (uint8_t i = 0; i < 6; i++)
      {
        Backward(15);
        drive();
        toggleLD2(500);
      }
      
      Backward(0);
      drive();
      celebrate();
      
    }

      while((timel_fin==1 && timer_fin ==1)!=1)
      {
        //waiting for the counting to finish
        toggleLD2(20);
        //printf("waiting for the counting to finish\r\n");
      }
      timecount++;

      //remove strange datas
      if (cml>100)
      {       
        if(ultraerrorcount_l<2  && lflag != 1)
        {
          ultraerrorcount_l++;
          return;//skip if there is error data
        }
        else
        {
          ultraerrorcount_l=0;
          lflag = 1;
        }
      }
      else{
        lflag = 0;
      }

      if (cmr>100)
      {       
        if(ultraerrorcount_r<2  && rflag != 1)
        {
          ultraerrorcount_r++;
          return;//skip if there is error data
        }
        else
        {
          ultraerrorcount_r=0;
          rflag = 1;
        }
      }
      else{
        rflag = 0;
      }

      //cml+=1;//remove fixed error

      if ((cmr-cml>50)&& timecount>200)//if the difference is too large, then turn right
      {
        if (turnRightCounter == 0)//first edge
        {
          timecount =0;
          turnRightCounter++;
          //turn on the red led
          printf("Turning right\r\n");
          HAL_GPIO_WritePin(ldr_GPIO_Port, ldr_Pin,GPIO_PIN_SET);

          for(uint8_t i = 0 ;i<2;i++)
          {
            Forward(15);
            drive();
            toggleLD2(500);
          }

          Forward(0);
          drive();
          toggleLD2(500);

          turn_Angle(70,2);

          HAL_GPIO_WritePin(ldr_GPIO_Port, ldr_Pin,GPIO_PIN_RESET);

          ATKPrcess();
          initial_selfAngelint= selfAngelint;

          while (1)
          {
            if(cmf<30)
              break;

            Forward(15);
            walkStraight();
            drive();
            togglewalk();

          }
          //turn left
          timecount=0;
          turnLeftCounter++;
          printf("Turning left\r\n");
          //turn on the green led
          HAL_GPIO_WritePin(ldg_GPIO_Port, ldg_Pin,GPIO_PIN_SET);

          Forward(0);
          drive();
          toggleLD2(500);

          //turn left
          turn_Angle(90,1);

          //finish the turning
          HAL_GPIO_WritePin(ldg_GPIO_Port, ldg_Pin,GPIO_PIN_RESET);
          Forward(15);
          drive();
          return;
 
        }
        else
        {
          timecount =0;
          turnRightCounter++;
          //turn on the red led
          printf("Turning right\r\n");
          HAL_GPIO_WritePin(ldr_GPIO_Port, ldr_Pin,GPIO_PIN_SET);

          for(uint8_t i = 0 ;i<2;i++)
          {
            Forward(15);
            drive();
            toggleLD2(500);
          }

          Forward(0);
          drive();
          toggleLD2(500);

          turn_Angle(60,2);

          //finish the turning
          for(uint8_t i = 0 ;i<12;i++)
          {
            Forward(15);
            drive();
            toggleLD2(500);
          }
          HAL_GPIO_WritePin(ldr_GPIO_Port, ldr_Pin,GPIO_PIN_RESET);
          return;
        }
      }

      if((cmf<25)&& timecount>200)//turn left
      {
        timecount=0;
        turnLeftCounter++;
        printf("Turning left\r\n");
        //turn on the green led
        HAL_GPIO_WritePin(ldg_GPIO_Port, ldg_Pin,GPIO_PIN_SET);

        Forward(0);
        drive();
        toggleLD2(500);

        //turn left
        turn_Angle(90,1);

        //finish the turning
        HAL_GPIO_WritePin(ldg_GPIO_Port, ldg_Pin,GPIO_PIN_RESET);
        Forward(15);
        drive();
        return;
      }
      else//nothing in front
      {
        Forward(15);
        Alignment(cml, cmr);
        drive();
        toggleLD2(25);
        return;      
      }

    /****************TASK 2******************/
    break;
  
  default:

    HAL_GPIO_WritePin(ldr_GPIO_Port,ldr_Pin,GPIO_PIN_SET);
    while(1)
    {
      printf("Error Task number!\r\n");
    }
    break;
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_LPUART1_UART_Init();
  MX_UART4_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  printf("Starting\r\n");
  // Load parameters to PID
  PID(&myPIDultra, &Inputultra, &Outputultra, &Setpointultra, 1.2, 1, 0.8, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&myPIDultra, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&myPIDultra, 50);
  PID_SetOutputLimits(&myPIDultra, -10, 10);

  PID(&myPIDdistance, &Inputdistance, &Outputdistance, &Setpointdistance, 0.8, 200, 15, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&myPIDdistance, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&myPIDdistance, 50);
  PID_SetOutputLimits(&myPIDdistance, -5, 5);

  PID(&myPIDopenmv, &Inputopenmv, &Outputopenmv, &Setpointopenmv, 0.2, 0, 0.12, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&myPIDopenmv, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&myPIDopenmv, 170);
  PID_SetOutputLimits(&myPIDopenmv, -10, 10);

  PID(&myPIDgyro, &Inputgyro, &Outputgyro, &Setpointgyro, 0.5, 0.1, 1.2, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&myPIDgyro, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&myPIDgyro, 50);
  PID_SetOutputLimits(&myPIDgyro, -4, 4);

  // start TIM1 PWM generator
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Machine Arm: 0(normal) and 180(putting)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // Bule servo(top): 90
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // Bule servo(openmv): 110(rectangle) and 65(45 degree)
  // start TIM5 IT left and right sensor
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

  // Servo initial position
  Set_angle(&htim2, TIM_CHANNEL_1, 0, 20000, 20);
  Set_angle(&htim2, TIM_CHANNEL_3, 90, 20000, 20);//90 for task 2
  // Set_angle(&htim2,TIM_CHANNEL_4, 110,20000,20);
  Set_angle(&htim2, TIM_CHANNEL_4, 100, 20000, 20);

  //Recode initial Pitch
  UART_DISABLE_RE(huart1);
  // ATKPrcess();
  // initial_Pitch = pitch;
  // initial_selfAngelint = selfAngelint;

  // UART_ENABLE_RE(huart3);
  // if (HAL_UART_Transmit(&huart3, "task1", 5, HAL_MAX_DELAY) == HAL_ERROR) 
  // {
  //   UART_DISABLE_RE(huart3); // error
  //   return HAL_ERROR;
  // }
  // UART_DISABLE_RE(huart3);
  
  HAL_Delay(5000);
  
  
  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
  printf("Initialized. \r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /****************Test******************/
    
      ATKPrcess();
      initial_selfAngelint= selfAngelint;

      while (1)
      {
        if(cmf<40)
          break;

        Forward(15);
        walkStraight();
        drive();
        togglewalk();
      }
      Forward(0);
      drive();
      UART_ENABLE_RE(huart3);
      HAL_UART_Transmit(&huart3, "task2", 5, HAL_MAX_DELAY);
      UART_DISABLE_RE(huart3);
      HAL_Delay(500);
      while (1)
      {
        UART_ENABLE_RE(huart3);
        if (HAL_UART_Receive(&huart3, (uint8_t *)buf, 2, HAL_MAX_DELAY)==HAL_OK)
        {
          if (buf [0]='b')
            break;
        }
      }
      //buf[1]='2';
      // 1 right; 2 front; 3 left
      switch (buf[1])
      {
      case '1':
        afterArrowIdent(45);
        
        turn_Angle(45,1);

        Forward(20);
        drive();
        HAL_Delay(4500);
        Forward(0);
        drive();
        HAL_Delay(500);
        superAlignment(1);
        celebrate();
        break;
      
      case '2':
        afterArrowIdent(90);

        turn_Angle(90,2);

        ATKPrcess();
        initial_selfAngelint = selfAngelint;

        while (1)
        {
          if (cmf < 40)
            break;

          Forward(15);
          walkStraight();
          drive();
          togglewalk();
        }

        turn_Angle(90, 1);
        superAlignment(1);
        celebrate();

        break;
      case '3':
        afterArrowIdent(135);

        turn_Angle(135, 2);
        ATKPrcess();
        initial_selfAngelint = selfAngelint;

        while (1)
        {
          if (cmf < 50)
            break;

          Forward(15);
          walkStraight();
          drive();
          togglewalk();
        }

        turn_Angle(90, 1);
        superAlignment(1);
        celebrate();

        break;
      default:
        break;
      }

    //toggleLD2(1000);
    // turn_Angle(90,2);
    // HAL_Delay(10000);
    /****************Test******************/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //just type the task number below
    //task(1);
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1699;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 33999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 169;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4.294967295E9;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ldr_Pin|ldg_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ldr_Pin ldg_Pin */
  GPIO_InitStruct.Pin = ldr_Pin|ldg_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
