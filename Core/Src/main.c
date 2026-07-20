/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "math.h"
#include <complex.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  float kp;                       //比例系数Proportional
  float ki;                       //积分系数Integral
  float kd;                       //微分系数Derivative
  float ek;                       //当前误差
  float ek1;                      //前一次误差e(k-1)
  float ek2;                      //再前一次误差e(k-2)
  float location_sum;             //累计积分位置
  float out;											//PID输出
}PID_LocTypeDef;

typedef struct Frame {
  float fdata[6];
  unsigned char tail[4]; // 固定帧尾
} Frame_t;

typedef struct SOGI{
    float ualfa_0;      // 输入信号 alpha 分量
    float SOGI_Ualfa;   // 输出正交 alpha
    float SOGI_Ubeta;   // 输出正交 beta

    float integral_2;   // alpha内部积分
    float integral_3;   // beta内部积分

    float Ugird_W0;     // ω0 = 2πf
    float samp_t;       // 采样周期
} SOGI_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float ek;
    float ek_1;
    float ek_2;
    float uk;
    float uk_1;
} PID_t;


typedef struct {
    float wt;       // 当前相位
    float w0;       // 当前角频�??????????????????
    PID_t pid;      // PLL的PID
    SOGI_t sogi;    // SOGI结构
  } PLL_t;

typedef struct{
    float kp ;
    float kr ;
    float wi ;
    float reference ;
    float ts ;
    float L_vir;
    float output_of_backward_integrator ;
    float output_of_feedback ;
    float output_of_forward_integrator ;
    float last_input_of_forward_integrator ;
    float error;
    float input_of_forward_integrator;
    float output ;
}PR_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_SIZE   4
#define PAID          3.1415926
#define T_sample      0.00005f     //系统采样频率20kHz
#define Gird_f        50           //系统电网频率50Hz
#define ROOT_2        1.414f
#define REROOT_2      0.707f

#define OLED_REFRESH_MS 200U
#define VOFA_SEND_DIVIDER 20U
#define START_KEY_DEBOUNCE_MS 30U
#define PWM_PERIOD_COUNTS 8400U
#define PWM_HALF_PERIOD_COUNTS 4200U
#define PWM_OUTPUT_PINS (GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float  sin_1[400] = {0};

uint16_t UO_Duty, IO_Duty;
uint16_t UO_cnt = 0, IO_cnt = 0;
uint16_t vofa_send_cnt = 0;
float ww;
float m = 0.75, n;
float uq,ud;

uint32_t dma_adc_buffer[2] = {0};

PLL_t UO_PLL;
PID_LocTypeDef UO_PID;
volatile float UO, UO_RMS, UO_AIM, UO_PID_SUM, UO_PID_RMS;

// PLL_t IO_PLL;
PR_t IO_PR;
volatile float IO, IO_AIM, IO_REF;
// volatile float IO_RMS;
static volatile uint8_t app_spwm_started = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SinglePhase(void);
void PLL_update(PLL_t *pll, float ualpha_input);
void Sogi_fun(SOGI_t *sogi);
void PLL_Init(PLL_t *pll);
void Sogi_Init(SOGI_t *sogi);
void dq_pll(PLL_t *pll);
float zl_pid_increase(float error, PID_t *pid);
void UART_SendFrame(UART_HandleTypeDef *huart, float ch1,float ch2,float ch3, float ch4, float ch5, float ch6);
void PID_Init(PID_LocTypeDef *PID);
float PID_location(float setvalue, float actualvalue, float PID_LIMIT_MIN, float PID_LIMIT_MAX, PID_LocTypeDef *PID);
void PR_Init(PR_t *s, float kp_set, float kr_set, float wi_set, float ts);
void PR_calc(PR_t *s, float reference, float feedback, float wg);
static void App_StartSpwm(void);
static void App_StopSpwm(void);
static void App_DrawMenu(void);
static void App_DrawRunStatus(void);
static void App_PwmOutputsToTimerAf(void);
static void App_PwmOutputsToGpioLow(void);
static uint8_t Keypad1Pressed(void);
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
  SinglePhase();
  UO_AIM = 24.0f;
  IO_AIM = 2.0f;

  PLL_Init(&UO_PLL);
  PID_Init(&UO_PID);

  // PLL_Init(&IO_PLL);
  PR_Init(&IO_PR, 0.001, 10, 2, 0.00005);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  App_StopSpwm();

  HAL_ADC_Start_DMA(&hadc1,dma_adc_buffer,2);
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC);

  HAL_Delay(100);
  OLED_Init();
  App_DrawMenu();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_oled_refresh = 0U;

    if (Keypad1Pressed() != 0U)
    {
      if (app_spwm_started == 0U)
      {
        App_StartSpwm();
        App_DrawRunStatus();
      }
      else
      {
        App_StopSpwm();
        App_DrawMenu();
      }
      last_oled_refresh = HAL_GetTick();
    }

    if ((HAL_GetTick() - last_oled_refresh) >= OLED_REFRESH_MS)
    {
      last_oled_refresh = HAL_GetTick();

      if (app_spwm_started == 0U)
      {
        App_DrawMenu();
      }
      else
      {
        App_DrawRunStatus();
      }
    }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void App_StartSpwm(void)
{
  App_PwmOutputsToTimerAf();

  UO_cnt = 0U;
  IO_cnt = 0U;
  vofa_send_cnt = 0U;
  UO_PID_SUM = 0.0f;
  UO_PID_RMS = 0.0f;
  UO = 0.0f;
  IO = 0.0f;
  IO_REF = 0.0f;
  m = 0.75f;
  n = 0.5f;
  UO_Duty = PWM_HALF_PERIOD_COUNTS;
  IO_Duty = PWM_HALF_PERIOD_COUNTS;

  PLL_Init(&UO_PLL);
  PID_Init(&UO_PID);
  PR_Init(&IO_PR, 0.001f, 10.0f, 2.0f, T_sample);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, UO_Duty);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, IO_Duty);

  app_spwm_started = 1U;

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
}

static void App_StopSpwm(void)
{
  app_spwm_started = 0U;

  HAL_TIM_Base_Stop_IT(&htim2);
  HAL_TIM_Base_Stop_IT(&htim3);

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Base_Stop_IT(&htim1);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);

  UO_Duty = 0U;
  IO_Duty = 0U;
  UO_cnt = 0U;
  IO_cnt = 0U;
  vofa_send_cnt = 0U;
  UO_PID_SUM = 0.0f;
  UO_PID_RMS = 0.0f;
  IO_REF = 0.0f;
  n = 0.0f;

  App_PwmOutputsToGpioLow();
}

static void App_DrawMenu(void)
{
  OLED_NewFrame();
  OLED_PrintASCIIString(24, 0, "MENU", &afont16x8, OLED_COLOR_NORMAL);
  OLED_PrintASCIIString(0, 16, "SPWM: OFF", &afont16x8, OLED_COLOR_NORMAL);
  OLED_PrintASCIIString(0, 32, "PRESS KEY", &afont16x8, OLED_COLOR_NORMAL);
  OLED_PrintASCIIString(0, 48, "TO START", &afont16x8, OLED_COLOR_NORMAL);
  OLED_ShowFrame();
}

static void App_DrawRunStatus(void)
{
  OLED_NewFrame();
  OLED_PrintASCIIString(0, 0, "UO_RMS:", &afont16x8, OLED_COLOR_NORMAL);
  OLED_PrintFloat(64, 0, UO_RMS, 3U, &afont16x8, OLED_COLOR_NORMAL);
  OLED_PrintASCIIString(0, 16, "Wt:", &afont16x8, OLED_COLOR_NORMAL);
  OLED_PrintFloat(64, 16, UO_PLL.wt, 3U, &afont16x8, OLED_COLOR_NORMAL);
  OLED_PrintASCIIString(0, 32, "IO_RMS:", &afont16x8, OLED_COLOR_NORMAL);
  // OLED_PrintFloat(64, 32, IO_RMS, 3U, &afont16x8, OLED_COLOR_NORMAL);
  OLED_ShowFrame();
}

static void App_PwmOutputsToTimerAf(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIO_InitStruct.Pin = PWM_OUTPUT_PINS;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

static void App_PwmOutputsToGpioLow(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOE, PWM_OUTPUT_PINS, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = PWM_OUTPUT_PINS;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOE, PWM_OUTPUT_PINS, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim2)
  {
    if (app_spwm_started == 0U)
    {
      return;
    }

    // duty = 4200 + 4200 * m * cos(2 * PAID * cnt++ * T_sample);
    // if (cnt >= 20000) cnt = 0;
    UO = ((float)(dma_adc_buffer[0] / 4096.0f) * 3.3f - 1.56f) * 33.8f;
    IO = -((float)(dma_adc_buffer[1] / 4096.0f) * 3.3f - 1.5f) * (1173.1f / 330.0f);

    UO_Duty = PWM_HALF_PERIOD_COUNTS + PWM_HALF_PERIOD_COUNTS * m * sin_1[UO_cnt++];
    if (UO_cnt >= 400) UO_cnt = 0;

    PLL_update(&UO_PLL, UO);
    UO_RMS = REROOT_2 * sqrtf(UO_PLL.sogi.SOGI_Ualfa * UO_PLL.sogi.SOGI_Ualfa + UO_PLL.sogi.SOGI_Ubeta * UO_PLL.sogi.SOGI_Ubeta);

    UO_PID_SUM += UO_RMS;
    if (UO_cnt % 200 == 0) 
    {
      UO_PID_RMS = UO_PID_SUM / 200.0f;
      UO_PID_SUM = 0.0f;
      m = PID_location(UO_AIM, UO_PID_RMS, 0.2f, 0.9f, &UO_PID);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, UO_Duty);

    IO_REF = ROOT_2 * IO_AIM * cos(UO_PLL.wt - 0.375f);
    PR_calc(&IO_PR, IO_REF, IO, UO_PLL.w0);
    n=(IO_PR.output) / 50.0f + 0.5;
    if (n >= 0.95) n = 0.95;
    if (n <= 0.05) n = 0.05;
    IO_Duty = PWM_PERIOD_COUNTS * n;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, IO_Duty);
  }

  if(htim == &htim3)
  {
    if (app_spwm_started != 0U)
    {
      UART_SendFrame(&huart2, UO / 10.0f, IO, IO_REF, UO_PLL.wt, n, UO_Duty / 1000.0f);
    }
  }
}

void SinglePhase(void)
{
    for(int i = 0; i < 400; i++ )
    {
		sin_1[i] =cos( PAID * i/200.0);
	}
}

void UART_SendFrame(UART_HandleTypeDef *huart, float ch1,float ch2,float ch3, float ch4, float ch5, float ch6)
{
  Frame_t frame;

  frame.fdata[0] = ch1;
  frame.fdata[1] = ch2;
  frame.fdata[2] = ch3;
  frame.fdata[3] = ch4;
  frame.fdata[4] = ch5;
  frame.fdata[5] = ch6;

  // 设置帧尾（VOFA+默认识别的帧尾）
  frame.tail[0] = 0x00;
  frame.tail[1] = 0x00;
  frame.tail[2] = 0x80;
  frame.tail[3] = 0x7F;

  HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Frame_t), HAL_MAX_DELAY);
}

void PLL_Init(PLL_t *pll)
{
  pll->wt = 0;
  pll->w0 = 2 * Gird_f * PAID; // 初始频率
  pll->pid.kp = 10;
  pll->pid.ki = 0.2;
  pll->pid.kd = 0;
  pll->pid.uk = 0;
  pll->pid.ek = pll->pid.ek_1 = pll->pid.ek_2 = 0;

  Sogi_Init(&pll->sogi);
}

void Sogi_Init(SOGI_t *sogi)
{
	sogi->Ugird_W0=2 * Gird_f * PAID;
  sogi->integral_2 = 0;
  sogi->integral_3 = 0;
  sogi->samp_t = T_sample;
}

void PID_Init(PID_LocTypeDef *PID)
{
  PID->kp = 0.005f;
  PID->ki = 0.0030f;
  PID->kd = 0;
  PID->ek = 0;
	PID->ek1 = 0;
	PID->ek2 = 0;
  PID->location_sum = 0;
  PID->out = 0;
}

void PLL_update(PLL_t *pll, float ualpha_input)
{
  pll->sogi.ualfa_0 = ualpha_input;
  Sogi_fun(&pll->sogi);
  dq_pll(pll);
}

void Sogi_fun(SOGI_t *sogi)
{
  float x_a = (sogi->ualfa_0 - sogi->integral_2) * 1.0f;
  sogi->integral_2 += sogi->Ugird_W0 * (x_a - sogi->integral_3) * sogi->samp_t;
  sogi->SOGI_Ualfa = sogi->integral_2;


  sogi->integral_3 += sogi->Ugird_W0 * sogi->integral_2 * sogi->samp_t;
  sogi->SOGI_Ubeta = sogi->integral_3;
}

float PID_location(float setvalue, float actualvalue, float PID_LIMIT_MIN, float PID_LIMIT_MAX, PID_LocTypeDef *PID)
{
	PID->ek =setvalue-actualvalue;
	PID->location_sum += PID->ek;                         //计算累计误差
	if((PID->ki!=0)&&(PID->location_sum>(PID_LIMIT_MAX/PID->ki))) PID->location_sum=PID_LIMIT_MAX/PID->ki;
	if((PID->ki!=0)&&(PID->location_sum<(PID_LIMIT_MIN/PID->ki))) PID->location_sum=PID_LIMIT_MIN/PID->ki;//积分限幅

  PID->out=PID->kp*PID->ek+(PID->ki*PID->location_sum)+PID->kd*(PID->ek-PID->ek1);
  PID->ek1 = PID->ek;
	if(PID->out<PID_LIMIT_MIN)	PID->out=PID_LIMIT_MIN;
	if(PID->out>PID_LIMIT_MAX)	PID->out=PID_LIMIT_MAX;//PID->out限幅

	return PID->out;
}

void dq_pll(PLL_t *pll)
{
  float c = cosf(pll->wt);
  float s = sinf(pll->wt);

  uq = c * pll->sogi.SOGI_Ubeta - s * pll->sogi.SOGI_Ualfa;
  ud = c * pll->sogi.SOGI_Ualfa + s * pll->sogi.SOGI_Ubeta;

  float con_increase = zl_pid_increase(-uq, &pll->pid);
  pll->w0 = 2*Gird_f*PAID - con_increase;
  pll->wt += pll->w0 * pll->sogi.samp_t;
  if (pll->wt >= 2 * PAID ) pll->wt -= 2 * PAID ;
}

float zl_pid_increase(float error, PID_t *pid)
{
  pid->ek = error;
  float delta_u = pid->kp * (pid->ek - pid->ek_1)
                + pid->ki * pid->ek
                + pid->kd * (pid->ek - 2 * pid->ek_1 + pid->ek_2);
  pid->ek_2 = pid->ek_1;
  pid->ek_1 = pid->ek;
  pid->uk += delta_u;
  return pid->uk;
}

void PR_Init(PR_t *s, float kp_set, float kr_set, float wi_set, float ts)
{
    s->kp = kp_set ;
    s->kr = kr_set ;
    s->wi = wi_set ;
    s->ts = ts ;
    s->L_vir=0;
    s->output_of_feedback = 0;
    s->output_of_backward_integrator = 0;
    s->output_of_forward_integrator = 0 ;
    s->error=0;
    s->input_of_forward_integrator=0;
    s->reference = 0 ;
    s->output= 0 ;
}

void PR_calc(PR_t *s, float reference, float feedback, float wg)
{
    s->reference = reference;

    s->error = reference - feedback ;
    s->input_of_forward_integrator = 2 * s->wi * s->kr * s->error - s->output_of_feedback;
    // Forward integrator :
    s->output_of_forward_integrator += s->ts *  s->input_of_forward_integrator;

    // Backward integrator:
    s->output_of_backward_integrator += s->ts * s->output_of_forward_integrator * wg * wg ;

    s->output_of_feedback = s->output_of_backward_integrator + 2 * s->wi * s->output_of_forward_integrator ;

    s->output=s->output_of_forward_integrator + s->kp* s->error;
}

static uint8_t Keypad1Pressed(void)
{
  static uint8_t key_down = 0U;
  static uint32_t press_start_tick = 0U;
  uint8_t pressed = 0U;
  uint32_t now = HAL_GetTick();

  HAL_GPIO_WritePin(GPIOD, KEY_H1_Pin|KEY_H2_Pin|KEY_H3_Pin|KEY_H4_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(KEY_H1_GPIO_Port, KEY_H1_Pin, GPIO_PIN_RESET);
  for (volatile uint32_t i = 0U; i < 100U; ++i) { __NOP(); }
  pressed = (HAL_GPIO_ReadPin(KEY_V1_GPIO_Port, KEY_V1_Pin) == GPIO_PIN_RESET) ||
            (HAL_GPIO_ReadPin(KEY_V2_GPIO_Port, KEY_V2_Pin) == GPIO_PIN_RESET) ||
            (HAL_GPIO_ReadPin(KEY_V3_GPIO_Port, KEY_V3_Pin) == GPIO_PIN_RESET) ||
            (HAL_GPIO_ReadPin(KEY_V4_GPIO_Port, KEY_V4_Pin) == GPIO_PIN_RESET);
  HAL_GPIO_WritePin(KEY_H1_GPIO_Port, KEY_H1_Pin, GPIO_PIN_SET);

  if (pressed == 0U)
  {
    HAL_GPIO_WritePin(KEY_H2_GPIO_Port, KEY_H2_Pin, GPIO_PIN_RESET);
    for (volatile uint32_t i = 0U; i < 100U; ++i) { __NOP(); }
    pressed = (HAL_GPIO_ReadPin(KEY_V1_GPIO_Port, KEY_V1_Pin) == GPIO_PIN_RESET) ||
              (HAL_GPIO_ReadPin(KEY_V2_GPIO_Port, KEY_V2_Pin) == GPIO_PIN_RESET) ||
              (HAL_GPIO_ReadPin(KEY_V3_GPIO_Port, KEY_V3_Pin) == GPIO_PIN_RESET) ||
              (HAL_GPIO_ReadPin(KEY_V4_GPIO_Port, KEY_V4_Pin) == GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_H2_GPIO_Port, KEY_H2_Pin, GPIO_PIN_SET);
  }

  if (pressed == 0U)
  {
    HAL_GPIO_WritePin(KEY_H3_GPIO_Port, KEY_H3_Pin, GPIO_PIN_RESET);
    for (volatile uint32_t i = 0U; i < 100U; ++i) { __NOP(); }
    pressed = (HAL_GPIO_ReadPin(KEY_V1_GPIO_Port, KEY_V1_Pin) == GPIO_PIN_RESET) ||
              (HAL_GPIO_ReadPin(KEY_V2_GPIO_Port, KEY_V2_Pin) == GPIO_PIN_RESET) ||
              (HAL_GPIO_ReadPin(KEY_V3_GPIO_Port, KEY_V3_Pin) == GPIO_PIN_RESET) ||
              (HAL_GPIO_ReadPin(KEY_V4_GPIO_Port, KEY_V4_Pin) == GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_H3_GPIO_Port, KEY_H3_Pin, GPIO_PIN_SET);
  }

  if (pressed == 0U)
  {
    HAL_GPIO_WritePin(KEY_H4_GPIO_Port, KEY_H4_Pin, GPIO_PIN_RESET);
    for (volatile uint32_t i = 0U; i < 100U; ++i) { __NOP(); }
    pressed = (HAL_GPIO_ReadPin(KEY_V1_GPIO_Port, KEY_V1_Pin) == GPIO_PIN_RESET) ||
              (HAL_GPIO_ReadPin(KEY_V2_GPIO_Port, KEY_V2_Pin) == GPIO_PIN_RESET) ||
              (HAL_GPIO_ReadPin(KEY_V3_GPIO_Port, KEY_V3_Pin) == GPIO_PIN_RESET) ||
              (HAL_GPIO_ReadPin(KEY_V4_GPIO_Port, KEY_V4_Pin) == GPIO_PIN_RESET);
    HAL_GPIO_WritePin(KEY_H4_GPIO_Port, KEY_H4_Pin, GPIO_PIN_SET);
  }

  if (pressed == 0U)
  {
    key_down = 0U;
    press_start_tick = 0U;
    return 0U;
  }

  if (key_down != 0U)
  {
    return 0U;
  }

  if (press_start_tick == 0U)
  {
    press_start_tick = HAL_GetTick();
    return 0U;
  }

  if ((now - press_start_tick) < START_KEY_DEBOUNCE_MS)
  {
    return 0U;
  }

  key_down = 1U;
  return 1U;
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
#ifdef USE_FULL_ASSERT
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
