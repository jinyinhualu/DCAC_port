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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_SIZE   4
#define ADC_DMA_BUFFER_LENGTH  1U
#define PAID          3.1415926
#define T_sample      0.00005f     //系统采样频率20kHz
#define Gird_f        50           //系统电网频率50Hz

#define OLED_REFRESH_MS 200U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SinglePhase(void);
void PLL_update(PLL_t *pll, float ualpha_input);
void Sogi_fun(SOGI_t *sogi);
void PLL_init(PLL_t *pll);
void Sogi_init(SOGI_t *sogi);
void dq_pll(PLL_t *pll);
float zl_pid_increase(float error, PID_t *pid);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float  sin_1[400] = {0};

uint16_t duty1;
uint16_t cnt = 0;
float ww;
float  m=0.75;
float uq,ud;

PLL_t pll_UO;
uint16_t dma_adc_buffer[ADC_DMA_BUFFER_LENGTH] = {0};
volatile float UO,UO_RMS;

int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
    return ch;
}
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
  PLL_init(&pll_UO);

  SinglePhase();

  HAL_Delay(100);
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
  MX_USART1_UART_Init();
  OLED_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_adc_buffer, ADC_DMA_BUFFER_LENGTH) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);

  HAL_TIM_Base_Start_IT(&htim2);

  printf("USART1 ready: 115200 8N1, PA9 TX, PA10 RX\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_oled_refresh = 0U;

    if ((HAL_GetTick() - last_oled_refresh) >= OLED_REFRESH_MS)
    {
      last_oled_refresh = HAL_GetTick();

      OLED_NewFrame();
      OLED_PrintASCIIString(0, 0, "U0:", &afont16x8, OLED_COLOR_NORMAL);
      OLED_PrintFloat(48, 0, UO_RMS, 3U, &afont16x8, OLED_COLOR_NORMAL);
      OLED_ShowFrame();
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim2)  //判断中断是否来自于定时器
  {
    duty1=4200+m*4200*sin_1[cnt];
    ww=cnt*3.1416/200;
    cnt++;
    if(cnt==400){
      cnt=0;
    }

    UO = ((dma_adc_buffer[0]/4095.0f)*3.3f - 1.5f)*30.0f; // 将ADC采样值转换为实际电压值，假设ADC参考电压为3.3V，分辨率为12位，偏置为1.5V，放大倍数为30
    PLL_update(&pll_UO, UO);
    UO_RMS = 0.707f*sqrtf((pll_UO.sogi.SOGI_Ubeta)*(pll_UO.sogi.SOGI_Ubeta)+
                        (pll_UO.sogi.SOGI_Ualfa)*(pll_UO.sogi.SOGI_Ualfa));

    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,duty1);
  }
}

void SinglePhase(void)
{
    for(int i = 0; i < 400; i++ )
    {
		sin_1[i] =cos( PAID * i/200.0);
	}
}

void PLL_init(PLL_t *pll)
{
  pll->wt = 0;
  pll->w0 = 2*Gird_f*PAID; // 初始频率
  pll->pid.kp = 10;
  pll->pid.ki = 0.2;
  pll->pid.kd = 0;
  pll->pid.uk = 0;
  pll->pid.ek = pll->pid.ek_1 = pll->pid.ek_2 = 0;

  Sogi_init(&pll->sogi);
}

void Sogi_init(SOGI_t *sogi)
{
	sogi->Ugird_W0=2*Gird_f*PAID;
  sogi->integral_2 = 0;
  sogi->integral_3 = 0;
  sogi->samp_t = T_sample;
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
