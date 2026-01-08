/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#define ARR_CNT 5
#define CMD_SIZE 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t rx2char;
extern cb_data_t cb_data;
extern volatile unsigned char rx2Flag;
extern volatile char rx2Data[50];
volatile int tim3Flag1Sec = 1;
volatile unsigned int tim3Sec;

static volatile uint16_t adc_dma[9];

static const uint8_t hs_seq[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}};
static const uint8_t rot_to_sensor[8] = {0, 1, 2, 4, 7, 6, 5, 3};
static const uint8_t sensor_to_rot[8] = {0, 1, 2, 7, 3, 6, 5, 4};

static const char *pos_str[8] = {"TL", "TM", "TR", "ML", "MR", "BL", "BM", "BR"};

static volatile uint32_t last_step_ms = 0;
static volatile uint32_t last_sense_ms = 0;
static volatile uint32_t last_send_ms = 0;
static volatile uint8_t sensor_send_enable = 0;
static volatile uint32_t sensor_send_period_ms = 1000;

static int8_t curr_pos = 0;
static int8_t target_pos = 0;
static int32_t move_steps_left = 0;
static uint8_t seq_idx = 0;

static float solar_v = 0.0f;

static int8_t pos_queue[4];
static uint8_t q_head = 0, q_tail = 0, q_count = 0;

volatile uint8_t save_pos_pending = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
char strBuff[64];
void MX_GPIO_LED_ON(int flag);
void MX_GPIO_LED_OFF(int flag);
void esp_event(char *);
char sendBuf[512] = {0};

static void step_drive(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
static void step_once(int dir);
static void flash_save_pos(uint32_t pos);
static uint32_t flash_load_pos(void);
static void start_move_to(int8_t rot_dest);
static void queue_push_unique(int8_t rot_dest);
static int8_t queue_pop(int *ok);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void step_drive(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, a ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, c ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, d ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void step_once(int dir)
{
  if (dir > 0)
  {
    seq_idx = (seq_idx + 1) & 0x07;
  }
  else
  {
    seq_idx = (seq_idx + 7) & 0x07;
  }

  step_drive(hs_seq[seq_idx][0], hs_seq[seq_idx][1],
             hs_seq[seq_idx][2], hs_seq[seq_idx][3]);
}

static void flash_save_pos(uint32_t pos)
{
  if (pos >= 8)
    return;

  uint32_t cur = *(volatile uint32_t *)(0x08060000U);
  uint32_t mg = *(volatile uint32_t *)(0x08060000U + 4);
  if (cur == pos && mg == 0xBEEFCAFEU)
    return;

  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef EraseInitStruct = {0};
  uint32_t SectorError = 0;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_7;
  EraseInitStruct.NbSectors = 1;
  HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (0x08060000U), pos);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (0x08060000U + 4), 0xBEEFCAFEU);
  HAL_FLASH_Lock();
}

static uint32_t flash_load_pos(void)
{
  uint32_t pos = *(volatile uint32_t *)(0x08060000U);
  uint32_t mg = *(volatile uint32_t *)(0x08060000U + 4);
  if (mg == 0xBEEFCAFEU && pos < 8)
    return pos;
  return 0;
}

static void start_move_to(int8_t rot_dest)
{
  if (rot_dest < 0)
    rot_dest += 8;
  if (rot_dest >= 8)
    rot_dest -= 8;
  target_pos = rot_dest;

  int d = target_pos - curr_pos;
  if (d > 4)
    d -= 8;
  if (d < -4)
    d += 8;

  move_steps_left = (d == 0) ? 0 : (512 * ((d > 0) ? d : -d));
}

static void queue_push_unique(int8_t rot_dest)
{
  if (rot_dest < 0)
    rot_dest += 8;
  if (rot_dest >= 8)
    rot_dest -= 8;

  int8_t last = (q_count ? pos_queue[(q_tail + 3) & 3] : (move_steps_left > 0 ? target_pos : curr_pos));
  if (rot_dest == last)
    return;

  if (q_count < 4)
  {
    pos_queue[q_tail] = rot_dest;
    q_tail = (q_tail + 1) & 3;
    q_count++;
  }
  else
  {
    pos_queue[(q_tail + 3) & 3] = rot_dest;
  }
}

static int8_t queue_pop(int *ok)
{
  if (q_count == 0)
  {
    if (ok)
      *ok = 0;
    return 0;
  }
  int8_t v = pos_queue[q_head];
  q_head = (q_head + 1) & 3;
  q_count--;
  if (ok)
    *ok = 1;
  return v;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  int ret = 0;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma, 9) != HAL_OK)
  {
    Error_Handler();
  }

  curr_pos = (int8_t)flash_load_pos();
  target_pos = curr_pos;

  printf("Start main() - wifi\r\n");
  ret |= drv_uart_init();
  ret |= drv_esp_init();
  if (ret != 0)
  {
    printf("Esp response error\r\n");
    Error_Handler();
  }

  AiotClient_Init();
  sensor_send_enable = 1;
  sensor_send_period_ms = 5000;
  last_send_ms = HAL_GetTick() - sensor_send_period_ms;

  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (strstr((char *)cb_data.buf, "+IPD") && cb_data.buf[cb_data.length - 1] == '\n')
    {
      strcpy(strBuff, strchr((char *)cb_data.buf, '['));
      memset(cb_data.buf, 0x0, sizeof(cb_data.buf));
      cb_data.length = 0;
      esp_event(strBuff);
    }
    if (rx2Flag)
    {
      printf("recv2 : %s\r\n", rx2Data);
      rx2Flag = 0;
    }

    if (tim3Flag1Sec)
    {
      tim3Flag1Sec = 0;
      if (!(tim3Sec % 10))
      {
        if (esp_get_status() != 0)
        {
          printf("server connecting ...\r\n");
          esp_client_conn();
          if (sensor_send_enable)
          {
            last_send_ms = HAL_GetTick() - sensor_send_period_ms;
          }
        }
      }
    }

    uint32_t now = HAL_GetTick();

    if (save_pos_pending)
    {
      save_pos_pending = 0;
      flash_save_pos((uint32_t)curr_pos); 
    }

    if ((now - last_sense_ms) >= 500)
    {
      last_sense_ms = now;

      uint16_t maxv = 0;
      int8_t max_idx = 0;

      for (int i = 0; i < 8; i++)
      {
        uint16_t v = adc_dma[i];
        if (v > maxv)
        {
          maxv = v;
          max_idx = i;
        }
      }

      uint16_t raw_s = adc_dma[8];
      solar_v = ((float)raw_s / 4095.0f) * 3.3f * 2.0f;

      char tx[160];
      int n = snprintf(tx, sizeof(tx),
                       "TL:%4.1f TM:%4.1f TR:%4.1f ML:%4.1f MR:%4.1f BL:%4.1f BM:%4.1f BR:%4.1f | Solar=%.3fV | Pos=%s->%s\r\n",
                       (adc_dma[0] * 100.0f) / 4095.0f, (adc_dma[1] * 100.0f) / 4095.0f, (adc_dma[2] * 100.0f) / 4095.0f, (adc_dma[3] * 100.0f) / 4095.0f, (adc_dma[4] * 100.0f) / 4095.0f, (adc_dma[5] * 100.0f) / 4095.0f, (adc_dma[6] * 100.0f) / 4095.0f, (adc_dma[7] * 100.0f) / 4095.0f,
                       solar_v, pos_str[rot_to_sensor[curr_pos]], pos_str[rot_to_sensor[target_pos]]);
      HAL_UART_Transmit(&huart2, (uint8_t *)tx, (uint16_t)n, 50);

      uint8_t cur_sensor_idx = rot_to_sensor[curr_pos];
      int diff = (int)maxv - (int)adc_dma[cur_sensor_idx];

      int8_t desired_rot = curr_pos;
      if (diff >= 300)
      {
        desired_rot = sensor_to_rot[max_idx];
      }

      if (move_steps_left > 0 || q_count > 0)
      {
        if (desired_rot != curr_pos)
          queue_push_unique(desired_rot);
      }
      else
      {
        if (desired_rot != curr_pos)
        {
          start_move_to(desired_rot);
        }
      }
    }

    if (sensor_send_enable)
    {
      uint32_t now2 = HAL_GetTick();
      if ((now2 - last_send_ms) >= sensor_send_period_ms)
      {
        last_send_ms = now2;

        // float 없이 정수 포맷으로 센서값 생성
        uint16_t tl = (uint16_t)((adc_dma[0] * 1000) / 4095);
        uint16_t tm = (uint16_t)((adc_dma[1] * 1000) / 4095);
        uint16_t tr = (uint16_t)((adc_dma[2] * 1000) / 4095);
        uint16_t ml = (uint16_t)((adc_dma[3] * 1000) / 4095);
        uint16_t mr = (uint16_t)((adc_dma[4] * 1000) / 4095);
        uint16_t bl = (uint16_t)((adc_dma[5] * 1000) / 4095);
        uint16_t bm = (uint16_t)((adc_dma[6] * 1000) / 4095);
        uint16_t br = (uint16_t)((adc_dma[7] * 1000) / 4095);
        uint32_t solar_mV = (uint32_t)((adc_dma[8] * 3300UL * 2UL) / 4095UL);

        snprintf(sendBuf, sizeof(sendBuf),
                 "[LT_STM_SQL]SENSOR@Pos@%s"
                 "@TL@%u.%u"
                 "@TM@%u.%u"
                 "@TR@%u.%u"
                 "@ML@%u.%u"
                 "@MR@%u.%u"
                 "@BL@%u.%u"
                 "@BM@%u.%u"
                 "@BR@%u.%u"
                 "@Solar@%lu.%03lu\r\n",
                 pos_str[rot_to_sensor[curr_pos]],
                 tl / 10, tl % 10,
                 tm / 10, tm % 10,
                 tr / 10, tr % 10,
                 ml / 10, ml % 10,
                 mr / 10, mr % 10,
                 bl / 10, bl % 10,
                 bm / 10, bm % 10,
                 br / 10, br % 10,
                 (unsigned long)(solar_mV / 1000UL),
                 (unsigned long)(solar_mV % 1000UL));
        esp_send_data(sendBuf);
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 38400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void esp_event(char *recvBuf)
{
  int i = 0;
  char *pToken;
  char *pArray[ARR_CNT] = {0};

  strBuff[strlen(recvBuf) - 1] = '\0'; //'\n' cut
  printf("Debug recv : %s\r\n", recvBuf);

  pToken = strtok(recvBuf, "[@]");
  while (pToken != NULL)
  {
    pArray[i] = pToken;
    if (++i >= ARR_CNT)
      break;
    pToken = strtok(NULL, "[@]");
  }

  if (!strcmp(pArray[1], "GETSENSOR"))
  {
    int sec = 0;
    if (pArray[2] && *pArray[2])
      sec = atoi(pArray[2]);
    if (sec <= 0)
    {
      sensor_send_enable = 0;
      sprintf(sendBuf, "[%s]ACK@0\n", pArray[0]);
    }
    else
    {
      sensor_send_enable = 1;
      sensor_send_period_ms = (uint32_t)sec * 1000UL;
      last_send_ms = HAL_GetTick() - sensor_send_period_ms; 
      sprintf(sendBuf, "[%s]ACK@%d\n", pArray[0], sec);
    }
    esp_send_data(sendBuf);
    return;
  }
  else if (!strncmp(pArray[1], " New conn", 8))
  {
    return;
  }
  else if (!strncmp(pArray[1], " Already log", 8))
  {
    return;
  }
  else
    return;

  esp_send_data(sendBuf);
  printf("Debug send : %s\r\n", sendBuf);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static int tim3Cnt = 0;

  if (htim->Instance == TIM3)
  {
    tim3Cnt++;
    if (tim3Cnt >= 1000) // 1ms * 1000 = 1Sec
    {
      tim3Flag1Sec = 1;
      tim3Sec++;
      tim3Cnt = 0;
    }
    return;
  }

  if (htim->Instance == TIM4)
  {
    if (move_steps_left > 0)
    {
      int dir = (target_pos - curr_pos);
      if (dir > 4)
        dir -= 8;
      if (dir < -4)
        dir += 8;

      int step_dir = (dir >= 0) ? -1 : +1; 
      step_once(step_dir);
      move_steps_left--;

      if (move_steps_left == 0)
      {
        curr_pos = target_pos;
        step_drive(0, 0, 0, 0);

        save_pos_pending = 1; 

        int ok = 0;
        int8_t next = queue_pop(&ok);
        if (ok)
        {
          start_move_to(next); 
        }
      }
    }
    return;
  }
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
