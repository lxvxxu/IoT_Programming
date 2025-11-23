/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Fixed for Linker Error)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // printf 사용
#include <stdlib.h> // abs() 함수 사용을 위해 추가
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 스테핑 모터의 하프 스텝(Half-step) 구동을 위한 8단계 시퀀스 배열
uint8_t halfstep_seq[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

// 모터 ID 및 회전 상수 정의
#define MOTOR_ID_ROTATION   1   // 중앙 원판을 회전시키는 스테핑 모터 ID
#define MOTOR_ID_PUMP_BASE  2   // 주사기 펌프 모터의 시작 ID (2, 3, 4, 5번 사용 예정)
const int STEPS_PER_REVOLUTION = 4096; // 28BYJ-48 기준 1회전 스텝 수
const int NUM_SCENTS = 4;              // 총 향료 개수 (4등분)
const int STEP_PER_ML = 50; // 1ml 주입하는 데 쓰이는 스텝 수
const int STEPS_PER_SCENT_INDEX = STEPS_PER_REVOLUTION / NUM_SCENTS;


// 1. Enum 정의: 향료에 고유한 ID(인덱스)를 부여하여 이름을 호출할 수 있게 함
typedef enum {
    SCENT_LAVENDER = 0, // 첫 번째 향료 (인덱스 0)
    SCENT_CEDARWOOD,    // 인덱스 1
    SCENT_VANILLA,      // 인덱스 2
    SCENT_BERGAMOT,     // 인덱스 3
    SCENT_COUNT         // 총 향료의 개수 (4)
} Scent_ID_t;

// 2. Struct 정의: 향료의 상세 정보를 묶어 관리하는 구조체
typedef struct {
    const char* name;
    const char* note;
    const char* description;
    // 향료 주사기를 구동할 때 필요한 모터 ID 또는 핀 정보 등을 여기에 추가할 수 있습니다.
    int pump_motor_id;    // 이 향료를 펌핑할 때 사용할 모터 ID
    int steps_per_unit;   // 펌프의 용량 단위당 스텝 수 (예: 1ml당 1000스텝)
} Scent_Data_t;


// 3. 데이터 배열 선언: 정의된 구조체 타입으로 실제 향료 정보들을 배열로 저장
#define MAX_INGREDIENTS 4
const Scent_Data_t SCENT_DB[SCENT_COUNT] = {
    [SCENT_LAVENDER] = {
        .name        = "Lavender",
        .note        = "Middle Note",
        .description = "허브 향 + 은은한 플로럴, 차분함·안정·청결한 느낌을 줌.",
        .pump_motor_id = MOTOR_ID_PUMP_BASE + SCENT_LAVENDER, // 2번 모터
        .steps_per_unit = STEP_PER_ML // 용량 단위당 스텝 수 for 1ml, 주사기 별로 정밀하게 조정 필요 시 설정
    },
    [SCENT_CEDARWOOD] = {
        .name        = "Cedarwood",
        .note        = "Base Note",
        .description = "드라이한 우디 향, 깊고 고요하며 안정적인 잔향 제공.",
        .pump_motor_id = MOTOR_ID_PUMP_BASE + SCENT_CEDARWOOD, // 3번 모터
        .steps_per_unit = STEP_PER_ML
    },
    [SCENT_VANILLA] = {
        .name        = "Vanilla",
        .note        = "Base Note",
        .description = "달콤하고 따뜻한 크리미한 향, 포근함과 감성적인 잔향 담당.",
        .pump_motor_id = MOTOR_ID_PUMP_BASE + SCENT_VANILLA, // 4번 모터
        .steps_per_unit = STEP_PER_ML
    },
    [SCENT_BERGAMOT] = {
        .name        = "Bergamot",
        .note        = "Top Note",
        .description = "상큼한 시트러스 향, 밝고 청량하고 산뜻한 첫인상 담당.",
        .pump_motor_id = MOTOR_ID_PUMP_BASE + SCENT_BERGAMOT, // 5번 모터
        .steps_per_unit = STEP_PER_ML
    }
};

// 4. Enum 정의: 기분에 고유한 ID(인덱스) 부여
typedef enum {
    MOOD_FRESH = 0,     // 0: 상쾌함
    MOOD_CALM,          // 1: 차분함
    MOOD_CONFIDENT,     // 2: 자신감
    MOOD_SWEET,         // 3: 달콤함 / 설렘
    MOOD_ENERGETIC,     // 4: 활기참
    MOOD_COZY,          // 5: 포근함
    MOOD_DEEP,          // 6: 신비 / 사색
    MOOD_COUNT          // 총 기분 개수 (7)
} Mood_ID_t;

typedef struct {
    Scent_ID_t id;
    float volume_ml;
} Ingredient_t;

typedef struct {
    const char* mood_name;
    Ingredient_t ingredients[MAX_INGREDIENTS];
    int ingredient_count;
} Perfume_Recipe_t;

// 5. 기분별 레시피 데이터 배열
const Perfume_Recipe_t RECIPE_DB[MOOD_COUNT] = {
    [MOOD_FRESH] = {
        .mood_name = "상쾌함 (Fresh)",
        .ingredient_count = 4,
        .ingredients = {
            // L : C : V : B = 1 : 1 : 1 : 4 (Total 7)
            {SCENT_LAVENDER, 0.29f},
            {SCENT_CEDARWOOD, 0.29f},
            {SCENT_VANILLA, 0.29f},
            {SCENT_BERGAMOT, 1.14f},
        }
    },
    [MOOD_CALM] = {
        .mood_name = "차분함 (Calm)",
        .ingredient_count = 4,
        .ingredients = {
            // L : C : V : B = 4 : 2 : 1 : 1 (Total 8)
            {SCENT_LAVENDER, 1.00f},
            {SCENT_CEDARWOOD, 0.50f},
            {SCENT_VANILLA, 0.25f},
            {SCENT_BERGAMOT, 0.25f},
        }
    },
    [MOOD_CONFIDENT] = {
        .mood_name = "자신감 (Confident)",
        .ingredient_count = 4,
        .ingredients = {
            // L : C : V : B = 1 : 3 : 1 : 2 (Total 7)
            {SCENT_LAVENDER, 0.29f},
            {SCENT_CEDARWOOD, 0.86f},
            {SCENT_VANILLA, 0.29f},
            {SCENT_BERGAMOT, 0.57f},
        }
    },
    [MOOD_SWEET] = {
        .mood_name = "달콤함 / 설렘 (Sweet)",
        .ingredient_count = 4,
        .ingredients = {
            // L : C : V : B = 2 : 1 : 4 : 1 (Total 8)
            {SCENT_LAVENDER, 0.50f},
            {SCENT_CEDARWOOD, 0.25f},
            {SCENT_VANILLA, 1.00f},
            {SCENT_BERGAMOT, 0.25f},
        }
    },
    [MOOD_ENERGETIC] = {
        .mood_name = "활기참 (Energetic)",
        .ingredient_count = 4,
        .ingredients = {
            // L : C : V : B = 1 : 1 : 1 : 5 (Total 8)
            {SCENT_LAVENDER, 0.25f},
            {SCENT_CEDARWOOD, 0.25f},
            {SCENT_VANILLA, 0.25f},
            {SCENT_BERGAMOT, 1.25f},
        }
    },
    [MOOD_COZY] = {
        .mood_name = "포근함 (Cozy)",
        .ingredient_count = 4,
        .ingredients = {
            // L : C : V : B = 2 : 2 : 4 : 1 (Total 9)
            {SCENT_LAVENDER, 0.44f},
            {SCENT_CEDARWOOD, 0.44f},
            {SCENT_VANILLA, 0.89f},
            {SCENT_BERGAMOT, 0.22f},
        }
    },
    [MOOD_DEEP] = {
        .mood_name = "신비 / 사색 (Deep)",
        .ingredient_count = 4,
        .ingredients = {
            // L : C : V : B = 1 : 4 : 1 : 1 (Total 7)
            {SCENT_LAVENDER, 0.29f},
            {SCENT_CEDARWOOD, 1.14f},
            {SCENT_VANILLA, 0.29f},
            {SCENT_BERGAMOT, 0.29f},
        }
    },
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// 버튼 인터럽트가 발생했는지 확인하기 위한 플래그
volatile uint8_t button_pressed_flag = 0;
volatile int current_scent_index = 0; // 현재 원판의 위치 인덱스 (0~3)
volatile Mood_ID_t selected_mood = MOOD_CALM;
/* USER CODE END PV */

///////////////////////////////////////////////////////////////////
///////////////////////// 스테핑 모터 START /////////////////////////
// 함수 프로토타입 선언
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

// 모터 코일 핀의 High/Low를 설정하는 함수
void set_coils(int motor_id, uint8_t a, uint8_t b, uint8_t c, uint8_t d);
// 지정된 스텝 수만큼 모터를 구동하는 함수
void step_steps(int motor_id, int steps, int delay_ms);
// 지연(딜레이) 함수 (HAL_Delay를 사용)
void step_delay_ms(uint32_t ms);

// --- 조향 시스템 핵심 함수 프로토타입 ---
void rotate_to_scent(Scent_ID_t target_id);
void dispense_scent(Scent_ID_t id, float volume_ml);
void mix_perfume_dose(Scent_ID_t id, float volume_ml);
// ---  기분별 향수 제조 함수 ---
void make_perfume_by_mood(Mood_ID_t mood_id);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
///////////////////////// 스테핑 모터 END /////////////////////////
////////////////////////////////////////////////////////////////


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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  printf("시작\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (button_pressed_flag == 1)
	  {
	      // 플래그를 즉시 초기화하여 버튼을 한 번 눌렀을 때 한 번만 동작하도록 함
	      button_pressed_flag = 0;

	      GPIO_PinState sensorState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);

	      if (sensorState == GPIO_PIN_RESET)
	      {
	    	  printf("=============================== \r\n");
	          printf("향료 재고를 확인했습니다. \r\n");
	          printf("향료를 주입합니다. \r\n");

	          // 선택된 기분에 따라 향수 제조 실행
	          make_perfume_by_mood(selected_mood);

	          printf("향료가 주입되었습니다. \r\n");
	          // HAL_Delay(1000); // 작업 완료 후 1초 대기 (선택 사항)
	          // 다음 호출을 위해 기분 인덱스를 변경 (예시로 버튼 누를 때마다 다음 기분 선택)
	          // 실제 구현에서는 로터리 인코더, UART 입력 등을 통해 selected_mood를 변경해야 함
	          selected_mood = (selected_mood + 1) % MOOD_COUNT;

	          sensorState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	          if (sensorState != GPIO_PIN_RESET)
	          	  printf("향료 재고가 소진되었습니다. \r\n향료를 채워주세요.\r\n");
	          printf("=============================== \r\n");
	      }
	      else
	      {
	          // printf("-> [EMPTY] No Liquid. Motor skipped.\r\n");
	    	  printf("=============================== \r\n");
	    	  printf("향료가 부족합니다. \r\n");
	    	  printf("향료통에 향료를 채워주세요. \r\n");
	    	  printf("=============================== \r\n");
	      }
	  }

	  HAL_Delay(50); // 메인 루프의 실행 속도 조절 (이 딜레이는 모터 속도와는 무관함)


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 50-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, RADIO_ANT_SWITCH_Pin|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_RESET_GPIO_Port, RADIO_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_ANT_SWITCH_Pin */
  GPIO_InitStruct.Pin = RADIO_ANT_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADIO_ANT_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_RESET_Pin LED_2_Pin */
  GPIO_InitStruct.Pin = RADIO_RESET_Pin|LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LIQUID_SENSOR_1_Pin */
  GPIO_InitStruct.Pin = LIQUID_SENSOR_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LIQUID_SENSOR_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_DIO_0_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RADIO_DIO_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_DIO_1_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RADIO_DIO_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_NSS_Pin */
  GPIO_InitStruct.Pin = RADIO_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RADIO_NSS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// -----------------------------------------------------------------
// 스테핑 모터 제어 함수 정의 START
// -----------------------------------------------------------------
// 핀 정보는 반드시 MX_GPIO_Init에서 설정된 핀으로 수정해야 함.
// 현재 주석 처리된 부분은 임시로 작성되었음.

void set_coils(int motor_id, uint8_t a, uint8_t b, uint8_t c, uint8_t d){
    switch(motor_id)
    {
        case MOTOR_ID_ROTATION : // 모터 1: 중앙 원판
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, a ? GPIO_PIN_SET : GPIO_PIN_RESET); // IN1
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, b ? GPIO_PIN_SET : GPIO_PIN_RESET); // IN2
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, c ? GPIO_PIN_SET : GPIO_PIN_RESET); // IN3
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,  d ? GPIO_PIN_SET : GPIO_PIN_RESET); // IN4
            break;

        case MOTOR_ID_PUMP_BASE + 0 : // 모터 2: Lavender (예: PC0, PC1, PC2, PC3)
//            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, a ? GPIO_PIN_SET : GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, c ? GPIO_PIN_SET : GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, d ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        case MOTOR_ID_PUMP_BASE + 1 : // 모터 3: Cedarwood (예: PA4, PA5, PA6, PA7)
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, a ? GPIO_PIN_SET : GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, c ? GPIO_PIN_SET : GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, d ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        case MOTOR_ID_PUMP_BASE + 2 : // 모터 4: Vanilla (예: PB4, PB5, PB6, PB7)
//            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, a ? GPIO_PIN_SET : GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, c ? GPIO_PIN_SET : GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, d ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        case MOTOR_ID_PUMP_BASE + 3 : // 모터 5: Bergamot (예: PC4, PC5, PC6, PC7)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, a ? GPIO_PIN_SET : GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, c ? GPIO_PIN_SET : GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, d ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;

        default: break;
    }
}


void step_delay_ms(uint32_t ms){
	HAL_Delay(ms); // HAL_Delay를 사용하여 스텝 간 지연을 구현
}

// motor_id : 제어할 모터의 ID. 현재 코드에서는 1만 지원.
// steps : 회전할 총 스텝 수. 이 값의 부호가 방향을 결정.
// delay_ms : 각 스텝(Step) 간의 딜레이 시간(ms)
// 이 값이 작을수록 모터가 더 빨리 회전함. (보통 1ms~5ms 사이를 사용)

// step_steps(모터ID, 회전 수, 회전 시간);

// 사용 예시
// 정방향 1번 회전
// step_steps(1, ONE_CYCLE, 1);
// 정방향 0.5번 회전
// step_steps(1, ONE_CYCLE/2, 1);
// 역방향 1번 회전
// step_steps(1, STEPS_REVERSE, 1);

void step_steps(int motor_id, int steps, int delay_ms){
	int idx = 0; // 시퀀스 인덱스
	int direction = (steps > 0) ? 1 : -1; // 방향 설정 (양수: 정회전, 음수: 역회전)

	for(int s=0; s<abs(steps); ++s){
    // 인덱스 업데이트
    if(direction > 0){
      idx = (idx + 1) % 8; // 정회전 (0 -> 7)
    } else {
      idx = (idx + 7) % 8; // 역회전 (7 -> 0)
    }

    // 모터 코일 작동
    set_coils( motor_id, halfstep_seq[idx][0], halfstep_seq[idx][1],
                   halfstep_seq[idx][2], halfstep_seq[idx][3] );

    // 스텝 간 지연 (속도 조절)
    step_delay_ms(delay_ms);
  }
}
// -----------------------------------------------------------------
// 스테핑 모터 제어 함수 정의 END
// -----------------------------------------------------------------


// -----------------------------------------------------------------
// 원판 회전 함수 START
// -----------------------------------------------------------------
// 지정된 향료 ID의 위치로 원판을 회전시킨다.
// target_id: 목표 향료 ID (SCENT_LAVENDER 등) 전역 변수가 필요합니다.

void rotate_to_scent(Scent_ID_t target_id)
{
    int required_steps;
    int direction;

    // 현재 인덱스와 목표 인덱스 간의 차이 계산
    int delta = target_id - current_scent_index;

    // 회전 방향 설정 (가장 짧은 경로 선택)
    if (delta > 2) { // 3 -> 0 으로 가는 경우 (-1)
        direction = -(NUM_SCENTS - delta);
    } else if (delta < -2) { // 0 -> 3 으로 가는 경우 (+1)
        direction = NUM_SCENTS + delta;
    } else {
        direction = delta;
    }

    required_steps = direction * STEPS_PER_SCENT_INDEX;

    printf("Rotate: Moving from %d to %d (Steps: %d)\r\n",
           current_scent_index, target_id, required_steps);

    // 회전 모터 구동 (안정적인 속도 5ms 가정)
    step_steps(MOTOR_ID_ROTATION, required_steps, 5);

    // 구동 후 코일 전원 끄기 (발열 방지)
    set_coils(MOTOR_ID_ROTATION, 0, 0, 0, 0);

    // 현재 인덱스 업데이트
    current_scent_index = target_id;
}
// -----------------------------------------------------------------
// 원판 회전 함수 END
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// 향료 주입 함수 START
// -----------------------------------------------------------------
// 지정된 향료 주사기의 리드 스크류 모터를 구동하여 용량을 주입한다.
// id: 주입할 향료 ID, volume_ml: 주입할 용량 (ml 단위)
void dispense_scent(Scent_ID_t id, float volume_ml)
{
    const Scent_Data_t *scent = &SCENT_DB[id];

    // 필요한 총 스텝 수 계산 (소수점 처리를 위해 반올림)
    int total_steps = (int)(volume_ml * scent->steps_per_unit + 0.5f);

    // 펌프 모터 구동 (주입 방향은 음수(-) 스텝으로 가정)
    printf("Dispensing %s: Volume %.2f ml -> %d steps\r\n",
           scent->name, volume_ml, total_steps);

    // 펌핑 모터 구동 (리드 스크류는 토크가 중요하므로 조금 더 느린 딜레이 8ms 가정)
    step_steps(scent->pump_motor_id, -total_steps, 8);

    // 구동 후 코일 전원 끄기
    set_coils(scent->pump_motor_id, 0, 0, 0, 0);
}
// -----------------------------------------------------------------
// 향료 주입 함수 END
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// 조향 메인 프로세스 START
// -----------------------------------------------------------------
void mix_perfume_dose(Scent_ID_t id, float volume_ml)
{
    printf("\r\n--- Starting Dosing for %s (%.2f ml) ---\r\n",
           SCENT_DB[id].name, volume_ml);

    // 1. 원판 회전: 해당 향료가 공병 위에 오도록 위치 지정
    rotate_to_scent(id);

    HAL_Delay(500); // 위치 지정 후 안정화 대기

    // 2. 펌핑: 지정된 용량만큼 주사기 펌프 구동
    dispense_scent(id, volume_ml);

    HAL_Delay(1000); // 펌핑 후 다음 동작까지 대기
}
// -----------------------------------------------------------------
// 조향 메인 프로세스 END
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// 버튼 인터럽트 콜백 함수 START
// -----------------------------------------------------------------

/**
  * @brief  GPIO 외부 인터럽트 발생 시 호출되는 콜백 함수.
  * @param  GPIO_Pin: 인터럽트를 발생시킨 핀.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // B1_Pin (보통 PC13)에서 인터럽트가 발생했을 때
    if (GPIO_Pin == B1_Pin)
    {
        // 버튼 눌림 감지 (하강 에지: 버튼 누름)
        // Putty로 원하는 내용 출력
        // printf(">>> [BUTTON PRESSED] Nucleo B1 Button Clicked! <<<\r\n");

        button_pressed_flag = 1;

    }
}

// -----------------------------------------------------------------
// 버튼 인터럽트 콜백 함수 END
// -----------------------------------------------------------------

// -----------------------------------------------------------------
// 향료 정보 출력 함수 START
// -----------------------------------------------------------------

void print_scent_info(Scent_ID_t id)
{
    // 유효성 검사
    if (id >= SCENT_COUNT) {
        printf("Error: Invalid Scent ID.\r\n");
        return;
    }

    const Scent_Data_t *scent = &SCENT_DB[id];

    printf("===========================================\r\n");
    printf("  [Scent Name]  : %s\r\n", scent->name);
    printf("  [Note Type]   : %s\r\n", scent->note);
    printf("  [Description] : %s\r\n", scent->description);
    printf("  [Steps per ml]: %d\r\n", scent->steps_per_unit);
    printf("===========================================\r\n");
}

// -----------------------------------------------------------------
// 향료 정보 출력 함수 END
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// 기분별 향수 제조 함수 START
// -----------------------------------------------------------------
void make_perfume_by_mood(Mood_ID_t mood_id)
{
    // 1. 유효성 검사
    if (mood_id >= MOOD_COUNT) {
        printf("Error: Invalid Mood ID.\r\n");
        return;
    }

    const Perfume_Recipe_t *recipe = &RECIPE_DB[mood_id];

    printf("\n\n--- [NEW ORDER] 기분: %s 를 위한 향수 제조 시작 (총 %d가지 재료) ---\r\n",
           recipe->mood_name, recipe->ingredient_count);

    // 2. 레시피 순차 실행
    for (int i = 0; i < recipe->ingredient_count; i++)
    {
        Ingredient_t ingredient = recipe->ingredients[i];

        printf("  [Step %d] %s 주입: %.2f ml\r\n",
               i + 1, SCENT_DB[ingredient.id].name, ingredient.volume_ml);

        // 기존의 향료 주입 함수 호출 (회전 + 펌핑)
        mix_perfume_dose(ingredient.id, ingredient.volume_ml);
    }

    printf("--- 향수 제조 완료! (기분: %s) ---\r\n", recipe->mood_name);
}
// -----------------------------------------------------------------
// 기분별 향수 제조 함수 END
// -----------------------------------------------------------------


int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
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
