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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "ILI9486.h"
#include "string.h"
#include "mainpp.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float KP;
	float KI;
	float KD;
	int error;
	int I_error;
	int D_error;
	int lasterror;
} PID_Variables;

typedef struct{
	int16_t setpoint;
	int16_t Out;
	int16_t en_speed;
} motor_variables;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define motor1_a GPIOA, GPIO_PIN_12
#define motor1_b GPIOA, GPIO_PIN_11
#define motor2_a GPIOB, GPIO_PIN_12
#define motor2_b GPIOB, GPIO_PIN_11
#define motor3_a GPIOB, GPIO_PIN_2
#define motor3_b GPIOB, GPIO_PIN_1
#define motor4_a GPIOB, GPIO_PIN_13
#define motor4_b GPIOC, GPIO_PIN_4
#define motors_a GPIOF, GPIO_PIN_5
#define motors_b GPIOF, GPIO_PIN_4
#define Ball_kick HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, RESET)
#define Ball_in HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, SET)
#define Seedling_get HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, RESET)
#define Seedling_get_1 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, RESET)
#define Seedling_take_1 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, SET)
#define Seedling_take_2 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, SET)
#define Ball_switch1 HAL_GPIO_ReadPin(Switch_BALL1_GPIO_Port, Switch_BALL1_Pin)
#define Ball_switch2 HAL_GPIO_ReadPin(Switch_BALL2_GPIO_Port, Switch_BALL2_Pin)
#define Up_switch1 HAL_GPIO_ReadPin(Switch_UP1_GPIO_Port, Switch_UP1_Pin)
#define Up_switch2 HAL_GPIO_ReadPin(Switch_UP2_GPIO_Port, Switch_UP2_Pin)
#define Down_switch1 HAL_GPIO_ReadPin(Switch_DOWN1_GPIO_Port, Switch_DOWN1_Pin)
#define Down_switch2 HAL_GPIO_ReadPin(Switch_DOWN2_GPIO_Port, Switch_DOWN2_Pin)
#define uragsh 10
#define hoish 6
#define zogs 8
#define max 10
#define CAN1_DEVICE_NUM     4
#define FIRST_GROUP_ID      0x200
#define MOTOR_SPEED_MAX     12000
#define CAN_DATA_SIZE       8
#define CAN1_RX_ID_START    0x201
#define MOTOR_ID            4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;
osThreadId myTask07Handle;
osThreadId myTask08Handle;
/* USER CODE BEGIN PV */
double PI = 3.14159;
double power;			//joysticknii hodolson zai (robotiin hurd)
double alpha;			//zuun joysticknii hevtee tenhlegtei haritsangui ontsog 0-2PI hoorond

uint8_t RX[9];			/*garaas medeelel avah husnegt*/
uint8_t TX[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  /* gar luu command ogoh togtmol utgat husnegt */

uint16_t lf;
uint16_t lb;
uint16_t rf;
uint16_t rb;
uint16_t x_g[9] = {0,60,120,180,240,300,360,420,480};
uint16_t y_g[9] = {0,40,80,120,160,200,240, 280, 320};

int8_t sensor_buff[2];

int16_t count_1;
int16_t count_2;
int16_t count_3;
int16_t count_4;
int16_t oldpos_1[10] ={0,0,0,0,0,0,0,0,0,0};
int16_t oldpos_2[10] ={0,0,0,0,0,0,0,0,0,0};
int16_t oldpos_3[10] ={0,0,0,0,0,0,0,0,0,0};
int16_t oldpos_4[10] ={0,0,0,0,0,0,0,0,0,0};

int x;					//joysticknii x utga
int y;					//joysticknii y utga
int r;					//baruun joysticknii X utga (ergeh chiglel, hurd)
int fl=0;
int left_front;
int left_back;
int right_front;
int right_back;
int ML;					//robotiin tov deer ehtei 45 hem ergesen coordinatiin system deerh zuun motoruudiin buulgalt
int MR;					//robotiin tov deer ehtei 45 hem ergesen coordinatiin system deerh baruun motoruudiin buulgalt
int seedling_switch_deed_1;
int seedling_switch_deed_2;
int bombog_switch_1;
int bombog_switch_2;
int seedling_switch_dood_1;
int seedling_switch_dood_2;
int mode=2;
int pwm_val;
int vel_up;
int en_speed_1;
int en_speed_2;
int en_speed_3;
int en_speed_4;
int indx = 0;
int older_1=0;
int older_2=0;
int older_3=0;
int older_4=0;
int switchStatePrev_1;
int switchStatePrev_2;
int switchStatePrev_3;
int switchStatePrev_4;
int switchStatePrev_5;
int switchStatePrev_6;
int space = 6;
int space_1 = 3;
int i=0;
int ii = 10000;
int can_flag=0;
int can_time;
double M_can_pos;
int M_can_target;

PID_Variables M1_pid;
PID_Variables M2_pid;
PID_Variables M3_pid;
PID_Variables M4_pid;
PID_Variables M5_pid;
PID_Variables M5_pos_pid;

motor_variables M1;
motor_variables M2;
motor_variables M3;
motor_variables M4;
motor_variables M5;

uint32_t TxMailbox;
uint8_t TxData[8];
uint8_t RxData[8];
int16_t speed;

char m1_hurd[4][max];
char flag[6]={0,0,0,0,0,0} ;
char pwm[4][max];
char ff[6]={0,0,0,0,0,0};

extern int right_x;
extern double left_x, left_y;
extern int comma;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
void Joystick_task(void const * argument);
void Ball_task(void const * argument);
void Seedling_task(void const * argument);
void Wheel_task(void const * argument);
void Encoder_task(void const * argument);
void Display_task(void const * argument);
void Ros_Transmit(void const * argument);
void M3508_task(void const * argument);

/* USER CODE BEGIN PFP */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void can_transmit(CAN_HandleTypeDef* hcan1, uint16_t id, int16_t msg1);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SemaphoreHandle_t switchSemaphore;
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_SPI4_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);   //BRUSHLESS
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, 1);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1);
    TIM5->CCR3=zogs;

    M1_pid.KP=0.45777;
    M1_pid.KI=0.0;
    M1_pid.KD=0;

    M2_pid.KP=0.46314;
    M2_pid.KI=0.0;
    M2_pid.KD=0.10;

    M3_pid.KP=0.41351;
    M3_pid.KI=0.0;
    M3_pid.KD=0.1;

    M4_pid.KP=0.47799;
    M4_pid.KI=0.0;
    M4_pid.KD=0.10;



    setup();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, Joystick_task, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, Ball_task, osPriorityNormal, 0, 512);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, Seedling_task, osPriorityNormal, 0, 512);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, Wheel_task, osPriorityNormal, 0, 1024);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, Encoder_task, osPriorityNormal, 0, 512);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, Display_task, osPriorityNormal, 0, 512);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* definition and creation of myTask07 */
  osThreadDef(myTask07, Ros_Transmit, osPriorityNormal, 0, 512);
  myTask07Handle = osThreadCreate(osThread(myTask07), NULL);

  /* definition and creation of myTask08 */
  osThreadDef(myTask08, M3508_task, osPriorityNormal, 0, 256);
  myTask08Handle = osThreadCreate(osThread(myTask08), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

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
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x205;
    sFilterConfig.FilterIdLow = 0x200;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
      	Error_Handler();
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      	Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      	Error_Handler();
    }
  /* USER CODE END CAN1_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 427;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 21600-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 1686;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 127;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

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
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, TFT_BL_Pin|TFT_CS_Pin|RELAY_1_Pin|MOTOR6_A_Pin
                          |RELAY_5_Pin|RELAY_6_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3|MOTOR5_B_Pin|MOTOR5_A_Pin|TFT_DC_Pin
                          |TFT_RST_Pin|MOTOR6_B_Pin|RELAY_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR4_B_GPIO_Port, MOTOR4_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GREEN_USER_LED_Pin|MOTOR3_B_Pin|MOTOR3_A_Pin|MOTOR2_B_Pin
                          |MOTOR2_A_Pin|MOTOR4_A_Pin|blue_user_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RELAY_2_Pin|RELAY_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CHIPSELECT_Pin|MOTOR1_B_Pin|MOTOR1_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TFT_BL_Pin TFT_CS_Pin RELAY_1_Pin MOTOR6_A_Pin
                           RELAY_5_Pin RELAY_6_Pin PE15 */
  GPIO_InitStruct.Pin = TFT_BL_Pin|TFT_CS_Pin|RELAY_1_Pin|MOTOR6_A_Pin
                          |RELAY_5_Pin|RELAY_6_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF3 MOTOR5_B_Pin MOTOR5_A_Pin TFT_DC_Pin
                           TFT_RST_Pin MOTOR6_B_Pin RELAY_4_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|MOTOR5_B_Pin|MOTOR5_A_Pin|TFT_DC_Pin
                          |TFT_RST_Pin|MOTOR6_B_Pin|RELAY_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Switch_UP2_Pin Switch_BALL1_Pin Switch_DOWN2_Pin */
  GPIO_InitStruct.Pin = Switch_UP2_Pin|Switch_BALL1_Pin|Switch_DOWN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Switch_BALL2_Pin */
  GPIO_InitStruct.Pin = Switch_BALL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Switch_BALL2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR4_B_Pin */
  GPIO_InitStruct.Pin = MOTOR4_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR4_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_USER_LED_Pin MOTOR3_B_Pin MOTOR3_A_Pin MOTOR2_B_Pin
                           MOTOR2_A_Pin MOTOR4_A_Pin blue_user_led_Pin */
  GPIO_InitStruct.Pin = GREEN_USER_LED_Pin|MOTOR3_B_Pin|MOTOR3_A_Pin|MOTOR2_B_Pin
                          |MOTOR2_A_Pin|MOTOR4_A_Pin|blue_user_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_2_Pin RELAY_3_Pin */
  GPIO_InitStruct.Pin = RELAY_2_Pin|RELAY_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Switch_UP1_Pin */
  GPIO_InitStruct.Pin = Switch_UP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Switch_UP1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CHIPSELECT_Pin MOTOR1_B_Pin MOTOR1_A_Pin */
  GPIO_InitStruct.Pin = SPI_CHIPSELECT_Pin|MOTOR1_B_Pin|MOTOR1_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Switch_DOWN1_Pin PD3 */
  GPIO_InitStruct.Pin = Switch_DOWN1_Pin|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */

  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
//void seedling_check(){
//	if(RX[3]==127){
//		x=-300;
//	}
//	else if(RX[3]==239){
//		y=300;
//	}
//	else if(RX[3]==191){
//		y=-300;
//	}
//	else if(RX[3]==223){
//		x=300;
//	}


//		power = hypot(x,y);
//		if(power>1000){
//			power=1000;
//		}
//		alpha = atan2(y,x);
//		ML = sin(alpha-PI/4)*power;
//		MR = cos(alpha-PI/4)*power;
//
//		M1.setpoint=MR;
//		M2.setpoint=ML;
//		M3.setpoint=ML;
//		M4.setpoint=MR;
//}
void switch_StateChanged_Ball(){
    if (switchStatePrev_3 != bombog_switch_1) {
        flag[2] = 1;
    } else {
        flag[2] = 0;
    }
    if (switchStatePrev_4 != bombog_switch_2) {
        flag[3] = 1;
    } else {
        flag[3] = 0;
    }
}
void motor_speed(){
	int pos1=0;
	int pos2=0;
	int pos3=0;
	int pos4=0;
	for(int i = 0; i < 10; i++){
		pos1 += oldpos_1[i];
		pos2 += oldpos_2[i];
		pos3 += oldpos_3[i];
		pos4 += oldpos_4[i];
	}
	M1.en_speed = pos1/10;
	M2.en_speed = pos2/10;
	M3.en_speed = pos3/10;
	M4.en_speed = pos4/10;
}
void motor1(){
	if(left_front>0){
		HAL_GPIO_WritePin(motor1_a, 0);
		HAL_GPIO_WritePin(motor1_b, 1);
		TIM3->CCR4=lf;
	}
	else if(left_front<0){
		HAL_GPIO_WritePin(motor1_a, 1);
		HAL_GPIO_WritePin(motor1_b, 0);
		TIM3->CCR4=lf;
	}
	else{
		HAL_GPIO_WritePin(motor1_a, 0);
		HAL_GPIO_WritePin(motor1_b, 0);
		TIM3->CCR4=lf;
	}
}

void motor2(){
	if(left_back>0){
		HAL_GPIO_WritePin(motor2_a, 0);
		HAL_GPIO_WritePin(motor2_b, 1);
		TIM3->CCR3=lb;
	}
	else if(left_back<0){
		HAL_GPIO_WritePin(motor2_a, 1);
		HAL_GPIO_WritePin(motor2_b, 0);
		TIM3->CCR3=lb;
	}
	else{
		HAL_GPIO_WritePin(motor2_a, 0);
		HAL_GPIO_WritePin(motor2_b, 0);
		TIM3->CCR3=lb;
	}
}

void motor3(){
	if(right_front>0){
		HAL_GPIO_WritePin(motor3_a, 0);
		HAL_GPIO_WritePin(motor3_b, 1);
		TIM3->CCR2=rf;
	}
	else if(right_front<0){
		HAL_GPIO_WritePin(motor3_a, 1);
		HAL_GPIO_WritePin(motor3_b, 0);
		TIM3->CCR2=rf;
	}
	else{
		HAL_GPIO_WritePin(motor3_a, 0);
		HAL_GPIO_WritePin(motor3_b, 0);
		TIM3->CCR2=rf;
	}
}

void motor4(){
	if(right_back>0){
		HAL_GPIO_WritePin(motor4_a, 1);
		HAL_GPIO_WritePin(motor4_b, 0);
		TIM3->CCR1=rb;
	}
	else if(right_back<0){
		HAL_GPIO_WritePin(motor4_a, 0);
		HAL_GPIO_WritePin(motor4_b, 1);
		TIM3->CCR1=rb;
	}
	else{
		HAL_GPIO_WritePin(motor4_a, 0);
		HAL_GPIO_WritePin(motor4_b, 0);
		TIM3->CCR1=rb;
	}
}

/*************************************************************************************/

void turn(){
	if(r>0){
		HAL_GPIO_WritePin(motor1_a, 0);
		HAL_GPIO_WritePin(motor1_b, 1);
		TIM3->CCR4= abs(r);
		HAL_GPIO_WritePin(motor2_a, 0);
		HAL_GPIO_WritePin(motor2_b, 1);
		TIM3->CCR3= abs(r);
		HAL_GPIO_WritePin(motor3_a, 1);
		HAL_GPIO_WritePin(motor3_b, 0);
		TIM3->CCR2= abs(r);
		HAL_GPIO_WritePin(motor4_a, 0);
		HAL_GPIO_WritePin(motor4_b, 1);
		TIM3->CCR1= abs(r);
	}
	else if(r<0){
		HAL_GPIO_WritePin(motor1_a, 1);
		HAL_GPIO_WritePin(motor1_b, 0);
		TIM3->CCR4= abs(r);
		HAL_GPIO_WritePin(motor2_a, 1);
		HAL_GPIO_WritePin(motor2_b, 0);
		TIM3->CCR3= abs(r);
		HAL_GPIO_WritePin(motor3_a, 0);
		HAL_GPIO_WritePin(motor3_b, 1);
		TIM3->CCR2= abs(r);
		HAL_GPIO_WritePin(motor4_a, 1);
		HAL_GPIO_WritePin(motor4_b, 0);
		TIM3->CCR1= abs(r);
	}
}

/*******************************************************************************/

void yvj_ergeh(){
	if(r<0){
		lf = abs(left_front+r*1.5);
		lb = abs(left_back+r*1.5);
	}
	if(r>0){
		rf = abs(right_front-r*1.5);
		rb = abs(right_back-r*1.5);
	}
	motor1();
	motor2();
	motor3();
	motor4();
}
int GetPos_M5(){
	return M_can_pos/1000;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, RxData) == HAL_OK){
		if(rxHeader.StdId == 0x201){
			int can_prev_time=can_time;
			can_time=HAL_GetTick();
			double delta_time=can_time-can_prev_time;
			can_flag=1;
//			if(i < 100000){
//				i++;
//			}else{
//				i=0;
//			}
			M5.en_speed = RxData[2] << 8;
			M5.en_speed = M5.en_speed  + RxData[3];
			delta_time=delta_time*M5.en_speed;
//			speed = speed * 1.5;ss
			M_can_pos=M_can_pos+(delta_time/198);
//			HAL_GetTick();
		}
	}

}
void can_transmit(CAN_HandleTypeDef* hcan, uint16_t id, int16_t msg1){
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             data[8];
    uint32_t            pTxMailbox;

    tx_header.StdId = id;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = CAN_DATA_SIZE;
    tx_header.TransmitGlobalTime = DISABLE;
    data[0] = msg1 >> 8;
    data[1] = msg1;
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &pTxMailbox) == HAL_OK){
    	while (HAL_CAN_IsTxMessagePending(hcan, pTxMailbox));
//    	Error_Handler();
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Joystick_task */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Joystick_task */
void Joystick_task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);	//SPI chip select
//		HAL_SPI_TransmitReceive(&hspi1, TX, RX, 9, 10); //full duplexeer medeelel avah RX deer hadgalah
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,SET);
//		if(comma == 0){
//			y=left_y*7.8125;//(128-RX[8])*7.8125;
//			x=left_x*7.8125;//(RX[7]-128)*7.8125;
//			r=right_x*1.828593664;//(RX[5]-128);
//		}
//
//		else if(comma == 10){
//			y=(128-RX[8])*2.62467;//(128-RX[8])*7.8125;
//			x=(RX[7]-128)*2.62467;//(RX[7]-128)*7.8125;
//			r=(RX[5]-128);//(RX[5]-128);
//		}
//		else if(RX[3] == 254){
//			y=(128-RX[8])*7.8125;//(128-RX[8])*2.62467;
//			x=(RX[7]-128)*7.8125;//(RX[7]-128)*2.62467;
//			r=(RX[5]-128);//(RX[5]-128);
//		}
//
//
//		power = hypot(x,y);
//		if(power>1000){
//			power=1000;
//		}
//		alpha = atan2(y,x);
//		ML = sin(alpha-PI/4)*power;
//		MR = cos(alpha-PI/4)*power;
//
//		M1.setpoint=MR;
//		M2.setpoint=ML;
//		M3.setpoint=ML;
//		M4.setpoint=MR;
//		if(can_flag==1){
//			can_flag=0;
//			can_transmit(&hcan1, FIRST_GROUP_ID, 0);
	  M_can_target=0;
	  osDelay(1000);
	  M_can_target=-11;
	  osDelay(5000);
	  M_can_target=-6;
	  osDelay(5000);

//			ii=ii-20;
//		}
//		osDelay(10);


//		can_transmit(&hcan1, FIRST_GROUP_ID, 0);
//		osDelay(2000);
//		can_transmit(&hcan1, FIRST_GROUP_ID, -10000);
//		osDelay(2000);
//		can_transmit(&hcan1, FIRST_GROUP_ID, 0);
//		osDelay(2000);
//		if(i%1000 == 0){
//			HAL_GPIO_TogglePin(blue_user_led_GPIO_Port, blue_user_led_Pin);
//		}else
//			HAL_GPIO_TogglePin(blue_user_led_GPIO_Port, blue_user_led_Pin);
//		osDelay(1);
//    osDelay(30);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Ball_task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ball_task */
void Ball_task(void const * argument)
{
  /* USER CODE BEGIN Ball_task */
  /* Infinite loop */
  for(;;)
  {
 	      if (Ball_switch1 != switchStatePrev_3) {
 	          if (Ball_switch1 == GPIO_PIN_SET) {
 	              ff[2]=0;
 	          } else {
 	              ff[2]++;
 	          }
 	      }
 	      if (ff[2] != 0) {
 	          flag[2] = 2;
 	      }
 	      if (Ball_switch2 != switchStatePrev_4) {
 	          if (Ball_switch2 == GPIO_PIN_SET) {
 	              ff[3]=0;
 	          } else {
 	              ff[3]++;
 	          }
 	      }
 	      if (ff[3] != 0) {
 	          flag[3] = 2;
 	      }
 	      if(((Ball_switch1 == 1) || (Ball_switch2 == 1)) && (RX[6] > 240)){
		  	TIM5->CCR3=zogs;
		  	osDelay(200);
	  	  	TIM5->CCR3=uragsh;
	  	  	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, SET);
	  	  }
 	      if(((Ball_switch1 == 0) || (Ball_switch2 == 0)) && (RX[3] == 253)){
	  	  TIM5->CCR3=zogs;
	  	  }
	  	  if(RX[6]<10){
	  		TIM5->CCR3=zogs;
	  		osDelay(200);
	  		TIM5->CCR3=hoish;
	  	  }
/**********************     RELAY BOMBOG    ************************************/
	  	  if(RX[4]==251){
	  		  Ball_kick;
	  		  osDelay(1000);
	  		  Ball_in;
	  	  		  }
			  osDelay(30);
  }



  /* USER CODE END Ball_task */
}

/* USER CODE BEGIN Header_Seedling_task */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Seedling_task */
void Seedling_task(void const * argument)
{
  /* USER CODE BEGIN Seedling_task */
  /* Infinite loop */
  for(;;)
  {
	 	      if (Up_switch1 != switchStatePrev_1) {
	 	          if (Up_switch1 == GPIO_PIN_SET) {
	 	              ff[0]=0;
	 	          } else {
	 	              ff[0]++;
	 	          }
	 	      }
	 	      if (ff[0] != 0) {
	 	          flag[0] = 2;
	 	      }

	 	      if (Up_switch2 != switchStatePrev_2) {
	 	          if (Up_switch2 == GPIO_PIN_SET) {
	 	              ff[1]=0;
	 	          } else {
	 	              ff[1]++;
	 	          }
	 	      }
	 	      if (ff[1] != 0) {
	 	          flag[1] = 2;
	 	      }
	 	      if (Down_switch1 != switchStatePrev_5) {
	 	          if (Down_switch1 == GPIO_PIN_RESET) {
	 	              ff[4]=0;
	 	          } else {
	 	              ff[4]++;
	 	          }
	 	      }
	 	      if (ff[4] != 0) {
	 	          flag[4] = 2;
	 	      }
	 	      if (Down_switch2 != switchStatePrev_6) {
	 	          if (Down_switch2 == GPIO_PIN_RESET) {
	 	              ff[5]=0;
	 	          } else {
	 	              ff[5]++;
	 	          }
	 	      }
	 	      if (ff[5] != 0) {
	 	          flag[5] = 2;
	 	      }

/************************************      SEEDLING ORGOH BUULGAH       **********************************/

//	  if(((Down_switch1 == 0) || (Down_switch2 == 0)) && (RX[4]==239)){
//	  		  mode=1;
//	  	  }
//	  	  if(((Down_switch1 == 0) || (Down_switch2 == 0)) && (RX[4]==255)){
//	  		  mode=2;
//	  	  }
//	  	  if(((Up_switch1 == 0) || (Up_switch2 == 0)) && (RX[4]==255)){
//	  		  mode=0;
//	  	  }
//	  	  if( RX[4]==191){
//	  		  mode=0;
//	  	  }

//	  	  if(mode==0){
//	  		  HAL_GPIO_WritePin(motors_a, 0);
//	  		  HAL_GPIO_WritePin(motors_b, 1);
//	  		  TIM12->CCR2=100/8;
//	  	  }
//	  	  if(mode==2){
//	  		  HAL_GPIO_WritePin(motors_a, 0);
//	  		  HAL_GPIO_WritePin(motors_b, 0);
//	  		  TIM12->CCR2=0;
//	  	  }
//	  	  if(mode==1){
//	  		  HAL_GPIO_WritePin(motors_a, 0);
//	  		  HAL_GPIO_WritePin(motors_b, 1);
//	  		  TIM12->CCR2=100;
//	  	  }
//	  	  if(mode==-1){
//	  		  HAL_GPIO_WritePin(motors_a, 1);
//	  		  HAL_GPIO_WritePin(motors_b, 0);
//	  		  TIM12->CCR2=10;
//	  	  }

//****************************************seedling_motor************************************************/
//	 	      seedling_check();

			  if(((Down_switch1 == 0) || (Down_switch2 == 0)) && (RX[4]==255)){
				  mode=2;
			  }
			  if(((Up_switch1 == 0) || (Up_switch2 == 0)) && (RX[4]==255)){
				  mode=2;
				  if(fl == 1){
					  fl=2;
				  }
				  else if(fl == -1){
					  fl=-2;
				  }
			  }
			  if(((Down_switch1 == 0) || (Down_switch2 == 0)) && (RX[4]==239)){
				  mode=1;
				  fl=1;
			  }
			  if(RX[4] == 191){
				  mode=0;
			  }
			  else if(RX[3] == 254){
				  mode=1;
				  fl=-1;
			  }
//****************************************seedling_relay************************************************/
			  if(((Down_switch1 == 0) || (Down_switch2 == 0)) && (RX[4]==247)){
				  Seedling_get;
				  Seedling_get_1;
			  }
			  if(RX[4]==223){
		  	  	  Seedling_take_1;
		  	  }
		  	  if(RX[4]==127){
		  	  	  Seedling_take_2;
		  	  }


	  	  if(mode==0){
	  		  HAL_GPIO_WritePin(motors_a, 1);
	  		  HAL_GPIO_WritePin(motors_b, 0);
	  		  TIM12->CCR2=8;
	  	  }
	  	  if(mode==2){
	  		  HAL_GPIO_WritePin(motors_a, 0);
	  		  HAL_GPIO_WritePin(motors_b, 0);
	  		  TIM12->CCR2=0;
	  	  }
	  	  if(mode==1){
	  		  HAL_GPIO_WritePin(motors_a, 0);
	  		  HAL_GPIO_WritePin(motors_b, 1);
	  		  TIM12->CCR2=50;
	  	  }

    osDelay(10);
  }
  /* USER CODE END Seedling_task */
}

/* USER CODE BEGIN Header_Wheel_task */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Wheel_task */
void Wheel_task(void const * argument)
{
  /* USER CODE BEGIN Wheel_task */
  /* Infinite loop */
  for(;;)
  {
	  if(vel_up == 1){
	   	  		  vel_up=0;
  	  	  		  M1_pid.error = M1.setpoint-M1.en_speed;
  	  	  		  M1_pid.I_error+=M1_pid.error;
  	  	  		  M1_pid.D_error=M1_pid.lasterror-M1_pid.error;
  	  	  		  M1_pid.lasterror=M1_pid.error;
  	  	  		  M1.Out=M1_pid.KP*M1_pid.error+M1_pid.KI*M1_pid.I_error+M1_pid.KD*M1_pid.D_error;

  	  	  		  M2_pid.error=M2.setpoint-M2.en_speed;
  	  	  		  M2_pid.I_error+=M2_pid.error;
  	  	  		  M2_pid.D_error=M2_pid.lasterror-M2_pid.error;
  	  	  		  M2_pid.lasterror=M2_pid.error;
  	  	  		  M2.Out=M2_pid.KP*M2_pid.error+M2_pid.KI*M2_pid.I_error+M2_pid.KD*M2_pid.D_error;

  	  	  		  M3_pid.error=M3.setpoint-M3.en_speed;
  	  	  		  M3_pid.I_error+=M3_pid.error;
  	  	  		  M3_pid.D_error=M3_pid.lasterror-M3_pid.error;
  	  	  		  M3_pid.lasterror=M3_pid.error;
  	  	  		  M3.Out=M3_pid.KP*M3_pid.error+M3_pid.KI*M3_pid.I_error+M3_pid.KD*M3_pid.D_error;

  	  	  		  M4_pid.error=M4.setpoint-M4.en_speed;
  	  	  		  M4_pid.I_error+=M4_pid.error;
  	  	  		  M4_pid.D_error=M4_pid.lasterror-M4_pid.error;
  	  	  		  M4_pid.lasterror=M4_pid.error;
  	  	  		  M4.Out=M4_pid.KP*M4_pid.error+M4_pid.KI*M4_pid.I_error+M4_pid.KD*M4_pid.D_error;
  	  	      left_front=M1.Out;
			  left_back=M2.Out;
			  right_front=M3.Out;
			  right_back=M4.Out;

			  lf = abs(left_front);
			  lb = abs(left_back);
			  rf = abs(right_front);
			  rb = abs(right_back);

			  if(abs(x)<5 && abs(y)<5 && abs(r)<5){
				  motor1();
				  motor2();
				  motor3();
				  motor4();
			  }
			  else if((abs(x)>5 || abs(y)>5) && abs(r)<5){
				  motor1();
				  motor2();
				  motor3();
				  motor4();
			  }
			  else if((abs(x)<5 && abs(y)<5) && abs(r)>5){
				  turn();
			  }
			  else if((abs(x)>5 || abs(y)>5) && abs(r)>5){
				  yvj_ergeh();
			  }
	  }

    osDelay(10);
  }
  /* USER CODE END Wheel_task */
}

/* USER CODE BEGIN Header_Encoder_task */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Encoder_task */
void Encoder_task(void const * argument)
{
  /* USER CODE BEGIN Encoder_task */
  /* Infinite loop */
  for(;;)
  {
		count_2 = TIM1->CNT;
		count_4 = -(TIM2->CNT);
		count_3 = -(TIM4->CNT);
		count_1 = -(TIM8->CNT);
		oldpos_1[0]=oldpos_1[1];
		oldpos_1[1]=oldpos_1[2];
		oldpos_1[2]=oldpos_1[3];
		oldpos_1[3]=oldpos_1[4];
		oldpos_1[4]=oldpos_1[5];
		oldpos_1[5]=oldpos_1[6];
		oldpos_1[6]=oldpos_1[7];
		oldpos_1[7]=oldpos_1[8];
		oldpos_1[8]=oldpos_1[9];
		oldpos_1[9] =count_1-older_1;
		older_1=count_1;
		oldpos_2[0]=oldpos_2[1];
		oldpos_2[1]=oldpos_2[2];
		oldpos_2[2]=oldpos_2[3];
		oldpos_2[3]=oldpos_2[4];
		oldpos_2[4]=oldpos_2[5];
		oldpos_2[5]=oldpos_2[6];
		oldpos_2[6]=oldpos_2[7];
		oldpos_2[7]=oldpos_2[8];
		oldpos_2[8]=oldpos_2[9];
		oldpos_2[9] =count_2-older_2;
		older_2=count_2;
		oldpos_3[0]=oldpos_3[1];
		oldpos_3[1]=oldpos_3[2];
		oldpos_3[2]=oldpos_3[3];
		oldpos_3[3]=oldpos_3[4];
		oldpos_3[4]=oldpos_3[5];
		oldpos_3[5]=oldpos_3[6];
		oldpos_3[6]=oldpos_3[7];
		oldpos_3[7]=oldpos_3[8];
		oldpos_3[8]=oldpos_3[9];
		oldpos_3[9] =count_3-older_3;
		older_3=count_3;
		oldpos_4[0]=oldpos_4[1];
		oldpos_4[1]=oldpos_4[2];
		oldpos_4[2]=oldpos_4[3];
		oldpos_4[3]=oldpos_4[4];
		oldpos_4[4]=oldpos_4[5];
		oldpos_4[5]=oldpos_4[6];
		oldpos_4[6]=oldpos_4[7];
		oldpos_4[7]=oldpos_4[8];
		oldpos_4[8]=oldpos_4[9];
		oldpos_4[9] =count_4-older_4;
		older_4=count_4;
		vel_up=1;
		motor_speed();


        osDelay(1);
  }
  /* USER CODE END Encoder_task */
}

/* USER CODE BEGIN Header_Display_task */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Display_task */
void Display_task(void const * argument)
{
  /* USER CODE BEGIN Display_task */
  /* Infinite loop */
    LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;
    LCD_Init( Lcd_ScanDir, 1000);
    LCD_SetBackLight();
    LCD_Clear(LCD_BACKGROUND);
    GUI_display();
  for(;;)
  {
	  /******************************JOYSTICK************************************/
	  if((RX[1] == 65) || (RX[1] == 0)){
	 		  GUI_DrawRectangle(x_g[6], y_g[2]+space, (x_g[6]+x_g[7])/2, (y_g[2]+y_g[3])/2-space, RED, DRAW_FULL, DOT_PIXEL_DFT);
	 	  }
	 	  else{
	 		  GUI_DrawRectangle(x_g[6], y_g[2]+space, (x_g[6]+x_g[7])/2, (y_g[2]+y_g[3])/2-space, GREEN, DRAW_FULL, DOT_PIXEL_DFT);
	 	  }
/******************************SWITCH************************************/
	  if(Up_switch1 == 1){
			GUI_DrawRectangle(x_g[0]+space, y_g[1]+space, x_g[1]-space, y_g[2]-space, WHITE, DRAW_FULL, DOT_PIXEL_DFT);
			fflush(stdout);
	  }
	  if(Up_switch1 == 0){
			GUI_DrawRectangle(x_g[0]+space, y_g[1]+space, x_g[1]-space, y_g[2]-space, GREEN, DRAW_FULL, DOT_PIXEL_DFT);
			fflush(stdout);
	  }
	  if(flag[0] == 2){
		  GUI_DrawRectangle(x_g[0]+space_1, y_g[1]+space_1, x_g[1]-space_1, y_g[2]-space_1, WHITE, DRAW_EMPTY, DOT_PIXEL_DFT);
		  fflush(stdout);
	  }
	  if(flag[1] == 2){
		  GUI_DrawRectangle(x_g[3]+space_1, y_g[1]+space_1, x_g[4]-space_1, y_g[2]-space_1, WHITE, DRAW_EMPTY, DOT_PIXEL_DFT);
		  fflush(stdout);
	  }
	  if(Up_switch2 == 1){
		  GUI_DrawRectangle(x_g[3]+space, y_g[1]+space, x_g[4]-space, y_g[2]-space, WHITE, DRAW_FULL, DOT_PIXEL_DFT);
			fflush(stdout);
	  }
	  if(Up_switch2 == 0){
		  GUI_DrawRectangle(x_g[3]+space, y_g[1]+space, x_g[4]-space, y_g[2]-space, GREEN, DRAW_FULL, DOT_PIXEL_DFT);
			fflush(stdout);
	  }
	  if(flag[3] == 2){
		  GUI_DrawCircle((x_g[2]+x_g[1])/2, ((y_g[3])+(y_g[1]))/2, 20, WHITE, DRAW_EMPTY, DOT_PIXEL_DFT);
		  fflush(stdout);
	  }
	  if(Ball_switch2 == 1){
		  GUI_DrawCircle((x_g[2]+x_g[1])/2, ((y_g[3])+(y_g[1]))/2, 15, WHITE, DRAW_FULL, DOT_PIXEL_DFT);
			fflush(stdout);
	  }
	  if(Ball_switch2 == 0){
		  GUI_DrawCircle((x_g[2]+x_g[1])/2, ((y_g[3])+(y_g[1]))/2, 15, GREEN, DRAW_FULL, DOT_PIXEL_DFT);
			fflush(stdout);
	  }
	  if(flag[2] == 2){
		  GUI_DrawCircle((x_g[3]+x_g[2])/2, ((y_g[3])+(y_g[1]))/2, 20, WHITE, DRAW_EMPTY, DOT_PIXEL_DFT);
		  fflush(stdout);
	  }
	  if(Ball_switch1 == 1){
		  GUI_DrawCircle((x_g[3]+x_g[2])/2, ((y_g[3])+(y_g[1]))/2, 15, WHITE, DRAW_FULL, DOT_PIXEL_DFT);
			fflush(stdout);
	  }
	  if(Ball_switch1 == 0){
		  GUI_DrawCircle((x_g[3]+x_g[2])/2, ((y_g[3])+(y_g[1]))/2, 15, GREEN, DRAW_FULL, DOT_PIXEL_DFT);
			fflush(stdout);
	  }
	  if(flag[5] == 2){
		  GUI_DrawRectangle(x_g[0]+space_1, y_g[2]+space_1, x_g[1]-space_1, y_g[3]-space_1, WHITE, DRAW_EMPTY, DOT_PIXEL_DFT);
		  fflush(stdout);
	  }
	  if(Down_switch2 == 1){
		  GUI_DrawRectangle(x_g[0]+space, y_g[2]+space, x_g[1]-space, y_g[3]-space, WHITE, DRAW_FULL, DOT_PIXEL_DFT);
			fflush(stdout);
	  }
	  if(Down_switch2 == 0){
		  GUI_DrawRectangle(x_g[0]+space, y_g[2]+space, x_g[1]-space, y_g[3]-space, GREEN, DRAW_FULL, DOT_PIXEL_DFT);
			fflush(stdout);
	  }
	  if(flag[4] == 2){
		  GUI_DrawRectangle(x_g[3]+space_1, y_g[2]+space_1, x_g[4]-space_1, y_g[3]-space_1, WHITE, DRAW_EMPTY, DOT_PIXEL_DFT);
		  fflush(stdout);
	  }
	  if(Down_switch1 == 1){
		  GUI_DrawRectangle(x_g[3]+space, y_g[2]+space, x_g[4]-space, y_g[3]-space, WHITE, DRAW_FULL, DOT_PIXEL_DFT);
			fflush(stdout);
	  }
	  if(Down_switch1 == 0){
		  GUI_DrawRectangle(x_g[3]+space, y_g[2]+space, x_g[4]-space, y_g[3]-space, GREEN, DRAW_FULL, DOT_PIXEL_DFT);
			fflush(stdout);
	  }
	  /******************************RELAY************************************/
	  if((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)) == 0){
		  GUI_DrawRectangle((x_g[4]+x_g[5])/2+space, y_g[1]+space, (x_g[5]+x_g[6])/2-space, y_g[2]-space, GRAY, DRAW_FULL, DOT_PIXEL_DFT);
	  }
	  else{
		  GUI_DrawRectangle((x_g[4]+x_g[5])/2+space, y_g[1]+space, (x_g[5]+x_g[6])/2-space, y_g[2]-space, RED, DRAW_FULL, DOT_PIXEL_DFT);
	  }
	  if( (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11)) == 0){
		  GUI_DrawRectangle((x_g[6]+x_g[7])/2+space, y_g[1]+space, (x_g[7]+x_g[8])/2-space, y_g[2]-space, GRAY, DRAW_FULL, DOT_PIXEL_DFT);
	  }
	  else{
		  GUI_DrawRectangle((x_g[6]+x_g[7])/2+space, y_g[1]+space, (x_g[7]+x_g[8])/2-space, y_g[2]-space, RED, DRAW_FULL, DOT_PIXEL_DFT);
	  }
	  if( (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3)) == 0){
		  GUI_DrawRectangle((x_g[5]+x_g[6])/2+space, y_g[1]+space, (x_g[6]+x_g[7])/2-space, y_g[2]-space, GRAY, DRAW_FULL, DOT_PIXEL_DFT);
	  }
	  else{
		  GUI_DrawRectangle((x_g[5]+x_g[6])/2+space, y_g[1]+space, (x_g[6]+x_g[7])/2-space, y_g[2]-space, RED, DRAW_FULL, DOT_PIXEL_DFT);
	  }
/******************************ROBOT************************************/
	  	sprintf(m1_hurd[0], "%d   ", M1.en_speed);
	  	sprintf(m1_hurd[1], "%d   ", M2.en_speed);
	  	sprintf(m1_hurd[2], "%d   ", M3.en_speed);
	  	sprintf(m1_hurd[3], "%d   ", M4.en_speed);
	  	sprintf(pwm[0], "%u   ", lf);
	  	sprintf(pwm[1], "%u   ", lb);
	  	sprintf(pwm[2], "%u   ", rf);
	  	sprintf(pwm[3], "%u   ", rb);
		switchStatePrev_1 = Up_switch2;
		switchStatePrev_2 = Up_switch2;
		switchStatePrev_3 = Ball_switch2;
		switchStatePrev_4 = Ball_switch2;

		GUI_DisString_EN(x_g[1]+space, y_g[4], m1_hurd[0], &Font16, LCD_BACKGROUND, WHITE);
		GUI_DisString_EN(x_g[1]+space, y_g[4]+5*space, pwm[0], &Font16, LCD_BACKGROUND, WHITE);

		GUI_DisString_EN(x_g[7]+space, y_g[4], m1_hurd[2], &Font16, LCD_BACKGROUND, WHITE);
		GUI_DisString_EN(x_g[7]+space, y_g[4]+5*space, pwm[2], &Font16, LCD_BACKGROUND, WHITE);

		GUI_DisString_EN(x_g[1]+space, y_g[6]+2*space, m1_hurd[1], &Font16, LCD_BACKGROUND, WHITE);
		GUI_DisString_EN(x_g[1]+space, y_g[6]+6*space, pwm[1], &Font16, LCD_BACKGROUND, WHITE);

		GUI_DisString_EN(x_g[7]+space, y_g[6]+2*space, m1_hurd[3], &Font16, LCD_BACKGROUND, WHITE);
		GUI_DisString_EN(x_g[7]+space, y_g[6]+6*space, pwm[3], &Font16, LCD_BACKGROUND, WHITE);
/******************************JOYSTICK CHIGLEL************************************/
		if((x_g[4] > (x_g[4]+(RX[7]-128))) || (y_g[6] > (y_g[6]+(128-RX[8])))){
			GUI_DrawLine(x_g[4]+(RX[7]-128)/4, y_g[6]+(RX[8]-128)/4 , x_g[4], y_g[6], MAGENTA, LINE_SOLID, DOT_PIXEL_2X2);
			GUI_DrawPoint(x_g[4]+(RX[7]-128)/4, y_g[6]+(RX[8]-128)/4, MAGENTA, DOT_PIXEL_3X3, DOT_STYLE_DFT);
			GUI_DrawRectangle(x_g[3], y_g[4]+space, x_g[5], y_g[8]-space, BLACK, DRAW_FULL, DOT_PIXEL_DFT);
		}
		else if((x_g[4] < (x_g[4]+(RX[7]-128))) || (y_g[6] < (y_g[6]+(128-RX[8])))){
			GUI_DrawLine(x_g[4], y_g[6] , x_g[4]+(RX[7]-128)/4, y_g[6]+(RX[8]-128)/4, MAGENTA, LINE_SOLID, DOT_PIXEL_2X2);
			GUI_DrawPoint(x_g[4]+(RX[7]-128)/4, y_g[6]+(RX[8]-128)/4, MAGENTA, DOT_PIXEL_3X3, DOT_STYLE_DFT);
			GUI_DrawRectangle(x_g[3], y_g[4]+space, x_g[5], y_g[8]-space, BLACK, DRAW_FULL, DOT_PIXEL_DFT);
		}

    osDelay(10);
  }
  /* USER CODE END Display_task */
}

/* USER CODE BEGIN Header_Ros_Transmit */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ros_Transmit */
void Ros_Transmit(void const * argument)
{
  /* USER CODE BEGIN Ros_Transmit */
  /* Infinite loop */
  for(;;)
  {

	  if(Up_switch1==0){
	  		  sensor_buff[0]=sensor_buff[0]|0b00000001;
	  	  }
	  	  else{
	  		  sensor_buff[0]=sensor_buff[0]&0b11111110;
	  	  }
	  	  if(Up_switch2==0){
	  		  sensor_buff[0]=sensor_buff[0]|0b00000010;
	  	  }
	  	  else{
	  		  sensor_buff[0]=sensor_buff[0]&0b11111101;
	  	  }
	  	  if(Down_switch1==0){
	  		  sensor_buff[0]=sensor_buff[0]|0b00000100;
	  	  }
	  	  else{
	  		  sensor_buff[0]=sensor_buff[0]&0b11111011;
	  	  }
	  	  if(Down_switch2==0){
	  		  sensor_buff[0]=sensor_buff[0]|0b00001000;
	  	  }
	  	  else{
	  		  sensor_buff[0]=sensor_buff[0]&0b11110111;
	  	  }
	  	  if(Ball_switch1==0){
	  		  sensor_buff[0]=sensor_buff[0]|0b00010000;
	  	  }
	  	  else{
	  		  sensor_buff[0]=sensor_buff[0]&0b11101111;
	  	  }
	  	  if(fl==2){
	  		  sensor_buff[0]=sensor_buff[0]|0b00100000;
	  	  }
	  	  else if(fl==-2){
	  		  sensor_buff[0]=sensor_buff[0]&0b11011111;
	  	  }
	  	  if(RX[3]==247){
	  		  sensor_buff[0]=sensor_buff[0]|0b01000000;
	  	  }
	  	  else{
	  		  sensor_buff[0]=sensor_buff[0]&0b10111111;
	  	  }
	  	  if(RX[3] == 254){
	  		  sensor_buff[0]=sensor_buff[0]|0b10000000;
	  	  }
	  	  else{
	  		  sensor_buff[0]=sensor_buff[0]&0b01111111;
	  	  }

	  	  loop();
    osDelay(5);
  }
  /* USER CODE END Ros_Transmit */
}

/* USER CODE BEGIN Header_M3508_task */
/**
* @brief Function implementing the myTask08 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_M3508_task */
void M3508_task(void const * argument)
{
  /* USER CODE BEGIN M3508_task */
    M5_pid.KP=1.5;
    M5_pid.KI=0.000;
    M5_pid.KD=0.02;
    M5_pos_pid.KP=1.5;
    M5_pos_pid.KI=0;
    M5_pos_pid.KD=0;
  /* Infinite loop */
  for(;;)
  {
	M5_pos_pid.error=M_can_target*1000-M_can_pos;
	M5.setpoint=M5_pos_pid.KP*M5_pos_pid.error;
	if(M5.setpoint>8000){
		M5.setpoint=8000;
	}
	if(M5.setpoint<-8000){
		M5.setpoint=-8000;
	}
	  M5_pid.error=M5.setpoint-M5.en_speed;
	  M5_pid.I_error+=M5_pid.error;
	  M5_pid.D_error=M5_pid.lasterror-M5_pid.error;
	  M5_pid.lasterror=M5_pid.error;
	  M5.Out=M5_pid.KP*M5_pid.error+M5_pid.KI*M5_pid.I_error+M5_pid.KD*M5_pid.D_error;
	  can_transmit(&hcan1, FIRST_GROUP_ID, M5.Out);
	  osDelay(2);
  }
  /* USER CODE END M3508_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
//  while (1)
//  {
//  }
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
