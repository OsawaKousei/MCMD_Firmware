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
#include "string.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <CAN_Main.h>
#include <UDPController.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

CAN_HandleTypeDef hcan1;

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 512 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId systemCheckTaskHandle;
uint32_t systemCheckTaskBuffer[ 512 ];
osStaticThreadDef_t systemCheckTaskControlBlock;
/* USER CODE BEGIN PV */
//CANモジュール基盤の設�?
NUM_OF_DEVICES num_of_devices;

//mcmdの設�?
MCMD_HandleTypedef mcmd4M1_struct;
MCMD_Feedback_Typedef mcmdM1_fb;//MCMDM1からのフィードバ�?クを受け取る構�??体を定義
MCMD_HandleTypedef mcmd4M2_struct;
MCMD_Feedback_Typedef mcmdM2_fb;//MCMDM2からのフィードバ�?クを受け取る構�??体を定義

//サーボ�?�設�?
CANServo_Param_Typedef servo_param;
CAN_Device servo_device;

//エアシリの設�?
CAN_Device air_device;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_CAN1_Init(void);
static void MX_ETH_Init(void);
void StartDefaultTask(void const * argument);
void StartSystemCheckTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(uint8_t ch)
#else
#define PUTCHAR_PROTYPE int fputc(int ch,FILE *f)
#endif



PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart3, &ch, 1, 500);
    return ch;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
    WhenTxMailbox0_1_2CompleteCallbackCalled();
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan){
    WhenTxMailbox0_1_2AbortCallbackCalled();
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
    WhenTxMailbox0_1_2CompleteCallbackCalled();
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan){
    WhenTxMailbox0_1_2AbortCallbackCalled();
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
    WhenTxMailbox0_1_2CompleteCallbackCalled();
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan){
    WhenTxMailbox0_1_2AbortCallbackCalled();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    WhenCANRxFifo0MsgPending(hcan, &num_of_devices);
}

//can通信の設�?
void canSetting(){
	printf("Start Initializing CAN System:Begin\n\r");
	HAL_Delay(10);

	CAN_SystemInit(&hcan1); // F7のCAN通信のinit

	// �?バイス数の設�?
	num_of_devices.mcmd3 = 1;
	num_of_devices.mcmd4 = 0;
	num_of_devices.air = 0;
	num_of_devices.servo = 0;

	printf("Start Initializing CAN System:End\n\r");
	HAL_Delay(10);
	CAN_WaitConnect(&num_of_devices);  // 設定された全てのCANモジュール基板との接続が確認できるまで�?�?
}

//モータ1のmcmd設�?
void mcmdMoter1Setting(){
	    // 接続�?��?�MCMDの設�?
	    mcmd4M1_struct.device.node_type = NODE_MCMD3;  // nodeのタイ�?
	    mcmd4M1_struct.device.node_id = 5;  // 基板の番号 (基板上�?�半固定抵抗を回す事で設定できる)
	    mcmd4M1_struct.device.device_num = 0;  // モーターの番号(0→M1,1→M2)

	    // 制御パラメータの設�?
	    mcmd4M1_struct.ctrl_param.ctrl_type = MCMD_CTRL_DUTY;  //制御タイプを設�?
	    mcmd4M1_struct.ctrl_param.PID_param.kp = 0.3f;  // Pゲイン 1.0
	    mcmd4M1_struct.ctrl_param.PID_param.ki = 0.0f;  // Iゲイン 0.0
	    mcmd4M1_struct.ctrl_param.PID_param.kd = 0.0f;  // Dゲイン 0.0 (Dゲインは使�?にくい)
	    mcmd4M1_struct.ctrl_param.accel_limit = ACCEL_LIMIT_ENABLE;  // PIDの偏差をclipする�?
	    mcmd4M1_struct.ctrl_param.accel_limit_size = 2.0f;  // PIDの偏差をclipする場合�?�絶対値のmax値
	    mcmd4M1_struct.ctrl_param.feedback = MCMD_FB_ENABLE;  // MCMDからF7にフィードバ�?クを�?�信するか否�?
	    mcmd4M1_struct.ctrl_param.timup_monitor = TIMUP_MONITOR_DISABLE;  // timeupは未実�?なのでDISABLE�?
	    mcmd4M1_struct.enc_dir = MCMD_DIR_FW;  // Encoderの回転方向設�?
	    mcmd4M1_struct.rot_dir = MCMD_DIR_BC;  // モーターの回転方向設�?
	    mcmd4M1_struct.quant_per_unit = 59.0/6400.0f;  // エンコー�?ーの�?解能に対する制御値の変化量�?�割�?

	    // 原点サーチ�?�設�?
	    mcmd4M1_struct.limit_sw_type = LIMIT_SW_NO;  // 原点サーチにNomaly Closedのスイ�?チを用�?�?
	    mcmd4M1_struct.calib = CALIBRATION_ENABLE;  // 原点サーチを行うかど�?か�??
	    mcmd4M1_struct.calib_duty = -0.2f;  // 原点サーチ時のduty
	    mcmd4M1_struct.offset = 0.0f;  // 原点のオフセ�?�?
	    mcmd4M1_struct.fb_type = MCMD_FB_POS;  // 読み取った位置�?報をF7にフィードバ�?クする�?

	    // パラメータなどの設定と動作命令をMCMDに送信する
		 MCMD_init(&mcmd4M1_struct);
		 HAL_Delay(10);
		 MCMD_Calib(&mcmd4M1_struct);  // キャリブレーションを行う
		 HAL_Delay(5000);  // キャリブレーションが終わるまで�?つ
}

//モータ2のmcmd設�?
void mcmdMoter2Setting(){
	    // 接続�?��?�MCMDの設�?
	    mcmd4M2_struct.device.node_type = NODE_MCMD3;  // nodeのタイ�?
	    mcmd4M2_struct.device.node_id = 5;  // 基板の番号 (基板上�?�半固定抵抗を回す事で設定できる)
	    mcmd4M2_struct.device.device_num = 1;  // モーターの番号(0→M1,1→M2)

	    // 制御パラメータの設�?
	    mcmd4M2_struct.ctrl_param.ctrl_type = MCMD_CTRL_DUTY;  //制御タイプを設�?
	    mcmd4M2_struct.ctrl_param.PID_param.kp = 0.3f;  // Pゲイン 1.0
	    mcmd4M2_struct.ctrl_param.PID_param.ki = 0.0f;  // Iゲイン 0.0
	    mcmd4M2_struct.ctrl_param.PID_param.kd = 0.0f;  // Dゲイン 0.0 (Dゲインは使�?にくい)
	    mcmd4M2_struct.ctrl_param.accel_limit = ACCEL_LIMIT_ENABLE;  // PIDの偏差をclipする�?
	    mcmd4M2_struct.ctrl_param.accel_limit_size = 2.0f;  // PIDの偏差をclipする場合�?�絶対値のmax値
	    mcmd4M2_struct.ctrl_param.feedback = MCMD_FB_ENABLE;  // MCMDからF7にフィードバ�?クを�?�信するか否�?
	    mcmd4M2_struct.ctrl_param.timup_monitor = TIMUP_MONITOR_DISABLE;  // timeupは未実�?なのでDISABLE�?
	    mcmd4M2_struct.enc_dir = MCMD_DIR_FW;  // Encoderの回転方向設�?
	    mcmd4M2_struct.rot_dir = MCMD_DIR_BC;  // モーターの回転方向設�?
	    mcmd4M2_struct.quant_per_unit = 59.0/6400.0f;  // エンコー�?ーの�?解能に対する制御値の変化量�?�割�?

	    // 原点サーチ�?�設�?
	    mcmd4M2_struct.limit_sw_type = LIMIT_SW_NO;  // 原点サーチにNomaly Closedのスイ�?チを用�?�?
	    mcmd4M2_struct.calib = CALIBRATION_ENABLE;  // 原点サーチを行うかど�?か�??
	    mcmd4M2_struct.calib_duty = -0.2f;  // 原点サーチ時のduty
	    mcmd4M2_struct.offset = 0.0f;  // 原点のオフセ�?�?
	    mcmd4M2_struct.fb_type = MCMD_FB_POS;  // 読み取った位置�?報をF7にフィードバ�?クする�?

	    // パラメータなどの設定と動作命令をMCMDに送信する
		 MCMD_init(&mcmd4M2_struct);
		 HAL_Delay(10);
		 MCMD_Calib(&mcmd4M2_struct);  // キャリブレーションを行う
		 HAL_Delay(5000);  // キャリブレーションが終わるまで�?つ
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_CAN1_Init();
  MX_ETH_Init();
  /* USER CODE BEGIN 2 */

  //記事ではmcmdなどの初期化コードを描くことになって�?る�?��?
  canSetting();
//  servoSetting();
  mcmdMoter1Setting();
  mcmdMoter2Setting();
//  airSetting();
  printf("calibrationFinished\r\n");

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
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of systemCheckTask */
  osThreadStaticDef(systemCheckTask, StartSystemCheckTask, osPriorityBelowNormal, 0, 512, systemCheckTaskBuffer, &systemCheckTaskControlBlock);
  systemCheckTaskHandle = osThreadCreate(osThread(systemCheckTask), NULL);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);  // LED1 消�?�
  /* Infinite loop */
  for(;;)
  {
	uint16_t button_data = UDPController_GetControllerButtons();  // buttonの入力を取�?
	if((button_data & CONTROLLER_CIRCLE) != 0){  // oボタンが押されて�?る�?��?
	   HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);  // LED1 点灯
	}else{
	   HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);  // LED1 消�?�

	}
	osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSystemCheckTask */
/**
* @brief Function implementing the systemCheckTask thread.
* @param argument: Not used
* @retval None
*
*/
//M1の動作確認用
void mcmdMoter1Checker(){
		 MCMD_SetTarget(&mcmd4M1_struct, 0.5f);  // 目標�?�を設�?
		 osDelay(10);
		 MCMD_Control_Enable(&mcmd4M1_struct);  // 制御開�?
		 printf("MCMDM1controllStart");
		 osDelay(1000);
		 MCMD_SetTarget(&mcmd4M1_struct, 0.00f);  // 目標�?�を設�?
	}

//M2の動作確認用
void mcmdMoter2Checker(){
		 MCMD_SetTarget(&mcmd4M2_struct, 0.5f);  // 目標�?�を設�?
		 osDelay(10);
		 MCMD_Control_Enable(&mcmd4M2_struct);  // 制御開�?
		 printf("MCMDM1controllStart");
		 osDelay(1000);
		 MCMD_SetTarget(&mcmd4M2_struct, 0.00f);  // 目標�?�を設�?
	}

//M1のエンコー�?ー確認用
void mcmdEncorder1Checker(){//無限ループ�?�中で実�?
	mcmdM1_fb = Get_MCMD_Feedback(&(mcmd4M1_struct.device));
	printf("value of M1 %d\r\n",(int)(mcmdM1_fb.value));
}

//M2のエンコー�?ー確認用
void mcmdEncorder2Checker(){//無限ループ�?�中で実�?
	mcmdM2_fb = Get_MCMD_Feedback(&(mcmd4M2_struct.device));
	printf("value of M2 %d\r\n",(int)(mcmdM2_fb.value));
}

//
void chenge_control_chekcer(){

}
/* USER CODE END Header_StartSystemCheckTask */
void StartSystemCheckTask(void const * argument)
{
  /* USER CODE BEGIN StartSystemCheckTask */
	mcmdMoter1Checker();
	mcmdMoter2Checker();
  /* Infinite loop */
  for(;;)
  {
	  mcmdEncorder1Checker();
	  mcmdEncorder2Checker();
	  osDelay(1000);
  }
  /* USER CODE END StartSystemCheckTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
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
  while (1)
  {
	  printf("error\r\n");
	  HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);  // LED1点灯
	  osDelay(100);
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
