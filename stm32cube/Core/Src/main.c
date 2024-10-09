/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "lwip/sockets.h"          <----
//#include <string.h>
#include "tcpServerRAW.h"
//#include "tcpClientRAW.h"
#include "interface.h"
//#include <stdio.h>
//#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define PORT	5015          <----
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

char uart_bufT[1000];
char get_info[1000];
int get_state = 0;
//char uart_bufR[100];
int uart_buf_len;

char spi_buf[100];
char spi_addr[20];

const uint16_t READ_Intruction = 0x8000;

const uint16_t Config_Addr = 0x0000;
const uint8_t SDO_Active = 0x18 | 0x81; // 0x81 or 0x01

const uint16_t Read_Buffer_Addr = 0x0004;
const uint8_t Read_Buffer = 0x00 | 0x01;	// read buffered registers instead of currently used
const uint8_t Read_Currently = 0x00;

const uint16_t UPD_Addr = 0x0005;
const uint8_t UPD_Auto = 0x00 | 0x01;	// Automatically Update registers

const uint16_t Power_Addr = 0x0010;
const uint8_t PLL_Bypassed = 0xD0;	// Power of PLL_Multiplier will be down
const uint8_t PLL_Enabled = 0xC0;	// Power of PLL_Multiplier will be up

const uint16_t N_divider_Addr = 0x0020;
uint8_t N_Divider = 0x03;	// range 0x00 to 0x1F (0 to 31) + 2 = 2 to 33. def. is 0x12 (18) + 2 = 20 (x 2) = 40 (sysclk = 25 MHz) = 1 GHz

const uint16_t FTW_Addr6 = 0x01A6;
const uint16_t FTW_Addr7 = 0x01A7;
const uint16_t FTW_Addr8 = 0x01A8;
const uint16_t FTW_Addr9 = 0x01A9;
const uint16_t FTW_AddrA = 0x01AA;
const uint16_t FTW_AddrB = 0x01AB;
double f_s = 1000.0;	// MHz of sysclk
double f_DDS = 0.0;	// MHz of initial frequency
double f_ref = 100.0;	// MHz of reference frequency

const uint16_t DAC_Current_AddrB = 0x040B;
const uint16_t DAC_Current_AddrC = 0x040C;
const double I_DAC_REF = 0.120;	// mA if R_DAC_REF = 10 k[ohm]
const double I_DAC_FS = 10.0;	// mA in output

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_SPI4_Init(void);

/* USER CODE BEGIN PFP */

void send_freq(double fdds);
void send_current(double idac);
void set_ref(double ref);
double get_freq(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern struct netif gnetif;
extern parameters par;
int32_t raw;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_USART3_UART_Init();
  MX_LWIP_Init();
  MX_TIM7_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */

  tcp_server_init();
  //tcp_client_init();
  tcp_server_init();
  initInterface();


  HAL_TIM_Base_Start_IT(&htim7);

  send_freq(123.123);
  send_current(31.7);
  set_ref(100);
  par.rf.val = get_freq();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ethernetif_input(&gnetif);
	  sys_check_timeouts();

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 80;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 300;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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
  HAL_GPIO_WritePin(GPIOE, RESET_Pin|CS_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IO_UPD_GPIO_Port, IO_UPD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RESET_Pin CS_Pin */
  GPIO_InitStruct.Pin = RESET_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : IO_UPD_Pin */
  GPIO_InitStruct.Pin = IO_UPD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IO_UPD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void send_freq(double fdds)
{
	uint64_t ftw;
	uint8_t value;

	if(fdds > 400.0){
		fdds = 400.0;
	}

	ftw = round((fdds/f_s) * pow(2, 48));

	spi_addr[0] = ((uint8_t*)&FTW_Addr6)[1];
	spi_addr[1] = ((uint8_t*)&FTW_Addr6)[0];
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&ftw)[0];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);

	spi_addr[0] = ((uint8_t*)&FTW_Addr7)[1];
	spi_addr[1] = ((uint8_t*)&FTW_Addr7)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&ftw)[1];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);

	spi_addr[0] = ((uint8_t*)&FTW_Addr8)[1];
	spi_addr[1] = ((uint8_t*)&FTW_Addr8)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&ftw)[2];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);

	spi_addr[0] = ((uint8_t*)&FTW_Addr9)[1];
	spi_addr[1] = ((uint8_t*)&FTW_Addr9)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&ftw)[3];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);

	spi_addr[0] = ((uint8_t*)&FTW_AddrA)[1];
	spi_addr[1] = ((uint8_t*)&FTW_AddrA)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&ftw)[4];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);

	spi_addr[0] = ((uint8_t*)&FTW_AddrB)[1];
	spi_addr[1] = ((uint8_t*)&FTW_AddrB)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&ftw)[5];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	// Update registers
	HAL_GPIO_WritePin(IO_UPD_GPIO_Port, IO_UPD_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(IO_UPD_GPIO_Port, IO_UPD_Pin, GPIO_PIN_RESET);
}

void send_current(double idac)
{
	uint16_t fsc;
	uint8_t value;

	if(idac < 8.6){

		idac = 8.6;
	}else if(idac > 31.7){

		idac = 31.7;
	}

	fsc = round(1024/192 * (idac/I_DAC_REF - 72));

	spi_addr[0] = ((uint8_t*)&DAC_Current_AddrB)[1];
	spi_addr[1] = ((uint8_t*)&DAC_Current_AddrB)[0];
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&fsc)[0];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);

	spi_addr[0] = ((uint8_t*)&DAC_Current_AddrC)[1];
	spi_addr[1] = ((uint8_t*)&DAC_Current_AddrC)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t *)&spi_addr, 2, 100);
	value = ((uint8_t *)&fsc)[1];
	HAL_SPI_Transmit(&hspi4, ((uint8_t *)&value), 1, 100);
}

void set_ref(double ref)
{
	f_ref = ref;

	spi_addr[0] = ((uint8_t*)&Power_Addr)[1];
	spi_addr[1] = ((uint8_t*)&Power_Addr)[0];

	if(ref == 100.0){

		f_s = 1000.0;

		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
		HAL_SPI_Transmit(&hspi4, (uint8_t*)&PLL_Enabled, 1, 100);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

//		// set N-divider for PLL
//		spi_addr[0] = ((uint8_t*)&N_divider_Addr)[1];
//		spi_addr[1] = ((uint8_t*)&N_divider_Addr)[0];
//		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
//		HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
//		HAL_SPI_Transmit(&hspi4, (uint8_t*)&N_Divider, 1, 100);
//		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	}else if(ref == 250.0 || ref == 1000.0){

		f_s = ref;

		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
		HAL_SPI_Transmit(&hspi4, (uint8_t*)&PLL_Bypassed, 1, 100);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

		send_freq(f_DDS);
	}
}


double get_freq(){

	uint16_t address;
	uint64_t ftw = 0x0;
	double fDDS = 0;

	// read freq. value
	address = READ_Intruction | FTW_Addr6;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&ftw)[0] = (unsigned int)spi_buf[0];
	address = READ_Intruction | FTW_Addr7;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&ftw)[1] = (unsigned int)spi_buf[0];
	address = READ_Intruction | FTW_Addr8;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&ftw)[2] = (unsigned int)spi_buf[0];
	address = READ_Intruction | FTW_Addr9;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&ftw)[3] = (unsigned int)spi_buf[0];
	address = READ_Intruction | FTW_AddrA;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&ftw)[4] = (unsigned int)spi_buf[0];
	address = READ_Intruction | FTW_AddrB;
	spi_addr[0] = ((uint8_t*)&address)[1];
	spi_addr[1] = ((uint8_t*)&address)[0];
	HAL_SPI_Transmit(&hspi4, (uint8_t*)&spi_addr, 2, 100);
	HAL_SPI_Receive(&hspi4, (uint8_t*)spi_buf, 1, 100);
	((uint8_t *)&ftw)[5] = (unsigned int)spi_buf[0];
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	fDDS = (ftw/pow(2, 48)) * f_s;

	get_state = 1;
	return fDDS;
}


/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */


  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM7) {
	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, SET);

	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, RESET);
    }
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

