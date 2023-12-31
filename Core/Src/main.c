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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	CAN_BUS_FRAME_NONE = 0x00,
	CANUSB_FRAME_STANDARD = 0x01,
	CANUSB_FRAME_EXTENDED = 0x02,
}
CANUSB_FRAME;

typedef enum
{
	CANUSB_CONFIG_TYPE_NORMAL = 0x02,
	CANUSB_CONFIG_TYPE_MANUAL = 0x03,
}
TYPE_SETTING_LOW_4b;

typedef enum
{
	NONE_FORMAT = 0x00,
	FIXED_FORMAT = 0x01,
	NORMAL_FORMAT = 0x02,
}
FRAME_FORMAT;

typedef enum
{
	CANUSB_SPEED_1000000 = 0x01,
	CANUSB_SPEED_800000  = 0x02,
	CANUSB_SPEED_500000  = 0x03,
	CANUSB_SPEED_400000  = 0x04,
	CANUSB_SPEED_250000  = 0x05,
	CANUSB_SPEED_200000  = 0x06,
	CANUSB_SPEED_125000  = 0x07,
	CANUSB_SPEED_100000  = 0x08,
	CANUSB_SPEED_50000   = 0x09,
	CANUSB_SPEED_20000   = 0x0a,
	CANUSB_SPEED_10000   = 0x0b,
	CANUSB_SPEED_5000    = 0x0c,
}
CANUSB_SPEED;

CANUSB_FRAME canTypeFrame = CAN_BUS_FRAME_NONE;
FRAME_FORMAT canFrameFormat = NONE_FORMAT;
volatile int countBlinkCanSedn = 0;
volatile int countBlinkCanRecv = 0;

#define BUFFER_SIZE 255
#define NUMBER_BLINK_LED_INDICATE 5

UART_HandleTypeDef huart1;
uint8_t receive_buff[255];                //Define the receive array
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

int configureCanBus(uint8_t dataLen);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(USART1 == huart1.Instance)                                   //Determine whether it is serial port 1
    {
        if(RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))   //Judging whether it is idle interruption
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart1);                     //Clear idle interrupt sign (otherwise it will continue to enter interrupt)
            USAR_UART_IDLECallback(huart);                          //Call interrupt handler
        }
    }
}

void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	//Stop this DMA transmission
	HAL_UART_DMAStop(&huart1);  

	//Calculate the length of the received data
	uint8_t data_length  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);   

	do
	{
		if(configureCanBus(data_length))
			break;

		uartToCanDataProcess(data_length);
		uartToCanDataFormatFix20B(data_length);
		countBlinkCanSedn = NUMBER_BLINK_LED_INDICATE;
	}
	while(0);

	//Zero Receiving Buffer
	memset(receive_buff,0,data_length);                                            
	data_length = 0;

	//Restart to start DMA transmission of 255 bytes of data at a time
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_buff, 255);                  
}
static int generate_checksum(const unsigned char *data, int data_len)
{
  int i, checksum;

  checksum = 0;
  for (i = 0; i < data_len; i++) {
    checksum += data[i];
  }

  return checksum & 0xff;
}

int configureCanBus(uint8_t dataLen)
{
	TYPE_SETTING_LOW_4b typeSetting;

	if(dataLen < 19)
		return 0;

	if(receive_buff[0] != 0xAA || receive_buff[1] != 0x55)
		return 0;

	if((receive_buff[2] & 0xF0) == 0x10)
		canFrameFormat = NORMAL_FORMAT;
	else if((receive_buff[2] & 0xF0) == 0x00)
		canFrameFormat = FIXED_FORMAT;
	else
		return 0;

	typeSetting = receive_buff[2] & 0x0F;

	if(receive_buff[4] != CANUSB_FRAME_STANDARD && receive_buff[4] !=
		CANUSB_FRAME_EXTENDED)
		return 0;

	canTypeFrame = receive_buff[4];

	if(receive_buff[19] != generate_checksum(receive_buff + 2, 17))
		return 0;

	if(typeSetting == CANUSB_CONFIG_TYPE_NORMAL)
	{
		switch(receive_buff[3])
		{
			case CANUSB_SPEED_1000000:
				hcan.Init.Prescaler = 9;
				hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
				hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
				break;
			case CANUSB_SPEED_800000:
				hcan.Init.Prescaler = 9;
				hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
				hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
				break;
			case CANUSB_SPEED_500000:
				hcan.Init.Prescaler = 9;
				hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
				hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
				break;
			case CANUSB_SPEED_400000:
				hcan.Init.Prescaler = 9;
				hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
				hcan.Init.TimeSeg2 = CAN_BS2_6TQ;
				break;
			case CANUSB_SPEED_250000:
				hcan.Init.Prescaler = 9;
				hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
				hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
				break;
			case CANUSB_SPEED_200000:
				hcan.Init.Prescaler = 9;
				hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
				hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
				break;
			case CANUSB_SPEED_125000:
				hcan.Init.Prescaler = 16;
				hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
				hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
				break;
			case CANUSB_SPEED_100000:
				hcan.Init.Prescaler = 15;
				hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
				hcan.Init.TimeSeg2 = CAN_BS2_7TQ;
				break;
			case CANUSB_SPEED_50000:
				hcan.Init.Prescaler = 30;
				hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
				hcan.Init.TimeSeg2 = CAN_BS2_7TQ;
				break;
			case CANUSB_SPEED_20000:
				hcan.Init.Prescaler = 72;
				hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
				hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
				break;
			case CANUSB_SPEED_10000:
				hcan.Init.Prescaler = 144;
				hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
				hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
				break;
			case CANUSB_SPEED_5000:
				hcan.Init.Prescaler = 288;
				hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
				hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
				break;
		}
	}
	else if(typeSetting == CANUSB_CONFIG_TYPE_MANUAL)
	{
		uint8_t seg1 = receive_buff[14];
		uint8_t seg2 = receive_buff[15];
		hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
		hcan.Init.TimeSeg2 = CAN_BS2_1TQ;

		if(seg1 & 0x01)
			hcan.Init.TimeSeg1 |= CAN_BTR_TS1_0;

		if(seg1 & 0x02)
			hcan.Init.TimeSeg1 |= CAN_BTR_TS1_1;

		if(seg1 & 0x04)
			hcan.Init.TimeSeg1 |= CAN_BTR_TS1_2;

		if(seg1 & 0x08)
			hcan.Init.TimeSeg1 |= CAN_BTR_TS1_3;

		if(seg2 & 0x01)
			hcan.Init.TimeSeg2 |= CAN_BTR_TS2_0;

		if(seg2 & 0x02)
			hcan.Init.TimeSeg2 |= CAN_BTR_TS2_1;

		if(seg2 & 0x04)
			hcan.Init.TimeSeg2 |= CAN_BTR_TS2_2;

		hcan.Init.Prescaler = *((uint16_t*)(receive_buff + 16));
	}
	else
		return 0;

	if (HAL_CAN_Init(&hcan) != HAL_OK)
		Error_Handler();

	HAL_CAN_Start(&hcan);

	return 1;
}

void uartToCanDataFormatFix20B(uint8_t dataLen)
{
	uint8_t csend[8]; // Tx Buffer
	uint32_t canMailbox; //CAN Bus Mail box variable
	CAN_TxHeaderTypeDef txHeader; //CAN Bus Receive Header

	if(dataLen < 20)
		return;

	if(receive_buff[0] != 0xAA || receive_buff[1] != 0x55 ||
		receive_buff[2] != 0x01)
		return;

	if(receive_buff[3] != 0x01 && receive_buff[3] != 0x02)
		return;

	if(receive_buff[4] != 0x01 && receive_buff[4] != 0x02)
		return;

	uint8_t canDataLen = receive_buff[9];

	if(canDataLen > 8)
		return;

	if(receive_buff[19] != generate_checksum(receive_buff + 2, 17))
		return;

	txHeader.DLC = canDataLen;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.TransmitGlobalTime = DISABLE;

	if(receive_buff[3] == CANUSB_FRAME_EXTENDED)
	{
		txHeader.IDE = CAN_ID_EXT;
		txHeader.ExtId = *((uint32_t*)(receive_buff + 5));
	}
	else
	{
		txHeader.IDE = CAN_ID_STD;
		txHeader.StdId = *((uint16_t*)(receive_buff + 5));
	}

	memcpy(csend, receive_buff + 10, canDataLen);
	HAL_CAN_AddTxMessage(&hcan, &txHeader ,csend ,&canMailbox);
}

void uartToCanDataProcess(uint8_t dataLen)
{
	uint8_t csend[8]; // Tx Buffer
	uint32_t canMailbox; //CAN Bus Mail box variable
	CAN_TxHeaderTypeDef txHeader; //CAN Bus Receive Header

	if(receive_buff[0] != 0xAA)
		return;
	
	if((receive_buff[1] & 0xC0) != 0xC0)
		return;

	uint8_t canDataLen = receive_buff[1] & 0x0F;

	if(canDataLen > 8)
		return;

	txHeader.DLC = canDataLen;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.TransmitGlobalTime = DISABLE;

	if(receive_buff[1] & (1 << 5))
	{
		if(dataLen < canDataLen + 7)
			return;

		if(receive_buff[canDataLen + 6] != 0x55)
			return;

		txHeader.IDE = CAN_ID_EXT;
		txHeader.ExtId = *((uint32_t*)(receive_buff + 2));
		memcpy(csend, receive_buff + 6, canDataLen);
	}
	else
	{
		if(dataLen < canDataLen + 5)
			return;

		if(receive_buff[canDataLen + 4] != 0x55)
			return;

		txHeader.IDE = CAN_ID_STD;
		txHeader.StdId = *((uint16_t*)(receive_buff + 2));
		memcpy(csend, receive_buff + 4, canDataLen);
	}

	HAL_CAN_AddTxMessage(&hcan, &txHeader ,csend ,&canMailbox);
}

void canToUartNormalFormat(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader; //CAN Bus Transmit Header
	uint8_t canRX[8];  //CAN Bus Receive Buffer
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX); //Receive CAN bus message to canRX buffer
	uint8_t transmitBuffer[20];
	int posBuffer = 0;
	transmitBuffer[posBuffer++] = 0xAA;
	transmitBuffer[posBuffer++] = 0xC0 + rxHeader.DLC +
			(CAN_ID_EXT == rxHeader.IDE ? 1 << 5 : 0);

	if(rxHeader.IDE == CAN_ID_STD)
	{
		*((uint16_t*)(transmitBuffer + posBuffer)) = rxHeader.StdId;
		posBuffer += 2;
	}
	else
	{
		*((uint32_t*)(transmitBuffer + posBuffer)) = rxHeader.ExtId;
		posBuffer += 4;
	}

	memcpy(transmitBuffer + posBuffer, canRX, rxHeader.DLC);
	posBuffer += rxHeader.DLC;
	transmitBuffer[posBuffer++] = 0x55;

	HAL_UART_Transmit_IT(&huart1, transmitBuffer, posBuffer);
}

void canToUartFixed20Format(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader; //CAN Bus Transmit Header
	uint8_t canRX[8];  //CAN Bus Receive Buffer
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX); //Receive CAN bus message to canRX buffer

	uint8_t transmitBuffer[20];
	int posBuffer = 0;

	transmitBuffer[posBuffer++] = 0xAA;
	transmitBuffer[posBuffer++] = 0x55;
	transmitBuffer[posBuffer++] = 0x01;
	transmitBuffer[posBuffer++] = CAN_ID_EXT == rxHeader.IDE ? 0x02 : 0x01;
	transmitBuffer[posBuffer++] = 0x01;

	memset(transmitBuffer+ posBuffer, 0 , 4);

	if(rxHeader.IDE == CAN_ID_STD)
		*((uint16_t*)(transmitBuffer + posBuffer)) = rxHeader.StdId;
	else
		*((uint32_t*)(transmitBuffer + posBuffer)) = rxHeader.ExtId;

	posBuffer += 4;
	transmitBuffer[posBuffer++] = rxHeader.DLC;


	memset(transmitBuffer+ posBuffer, 0 , 8);
	memcpy(transmitBuffer + posBuffer, canRX, rxHeader.DLC);
	posBuffer +=8;
	transmitBuffer[posBuffer++] = 0;
	transmitBuffer[posBuffer++] = generate_checksum(transmitBuffer + 2, 17);

	HAL_UART_Transmit_IT(&huart1, transmitBuffer, posBuffer);
}

__weak void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
            user file
   */
	if(canFrameFormat == NORMAL_FORMAT)
		canToUartNormalFormat(hcan);
	else if(canFrameFormat == FIXED_FORMAT)
		canToUartFixed20Format(hcan);
	else
		return;

	countBlinkCanRecv = NUMBER_BLINK_LED_INDICATE;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(countBlinkCanSedn && --countBlinkCanSedn)
		HAL_GPIO_TogglePin(CAN_SEND_GPIO_Port,CAN_SEND_Pin);
	else
		HAL_GPIO_WritePin(CAN_SEND_GPIO_Port,CAN_SEND_Pin, GPIO_PIN_SET);

	if(countBlinkCanRecv && --countBlinkCanRecv)
		HAL_GPIO_TogglePin(CAN_RECV_GPIO_Port,CAN_RECV_Pin);
	else
		HAL_GPIO_WritePin(CAN_RECV_GPIO_Port,CAN_RECV_Pin, GPIO_PIN_SET);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	CAN_FilterTypeDef canfil; //CAN Bus Filter
	uint32_t canMailbox; //CAN Bus Mail box variable

	canfil.FilterBank = 0;
	canfil.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfil.FilterIdHigh = 0;
	canfil.FilterIdLow = 0;
	canfil.FilterMaskIdHigh = 0;
	canfil.FilterMaskIdLow = 0;
	canfil.FilterScale = CAN_FILTERSCALE_32BIT;
	canfil.FilterActivation = ENABLE;
	canfil.SlaveStartFilterBank = 14;
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
//     HAL_UART_Transmit_IT(&huart1, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));
//     HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 10);
//     HAL_UART_Receive_IT(&huart1, (uint8_t *)receive_buff, 255);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_CAN_ConfigFilter(&hcan,&canfil); //Initialize CAN Filter
  HAL_CAN_Start(&hcan); //Initialize CAN Bus
  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);// Initialize CAN Bus Rx Interrupt
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_buff, 255);     //Set up the DMA
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(CAN_SEND_GPIO_Port,CAN_SEND_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CAN_RECV_GPIO_Port,CAN_RECV_Pin, GPIO_PIN_SET);

  while (1)
  {
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  htim2.Init.Prescaler = 250;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CAN_RECV_Pin|CAN_SEND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_RECV_Pin CAN_SEND_Pin */
  GPIO_InitStruct.Pin = CAN_RECV_Pin|CAN_SEND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
