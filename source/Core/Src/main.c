/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BOARD1 1  // comment out this line when compiling for board #2
#if defined(BOARD1)
    const uint32_t NODEID     = 0x123;   // node 1
    const uint32_t FILTERID   = 0x111;   // voltage
    const uint32_t FILTERMASK = 0;       // not used
#else
    const uint32_t NODEID     = 0x124;   // node 2
    const uint32_t FILTERID   = 0x112;   // Current
    const uint32_t FILTERMASK = 0x113;   // Temperature
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/*CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void debugPrint(UART_HandleTypeDef *huart, char _out[]);
void debugPrintln(UART_HandleTypeDef *huart, char _out[]);
void PrintlnEightBit(UART_HandleTypeDef *huart,uint8_t TxData[]);
void UnderVoltageError();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	CAN_TxHeaderTypeDef TxHeader;  // CAN Tx message header structure
	CAN_RxHeaderTypeDef RxHeader;  // CAN Rx message header structure

	uint32_t TxMailbox;			   // Mailbox where Tx message is stored

	uint8_t TxData[8];			   // Vector that will contain data to be transmitted
	uint8_t RxData[8];             // Vector that will contain data received
	uint8_t count = 1;             // Simple counter used for debugging


	// User button callback
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		if(GPIO_Pin == B1_Pin)
		    {
			HAL_GPIO_WritePin(GPIOB, LD2_Pin,GPIO_PIN_SET); // set on The Output (LED) Pin

		    //now send can message
		    TxData[0] = count;
		    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		          {
		        	  Error_Handler();
		          }
		    count++;

		    debugPrintln(&huart2, "Some data has been sent:");
		    PrintlnEightBit(&huart2, TxData);
		    debugPrintln(&huart2, "\r\n************************************");
		    HAL_Delay(500); //debouncing
		    HAL_GPIO_WritePin(GPIOB, LD2_Pin,GPIO_PIN_RESET); // set off The Output (LED) Pin

		    }
	}

	// CAN Rx callback
	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
	{
		char temp_string[16]; //temporaney 15 character string to print numerical values
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
		debugPrintln(&huart2, "Some data has been received:");
		PrintlnEightBit(&huart2, RxData);
		#if defined(BOARD1)
		/* BEGIN node 1 code */
		if (RxHeader.StdId==0x111){    //voltages
			float mult_factor_v = 1000;
			float voltage[4];
			for (int i=0 ; i<=3 ; i++){
				voltage[i] = (RxData[i*2+1]<<8|RxData[i*2])/mult_factor_v;
				debugPrint(&huart2, "Battery Voltage [");
				sprintf(temp_string,"%d", i);
				debugPrint(&huart2, temp_string);
				debugPrint(&huart2, "] = ");
				sprintf(temp_string,"%.3f", voltage[i]);
				debugPrint(&huart2, temp_string);
				debugPrintln(&huart2, " V");
				if (voltage[i]<=2.2){
					UnderVoltageError();
				}
			}
		}
		/* END   node 1 code */
		#else
		/* BEGIN node 2 code */
		if (RxHeader.StdId==0x112){    //current
			float mult_factor_I = 10;
			float current = (RxData[1]<<8|RxData[0])/mult_factor_I;
			debugPrint(&huart2, "Current = ");
			sprintf(temp_string,"%.3f", current);
			debugPrint(&huart2, temp_string);
			debugPrintln(&huart2, " A");
		}
		if (RxHeader.StdId==0x113){    //temperature
			float mult_factor_T = 100;
			float temperature = (RxData[1]<<8|RxData[0])/mult_factor_T;
			debugPrint(&huart2, "Temperature = ");
			sprintf(temp_string,"%.3f", temperature);
			debugPrint(&huart2, temp_string);
			debugPrintln(&huart2, " deg");
		}

		/* END   node 2 code */
		#endif
		// print the id
		debugPrint(&huart2, "From NODEID:0x");
		sprintf(&temp_string[0],"%lx", RxHeader.StdId);
		debugPrintln(&huart2, temp_string);
		debugPrintln(&huart2, "************************************");

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
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_CAN_Start(&hcan) != HAL_OK)        // start can pheriperal
        {
      	  Error_Handler();
        }
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  	TxHeader.DLC = 8;                        // Can data lenght (8 byte)
    TxHeader.IDE = CAN_ID_STD;               // Id lenght       (standard 11 bit)
    TxHeader.RTR = CAN_RTR_DATA;             // Type of frame   (Data frame)
    TxHeader.StdId = NODEID;				 // Can id          (0x123 or 0x124)
    TxHeader.ExtId = 0;                      // Extended id     (not used)
    TxHeader.TransmitGlobalTime = DISABLE;   //                 (disabled)

    // some random data to be send
    TxData[0] = 0x01;
    TxData[1] = 0x02;
    TxData[2] = 0x03;
    TxData[3] = 0x04;

    // first interaction between node and pc
    char NODEIDstr[4];
    debugPrintln(&huart2, "************************************");
    debugPrintln(&huart2, "*This node is turning on correctly!*");
    debugPrint(&huart2, "* node ID: ");
    sprintf(&NODEIDstr[0],"%lx", NODEID);
    debugPrint(&huart2, "0x");
    debugPrintln(&huart2, NODEIDstr);
    debugPrintln(&huart2, "************************************");
  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;  // Enale can filtering
    canfilterconfig.FilterBank = 0;                        // Anything between 0 to SlaveStartFilterBank
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;   // Filter bank to be initialized
    canfilterconfig.FilterIdHigh = FILTERID <<5;           // Standard ID accepted (#1)
    canfilterconfig.FilterIdLow = 0x0;                     // Extended part of ID (not used)
    canfilterconfig.FilterMaskIdHigh = FILTERMASK <<5;     // Standard ID accepted (#2)
    canfilterconfig.FilterMaskIdLow = 0x0;	               // Extended part of ID (not used)
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;    // Specify filter mode
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;   // Filter dimension
    canfilterconfig.SlaveStartFilterBank = 0  ; 		   // Not important

    HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);	       // Initialize the filter

  /* USER CODE END CAN_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//normal print
void debugPrint(UART_HandleTypeDef *huart, char _out[]){
 HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 100);
}

//print with newline
void debugPrintln(UART_HandleTypeDef *huart, char _out[]){
 HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 100);
 char newline[2] = "\r\n";
 HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
}

//print a vector of 8 uint8_t numbers
void PrintlnEightBit(UART_HandleTypeDef *huart,uint8_t TxData[]){
	char string[(8*4+1)];
	    for (int i = 0; i<8; i++){
		    sprintf(&string[(i*4)],"%04d",TxData[i]);
	        string[(i*4)]=' ';
	    }
	string[8*4+1] = '\0';

    HAL_UART_Transmit(huart, (uint8_t *) string, strlen(string), 100);
    char newline[2] = "\r\n";
    HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
}

void UnderVoltageError(){
	unsigned char ERRORMSG[8] = "ERROR";
	debugPrintln(&huart2, "");
	debugPrintln(&huart2, "****************************************");
	debugPrintln(&huart2, "* An error has occurred!: UNDERVOLTAGE *");
	debugPrintln(&huart2, "* execution has stopped, shooting down *");
	debugPrintln(&huart2, "****************************************");
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, ERRORMSG, &TxMailbox) != HAL_OK)
		  {
			  Error_Handler();
		  }
	while(1){
		HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
		HAL_Delay(100);
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
  HAL_GPIO_WritePin(GPIOB, LD2_Pin,GPIO_PIN_SET);
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

