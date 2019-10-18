/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

//#define DEBUG_RPI_COM
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
RpiComStates_t state;
int packetCount = 0;
int errorCount = 0;
int replyCount = 0;

uint8_t readBuffer[BUFFER_SIZE];
uint8_t writeBuffer[BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	state = IDLE;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		readBuffer[i] = 0;
		writeBuffer[i] = 0;
	}

	printf("Init finished\r\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	int ctr = 0;

	while (1) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		loop();
		//HAL_Delay(1);
	    ctr++;
	    if (ctr == 50000)
	    {
	        printf("Message Stats P %d\tE %d\tR %d\r\n", packetCount, errorCount, replyCount);
	        ctr = 0;
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DBG_PIN_GPIO_Port, DBG_PIN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SPI_CS_Pin */
	GPIO_InitStruct.Pin = SPI_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DBG_PIN_Pin */
	GPIO_InitStruct.Pin = DBG_PIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DBG_PIN_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, 0xFFFF);
	return ch;
}

uint8_t readByte(uint8_t index) {
	return readBuffer[index];
}

uint32_t readUInt(uint8_t index) {
	uint32_t result = 0;
	result = readBuffer[index];
	result += readBuffer[index + 1] << 8;
	result += readBuffer[index + 2] << 16;
	result += readBuffer[index + 3] << 24;

	return result;
}

void prepareWriteBuffer(UkpMessages messageType, uint8_t payloadLength) {
	clearWriteBuffer();
	writeBuffer[0] = 0x22;
	writeBuffer[1] = 0x33;
	writeBuffer[2] = messageType;
	writeBuffer[3] = payloadLength;

	for (int i = UKP_HEADER_LENGTH; i < BUFFER_SIZE; i++) {
		writeBuffer[i] = 0;
	}
}

void clearCurrentReadBuffer() {
	for (int i = 0; i < BUFFER_SIZE; i++) {
		readBuffer[i] = 0;
	}
}

HAL_StatusTypeDef restartDma(uint16_t size, bool transmit) {
	//HAL_SPI_DMAStop(&hspi1);
	HAL_SPI_Abort(&hspi1);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);

	if(transmit){
		return HAL_SPI_Transmit_DMA(&hspi1, writeBuffer, size);

	}
	else{
		return HAL_SPI_Receive_DMA(&hspi1, readBuffer, BUFFER_SIZE);
	}
}

void sendReply(uint8_t payloadLength) {
	uint32_t crc = hcp_crc24q(writeBuffer, UKP_HEADER_LENGTH + payloadLength);

#ifdef DEBUG_RPI_COM
	printf("Send Reply: length(%d) crc(0x%x)\r\n", payloadLength,crc);
#endif

	uint8_t *p = (uint8_t *) &crc;
	for (int i = 0; i < 4; i++) {
		writeBuffer[UKP_HEADER_LENGTH + payloadLength + i] = p[3 - i];
	}

	HAL_StatusTypeDef status = restartDma(
			UKP_HEADER_LENGTH + payloadLength + UKP_FOOTER_LENGTH, true);

	if (status == 0) {
		state = READY_TO_TRANSMIT;
		replyCount++;
	} else {
		state = ERROR_R;
#ifdef DEBUG_RPI_COM
		printf("ERR %d\r\n", status);
#endif
	}
}

void sendAckReply() {
#ifdef DEBUG_RPI_COM
	printf("SEND ACK\r\n");
#endif
	prepareWriteBuffer(ACK_NAK, 1);
	writeBuffer[4] = 1;
	sendReply(1);
}

void sendNakReply() {
#ifdef DEBUG_RPI_COM
	printf("SEND NAK\r\n");
#endif
	prepareWriteBuffer(ACK_NAK, 1);
	writeBuffer[4] = 0;
	sendReply(1);
}

void loop() {
	if (state == IDLE) {
		HAL_StatusTypeDef status = restartDma(BUFFER_SIZE, false);
		if (status == 0) {
			state = RECIEVE_DMA_WAITING;
		}
	} else if (state == RECIEVE_DMA_COMPLETE) {
        HAL_SPI_Abort(&hspi1);

		uint8_t index = 0;
		uint8_t length = 0;

		if (readBuffer[index] != 0x22 || readBuffer[index + 1] != 0x33) {
			errorCount++;
           /* for (int i = 0; i < BUFFER_SIZE; i++)
            {
                printf("%x ", readBuffer[i]);
            }
            printf("\r\n\r\n");*/
			clearCurrentReadBuffer();
			state = IDLE;
			return;
		}

		index = index + 2;
		length = 0;

		UkpMessages currentPacketType = (UkpMessages) readByte(index);
		index++;

		length = readByte(index);
		index++;

		uint8_t crcOffset = UKP_HEADER_LENGTH + length;
		uint32_t calculatedCrc = hcp_crc24q(readBuffer, crcOffset);
		uint32_t recievedCrc = readUInt(crcOffset);

		if (calculatedCrc != recievedCrc) {
			printf("CRC ERROR: ");
			printf("Recieved 0x%x but calculated 0x%x\r\n", recievedCrc,
					calculatedCrc);

			sendNakReply();
			errorCount++;
			clearCurrentReadBuffer();
			state = IDLE;
			return;
		}

		switch (currentPacketType) {
		case MY_EXAMPLE_UPDATE_MSG: {
			uint8_t myExampleAttribute1 = readByte(index);
			index++;
			uint8_t myExampleAttribute2 = readByte(index);
#ifdef DEBUG_RPI_COM
			//printf("myExampleAttribute1 %d\r\n", myExampleAttribute1);
			//printf("myExampleAttribute2 %d\r\n", myExampleAttribute2);
#endif
			sendAckReply();
			break;
		default:
			state = IDLE;
			break;
		}

		}

		packetCount++;
		clearCurrentReadBuffer();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == SPI_CS_Pin) {
		GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOA, SPI_CS_Pin);
		if (pinState == GPIO_PIN_SET) {
		    SET_BIT(SPI1->CR1, SPI_CR1_SSI);
			if (state == RECIEVE_DMA_RUNNING) {
				state = RECIEVE_DMA_COMPLETE;
			} else if (state == TRANSMIT_RUNNING) {
				state = IDLE;
			}

		} else if (pinState == GPIO_PIN_RESET) {

			if (state == READY_TO_TRANSMIT) {
				state = TRANSMIT_RUNNING;
			} else if (state == RECIEVE_DMA_WAITING) {
				state = RECIEVE_DMA_RUNNING;
			}
		    uint16_t sr = READ_REG(SPI1->SR);
		    CLEAR_BIT(SPI1->CR1, SPI_CR1_SSI);
		}
	}
}
uint32_t hcp_crc24q(unsigned char *buff, int len) {
	uint32_t crc = 0;
	for (int i = 0; i < len; i++) {
		crc = ((crc << 8) & 0xFFFFFF) ^ tbl_CRC24Q[(crc >> 16) ^ buff[i]];
	}
	return crc;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
