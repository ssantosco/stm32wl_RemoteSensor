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
#include "adc.h"
#include "dma.h"
#include "subghz.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "radio_driver.h"
#include "stm32wlxx_nucleo.h"
#include "stm32wlxx_nucleo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Structures for FSM radio system */
typedef enum
{
	STATE_NULL,
	STATE_MASTER,
} state_t;

typedef enum
{
	SSTATE_NULL,
	SSTATE_RX,
	SSTATE_TX
} substate_t;

typedef struct
{
	state_t state;
	substate_t subState;
	uint32_t rxTimeout;
	uint32_t rxMargin;
	uint32_t randomDelay;
	char rxBuffer[RX_BUFFER_SIZE];
	uint8_t rxSize;
} clientFSM_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RF_FREQUENCY                                868000000 /* Hz */
#define TX_OUTPUT_POWER                             14        /* dBm */
#define LORA_BANDWIDTH                              0         /* Hz */
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */

#define ADC_DMA_BUFFER_SIZE 						1000	 	// Number of ADC values over DMA
#define TX_BUFFER_SIZE 								40			// Buffer for TX data pay load
#define CHANNEL_NUMBER  							2			// ADC channel total number
#define THRESHOLD_AMP 								2048		// Maximum Threshold in mA
#define THRESHOLD_VOL 								2048	 	// Maximum Threshold in V
#define THRESHOLD_TIME								10000		// Time to trigger alarm off or on
#define ADC_EVAL_TIMER								25	 		// Delay time for evaluate the thresholds for alarm siren in ms
#define TX_SEND_TIMER								10000		// Data RF send every xx in ms
#define ADC_SAMPLE_2_TX_TIMER						1000		// Timer to put a sample into txpayload.txBuffer in ms.
#define UART_LOG_LEVEL								2		 	// Set level of log messages over UART
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Structure for TX pay load management */
typedef struct
{
	uint8_t ready2TX;						// Flag to inform about the status of the data loaded (1 -> it's ready to TX, 0 -> it's loading data)
	uint8_t chCnt;							// Total amount of chars into txBuffer
	char txBuffer[TX_BUFFER_SIZE]; 			// Size of data for 20 samples of 16 bit (12 bit ADC)
} payload_t;

/* LoRA variables */
void (*volatile eventReceptor)(clientFSM_t *const fsm);
PacketParams_t packetParams;
const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };

/* TX send verification, two counters to avoid excessive retransmissions */
static int8_t ackNokCnt=0;
static int8_t rxToutCnt=0;

int alarmState = 0;							// Variable to trigger ALARM state, default state 0
int alarmCntOn = 0;							// Counter to set the alarm to on
int alarmCntOff = 0;						// Counter to set the alarm to off
int tim16cbDiv = 0;							// Variable 1 to divide the Interrupt count for output siren and light
int sampleCnt = 0;							// Counter for txpayload.txBuffer filling

payload_t txPayload;						// TX Pay load structure
uint16_t ADCbuffer[ADC_DMA_BUFFER_SIZE];	// Buffer for ADC-DMA reads
uint16_t currentAVG=0;						// AVG values to load into the txpayload.txBuffer
uint16_t voltageAVG=0;

uint32_t toaT0 = 0;							// Variables to estimate ToA
uint32_t toaT1 = 0;
uint32_t hacccT0 = 0;						// Variables for metering the time to calculate AVG values into the ADC conversion callback
uint32_t hacccT1 = 0;
uint32_t htpecT0 = 0;						// Variables for metering the time to to put a sample into txpayload.txBuffer, used in time elapsed interrupt
uint32_t htpecT1 = 0;
// Handlers
TIM_HandleTypeDef htim1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* FSM Radio function prototypes */
void radioInit(void);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);
void eventTxDone(clientFSM_t *const fsm);
void eventRxDone(clientFSM_t *const fsm);
void eventTxTimeout(clientFSM_t *const fsm);
void eventRxTimeout(clientFSM_t *const fsm);
void eventRxError(clientFSM_t *const fsm);
void enterMasterRx(clientFSM_t *const fsm);
void enterMasterTx(clientFSM_t *const fsm);
void transitionRxDone(clientFSM_t *const fsm);

/* TFG function prototypes */
int setAlarmState(int alarmTrigger);
void thresholdEval(uint16_t current, uint16_t voltage);
void uartLog(char* text, int escChar, int level);

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
	clientFSM_t fsm;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable GPIO Clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// DEBUG_SUBGHZSPI_{NSSOUT, SCKOUT, MSIOOUT, MOSIOUT} pins
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_SUBGHZSPI;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// DEBUG_RF_{HSE32RDY, NRESET} pins
	// GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;  // Pins used in other utilities
	GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_RF;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// DEBUG_RF_{SMPSRDY, LDORDY} pins
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// RF_BUSY pin
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Alternate = GPIO_AF6_RF_BUSY;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// RF_{IRQ0, IRQ1, IRQ2} pins
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_8;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SUBGHZ_Init();
  MX_ADC_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	char uartBuff[100];
	strcpy(uartBuff, "\n\rClient Side\r\nAPP_VERSION=0.0.1\r\n---------------");
	uartLog(uartBuff, 0, 1);
	sprintf(uartBuff, "LORA_MODULATION\r\nLORA_BW=%d Hz\r\nLORA_SF=%d", (1 << LORA_BANDWIDTH) * 125, LORA_SPREADING_FACTOR);
	uartLog(uartBuff, 0, 1);
	sprintf(uartBuff, "SystemClock: %lu Hz", SystemCoreClock);
	uartLog(uartBuff, 0, 1);
	radioInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// get random number
	uint32_t rnd = 0;
	SUBGRF_SetDioIrqParams(IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
	rnd = SUBGRF_GetRandom();

	fsm.state = STATE_MASTER;
	fsm.subState = SSTATE_TX;
	fsm.rxTimeout = 3000; // 3000 ms
	fsm.rxMargin = 200;   // 200 ms
	fsm.randomDelay = rnd >> 22; // [0, 1023] ms
	sprintf(uartBuff, "rand: %lu", fsm.randomDelay);
	uartLog(uartBuff, 0, 1);

	HAL_Delay(fsm.randomDelay);
	SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
			IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
			IRQ_RADIO_NONE,
			IRQ_RADIO_NONE );
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
	SUBGRF_SetRx(fsm.rxTimeout << 6);
	fsm.state = STATE_MASTER;
	fsm.subState = SSTATE_RX;

	// Start reading of ADC values and put them to DMA
	uartLog("\r\nStart reading ADCs to DMA", 0, 1);
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADCbuffer, ADC_DMA_BUFFER_SIZE);
	alarmState=setAlarmState(0);

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		eventReceptor = NULL;
		while (eventReceptor == NULL);
		eventReceptor(&fsm);
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
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Initialize the Sub-GHz radio and dependent hardware.
 * @retval None
 */
void radioInit(void)
{
	// Initialize the hardware (SPI bus, TCXO control, RF switch)
	SUBGRF_Init(RadioOnDioIrq);

	// Use DCDC converter if `DCDC_ENABLE` is defined in radio_conf.h
	// "By default, the SMPS clock detection is disabled and must be enabled before enabling the SMPS." (6.1 in RM0453)
	SUBGRF_WriteRegister(SUBGHZ_SMPSC0R, (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE));
	SUBGRF_SetRegulatorMode();

	// Use the whole 256-byte buffer for both TX and RX
	SUBGRF_SetBufferBaseAddress(0x00, 0x00);

	SUBGRF_SetRfFrequency(RF_FREQUENCY);
	SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
	SUBGRF_SetStopRxTimerOnPreambleDetect(false);

	SUBGRF_SetPacketType(PACKET_TYPE_LORA);

	SUBGRF_WriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
	SUBGRF_WriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );

	ModulationParams_t modulationParams;
	modulationParams.PacketType = PACKET_TYPE_LORA;
	modulationParams.Params.LoRa.Bandwidth = Bandwidths[LORA_BANDWIDTH];
	modulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)LORA_CODINGRATE;
	modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
	modulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)LORA_SPREADING_FACTOR;
	SUBGRF_SetModulationParams(&modulationParams);

	packetParams.PacketType = PACKET_TYPE_LORA;
	packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
	packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
	packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
	packetParams.Params.LoRa.PayloadLength = 0xFF;
	packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
	SUBGRF_SetPacketParams(&packetParams);

	//SUBGRF_SetLoRaSymbNumTimeout(LORA_SYMBOL_TIMEOUT);

	// WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
	// RegIqPolaritySetup @address 0x0736
	SUBGRF_WriteRegister( 0x0736, SUBGRF_ReadRegister( 0x0736 ) | ( 1 << 2 ) );
}


/**
 * @brief  Receive data trough SUBGHZSPI peripheral
 * @param  radioIrq  interrupt pending status information
 * @retval None
 */
void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
	char uartBuff[8];
	switch (radioIrq)
	{
	case IRQ_TX_DONE:
		eventReceptor = eventTxDone;
		// TFG. TX end time and ToA metering
		toaT1=HAL_GetTick();
		sprintf(uartBuff, "ToA: %lu", toaT1-toaT0);
		uartLog(uartBuff, 0, 1);
		break;
	case IRQ_RX_DONE:
		eventReceptor = eventRxDone;
		break;
	case IRQ_RX_TX_TIMEOUT:
		if (SUBGRF_GetOperatingMode() == MODE_TX)
		{
			eventReceptor = eventTxTimeout;
		}
		else if (SUBGRF_GetOperatingMode() == MODE_RX)
		{
			eventReceptor = eventRxTimeout;
		}
		break;
	case IRQ_CRC_ERROR:
		eventReceptor = eventRxError;
		break;
	default:
		break;
	}
}


/**
 * @brief  Process the TX Done event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventTxDone(clientFSM_t *const fsm)
{
	uartLog("Event TX Done", -1, 2);
	switch (fsm->state)
	{
	case STATE_MASTER:
		switch (fsm->subState)
		{
		case SSTATE_TX:
			enterMasterRx(fsm);
			fsm->subState = SSTATE_RX;
			break;
		default:
			break;
		}
		break;

		default:
			break;
	}
}


/**
 * @brief  Process the RX Done event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventRxDone(clientFSM_t *const fsm){
	char uartBuff[100];
	uartLog("Event RX Done", 0, 2);
	switch(fsm->state)
	{
	case STATE_MASTER:
		switch (fsm->subState)
		{
		case SSTATE_RX:
			transitionRxDone(fsm);
			// TFG. Data send verification.
			if (strncmp(fsm->rxBuffer, "ACK OK", 6) == 0){
				ackNokCnt=0;
				uartLog("RX <-- ACK", 0, 1);
				sprintf(uartBuff,"%d mili seconds pause, correct pay load sent", TX_SEND_TIMER);
				uartLog(uartBuff, 0, 1);
				// TFG. Initialize the TX buffer and set the flag
				for (int i=0; i < 40; i++){
					txPayload.txBuffer[i]=0;
				}
				txPayload.ready2TX = 0;
				enterMasterTx(fsm);
				fsm->subState = SSTATE_TX;
			}
			else{
				// TFG. 5 send retries
				if (ackNokCnt<5){
					HAL_Delay(fsm->randomDelay);
					enterMasterTx(fsm);
					fsm->subState = SSTATE_TX;
					ackNokCnt++;
				}
				else{
					sprintf(uartBuff,"%d mili seconds pause --> 5 TX without ACK", TX_SEND_TIMER);
					uartLog(uartBuff, 0, 1);
					// TFG. Initialize the TX buffer and set the flag
					for (int i=0; i < 40; i++){
						txPayload.txBuffer[i]=0;
					}
					txPayload.ready2TX = 0;
					HAL_Delay(TX_SEND_TIMER);
					ackNokCnt=0;
					enterMasterTx(fsm);
					fsm->subState = SSTATE_TX;
				}
			}
		default:
			break;
		}
	default:
		break;
	}
}


/**
 * @brief  Process the TX Timeout event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventTxTimeout(clientFSM_t *const fsm)
{
uartLog("\r\nEvent TX Timeout\r\n", 0, 1);
	switch (fsm->state)
	{
	case STATE_MASTER:
		switch (fsm->subState)
		{
		case SSTATE_TX:
			enterMasterRx(fsm);
			fsm->subState = SSTATE_RX;
			break;
		default:
			break;
		}
		break;
		default:
			break;
	}
}


/**
 * @brief  Process the RX Timeout event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventRxTimeout(clientFSM_t *const fsm)
{
	char uartBuff[100];
	uartLog("\r\nEvent RX Timeout\r\n", 0, 1);
	switch (fsm->state)
	{
	case STATE_MASTER:
		switch (fsm->subState)
		{
		case SSTATE_RX:
			// TFG. 5 send retries
			if (rxToutCnt<5){
				HAL_Delay(fsm->randomDelay);
				enterMasterTx(fsm);
				fsm->subState = SSTATE_TX;
				rxToutCnt++;
			}
			else {

				sprintf(uartBuff, "\r\n%d mili seconds pause --> 5 RX timeout\r\n", TX_SEND_TIMER);
				uartLog(uartBuff, 0, 1);
				// TFG. Initialize the TX buffer and set the flag
				for (int i=0; i < 40; i++){
					txPayload.txBuffer[i]=0;
				}
				txPayload.ready2TX = 0;
				HAL_Delay(TX_SEND_TIMER);
				rxToutCnt=0;
				enterMasterTx(fsm);
				fsm->subState = SSTATE_TX;
			}
			break;
		default:
			break;
		}
		break;
		default:
			break;
	}
}


/**
 * @brief  Process the RX Error event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventRxError(clientFSM_t *const fsm)
{
	uartLog("\r\nEvent Rx Error\r\n", 0, 1);
	switch (fsm->state)
	{
	case STATE_MASTER:
		switch (fsm->subState)
		{
		case SSTATE_RX:
			HAL_Delay(fsm->randomDelay);
			enterMasterTx(fsm);
			fsm->subState = SSTATE_TX;
			break;
		default:
			break;
		}
		break;
		default:
			break;
	}
}


/**
 * @brief  Entry actions for the RX sub-state of the Master state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterMasterRx(clientFSM_t *const fsm)
{
	uartLog("Master Rx start", 0, 4);
	SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
			IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
			IRQ_RADIO_NONE,
			IRQ_RADIO_NONE );
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
	packetParams.Params.LoRa.PayloadLength = 0xFF;
	SUBGRF_SetPacketParams(&packetParams);
	SUBGRF_SetRx(fsm->rxTimeout << 6);
}

/**
 * @brief  Entry actions for the TX sub-state of the Master state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterMasterTx(clientFSM_t *const fsm)
{
	// TFG. Buffer for store values
	char uartBuff[40];
	// TFG. Wait until the payload is full
	while(txPayload.ready2TX == 0);
	HAL_Delay(fsm->rxMargin);
	uartLog("TX --> data payload: ", 1, 1);
	for (int i = 0; i < txPayload.chCnt/2; i++) {
		uint16_t value = *((uint16_t*)(txPayload.txBuffer + i*2)); // TFG. Read values from char pay-load
		sprintf(uartBuff, "%d", value);
		uartLog(uartBuff, 2, 1);
	}
	uartLog("", 0, 1);

	uartLog("Master Tx start", 0, 4);
	SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
			IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
			IRQ_RADIO_NONE,
			IRQ_RADIO_NONE );
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
	// Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
	SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
	packetParams.Params.LoRa.PayloadLength = txPayload.chCnt;
	SUBGRF_SetPacketParams(&packetParams);

	// TFG. To estimate ToA
	toaT0=HAL_GetTick();
	SUBGRF_SendPayload((uint8_t *)txPayload.txBuffer, txPayload.chCnt, 0);
}


/**
 * @brief  Transition actions executed on every RX Done event (helper function)
 * @param  fsm pointer to FSM context
 * @retval None
 */
void transitionRxDone(clientFSM_t *const fsm)
{
	PacketStatus_t packetStatus;
	char uartBuff[50];

	// Workaround 15.3 in DS.SX1261-2.W.APP (because following RX w/ timeout sequence)
	SUBGRF_WriteRegister(0x0920, 0x00);
	SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02));

	SUBGRF_GetPayload((uint8_t *)fsm->rxBuffer, &fsm->rxSize, 0xFF);
	SUBGRF_GetPacketStatus(&packetStatus);

	//TFG. Info about RF parameters.
	sprintf(uartBuff, "RssiValue=%d dBm, SnrValue=%d dB", packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt);
	uartLog(uartBuff, 0, 2);
}

/**
 * @brief TFG. Set the alarm to on or off.
 * @param alarmTrigger	Integer value to set the alarm state
 * @retval 0 -> Alarm OFF, 1 -> Alarm ON, 2 -> Alarm Error
 */
int setAlarmState(int alarmTrigger){
	tim16cbDiv=0;
	if (alarmTrigger==1){
		HAL_GPIO_WritePin(GPIOA, LIGHT_Pin|SIREN_Pin, GPIO_PIN_SET);
		uartLog("\r\nALARM ON\r\n", 0, 1);
		return 1;
	}
	else if (alarmTrigger==0){
		HAL_GPIO_WritePin(GPIOA, LIGHT_Pin|SIREN_Pin, GPIO_PIN_RESET);
		uartLog("\r\nALARM OFF\r\n", 0, 1);
		return 0;
	}
	else{
		uartLog("\r\nALARM ON, system error\r\n", 0, 1);
		return 2;
	}

}

/**
 * @brief TFG. Evaluate the trhesholv criteria and turn on or off the alarm
 * @param channelArrayIn	Vector that stores the filtered values
 *
 */
void thresholdEval(uint16_t current, uint16_t voltage){
	char uartBuff[100];
	//TFG. If both signals exceed the threshold, the ON alarm counter is incremented and the OFF counter is reset to 0.
	if((current > THRESHOLD_VOL) && (voltage > THRESHOLD_AMP)){
		alarmCntOn++;
		alarmCntOff=0;
		// Only for debug level 4
		if (UART_LOG_LEVEL == 4 && alarmCntOn % 40 == 0){
			sprintf(uartBuff, "Alarm state ON counter: %d // current: %f // voltage: %f", alarmCntOn, (((float)current/4096)*16)+4, ((float)voltage/4096)*10);
			uartLog(uartBuff, 0, 4);
		}
	}
	//TFG. If any of the signals does not exceed the threshold, the OFF alarm counter is incremented and the ON counter is reset to 0.
	else if ((current <= THRESHOLD_VOL) || (voltage <= THRESHOLD_AMP)){
		alarmCntOff++;
		alarmCntOn=0;
		// Only for debug level 4
		if (UART_LOG_LEVEL == 4 && alarmCntOff % 40 == 0){
			sprintf(uartBuff, "Alarm state OFF counter: %d // current: %f // voltage: %f", alarmCntOff, (((float)current/4096)*16)+4, ((float)voltage/4096)*10);
			uartLog(uartBuff, 0, 4);
		}
	}
	//TFG. If the counter to ON exceed the threshold, the alarm it is set to ON.
	if (alarmCntOn >= (THRESHOLD_TIME/ADC_EVAL_TIMER)){
		if (alarmState!=1){
			alarmState=setAlarmState(1);
		}
		alarmCntOn=0;
	}
	//TFG. If the counter to OFF exceed the threshold, the alarm it is set to OFF.
	else if (alarmCntOff >= (THRESHOLD_TIME/ADC_EVAL_TIMER)){
		if (alarmState!=0){
			alarmState=setAlarmState(0);
		}
		alarmCntOff=0;
	}
}

/**
 * @brief TFG. Print log messages over UART port.
 * @param text		Text message
 * @param escChar	Separator character
 * @param level		Log level
 */
void uartLog(char* text, int escChar, int level){
	char uartBuff[100];
	if (escChar==0){
		sprintf(uartBuff, "%s\r\n", text);
	}
	if (escChar==1){
		sprintf(uartBuff, "%s", text);
	}
	if (escChar==2){
		sprintf(uartBuff, "%s ", text);
	}
	if (escChar==2){
		sprintf(uartBuff, "%s, ", text);
	}
	if (level <= UART_LOG_LEVEL ){
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
	}
	else{
	}
}

/**
 * @brief TFG. Elapsed time interrupt.
 * @param *htim		Pointer to htim handler
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	/* USER CODE BEGIN Callback 1 */

	if (htim->Instance == TIM16){
		//TFG. Variable to store the time
		htpecT1 = HAL_GetTick();

		//TFG. If alarm is ON a counter is incremented every 10 ms to toggle the alarm light every 1 second.
		if (alarmState == 1) {
			HAL_GPIO_WritePin(SIREN_GPIO_Port, SIREN_Pin, GPIO_PIN_SET);
			// While alarm==1 -> SIREN ALARM ON
			if (tim16cbDiv > 100){ // whith alarm==1 from 0 ms to 1000 ms -> ALARM LIGHT Toggle.
				HAL_GPIO_TogglePin(LIGHT_GPIO_Port, LIGHT_Pin);
				tim16cbDiv=0;
			}
			else{
				tim16cbDiv++;
			}
		}

		//TFG. Save samples every second into TX pay-load. The time is estimated with HAL_GetTick().
		if (htpecT1-htpecT0 >= ADC_SAMPLE_2_TX_TIMER && txPayload.ready2TX != 1){
			htpecT0=hacccT1;
			char uartBuff[TX_BUFFER_SIZE];
			//TFG. Save average values of voltage and current to the correct position in pay-load.
			if (sampleCnt < TX_BUFFER_SIZE/4 && txPayload.ready2TX == 0){
				memcpy(txPayload.txBuffer + (sampleCnt*4), &currentAVG, sizeof(currentAVG));
				memcpy(txPayload.txBuffer + ((sampleCnt*4)+2), &voltageAVG, sizeof(voltageAVG));
				sprintf(uartBuff, "Current %d: %d\r\nVoltage %d: %d", (sampleCnt), *((uint16_t*)(txPayload.txBuffer + sampleCnt*4)), (sampleCnt), *((uint16_t*)(txPayload.txBuffer + (sampleCnt*4) + 2)));
				uartLog(uartBuff, 0, 3);
				sampleCnt++;
			}
			//TFG. When pay-load is full set the flag and reset counter to 0.
			else if (sampleCnt == TX_BUFFER_SIZE/4){
				txPayload.chCnt = sizeof(txPayload.txBuffer);
				txPayload.ready2TX = 1;
				sampleCnt=0;
			}
		}
	}
	/* USER CODE END Callback 1 */
}
/**
 * @brief TFG. Callback for end of conversion interrupt
 *  * @param *hadc		Pointer to hadc handler
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	// TFG. AVG value of each DMA-ADC sample array.
	uint32_t tmpCurrentAVG=0;
	uint32_t tmpVoltageAVG=0;
	hacccT1 = HAL_GetTick();
	//TFG. When the defined time has passed between each conversion, calculate two average values.
	if (hacccT1-hacccT0 >= ADC_EVAL_TIMER){
		hacccT0 = hacccT1;
		for (int i = 0; i < ADC_DMA_BUFFER_SIZE; i++) {
			if (i % 2 == 0){
				tmpCurrentAVG=tmpCurrentAVG+ADCbuffer[i];
			}
			else{
				tmpVoltageAVG=tmpVoltageAVG+ADCbuffer[i];
			}
		}
		currentAVG=(tmpCurrentAVG/(ADC_DMA_BUFFER_SIZE/2));
		voltageAVG=(tmpVoltageAVG/(ADC_DMA_BUFFER_SIZE/2));
		//TFG. Evaluate the threshold.
		thresholdEval(currentAVG, voltageAVG);
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
