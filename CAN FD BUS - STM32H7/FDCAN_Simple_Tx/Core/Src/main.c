/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include <stdbool.h>

#define NODE_A_ID 1

#define ID_NODE_A_TX 0x100

#define DHT_PORT Output_GPIO_Port
#define DHT_PIN Output_Pin

#define MMA8451_ADDR        (0x1D << 1) // Dirección I2C (shifted)
#define WHO_AM_I_REG        0x0D
#define CTRL_REG1           0x2A
#define OUT_X_MSB           0x01


#define MMA8451_ADDR        (0x1D << 1) // Dirección I2C (shifted)
#define WHO_AM_I_REG        0x0D
#define CTRL_REG1           0x2A
#define OUT_X_MSB           0x01






uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, reflex;
uint16_t SUM;

uint16_t ADC_VAL;
float voltage=0;
float angle=0;


typedef struct {
    float Temperature;
    float Humidity;
} DHT_DataTypedef;

DHT_DataTypedef DHT11_Data;

float Temperature, Humidity;


int16_t ax, ay, az;
float ax_g, ay_g, az_g;

char msg[300];

volatile uint32_t pulse_count = 0;
uint32_t last_time = 0;
uint32_t rpm = 0;

uint32_t last_interrupt_time = 0;
const uint32_t debounce_ms = 10;


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
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


uint8_t Node_ID = NODE_A_ID;


// Datos CAN
FDCAN_TxHeaderTypeDef TxHeader;
//uint8_t TxData[1];
uint8_t TxData[64]; // 32-bit integer


float float1 = 3.1416f; // Valor de ejemplo a enviar
float float2 = 2.7182f;
float float3 = 1.34f;
uint8_t X=1;


//float receivedFloat = 0.0f;  // Variable para almacenar el float recibido
//char msg[50];



void SendFloat(float value1, float value2, float value3, float value4, float value5, float value6, float value7, float value8)
{
    // Copiar los floats al buffer
    memcpy(&TxData[0], &value1, sizeof(float));
    memcpy(&TxData[4], &value2, sizeof(float));
    memcpy(&TxData[8], &value3, sizeof(float));   // Angle
    memcpy(&TxData[12], &value4, sizeof(float));
    memcpy(&TxData[16], &value5, sizeof(float));
    memcpy(&TxData[20], &value6, sizeof(float));
    memcpy(&TxData[24], &value7, sizeof(float));
    memcpy(&TxData[28], &value8, sizeof(float));

    // Configurar el header (asegúrate de que esto ya está inicializado)
    TxHeader.DataLength = FDCAN_DLC_BYTES_64;  // 12 bytes
    TxHeader.IdType = FDCAN_STANDARD_ID;       // ID estándar

    // Enviar
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {
        // Manejar error de transmisión
        Error_Handler();
    }
}



//CODE for Sensor

void delay (uint16_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim6,0);
	while (__HAL_TIM_GET_COUNTER(&htim6) < delay);
}


uint8_t Presence = 0;

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


void DHT_Start (void)
{
	Set_Pin_Output (DHT_PORT, DHT_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 0);   // pull the pin low


	delay (18000);   // wait for 18ms

    HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 1);   // pull the pin high
    delay (20);   // wait for 30us
	Set_Pin_Input(DHT_PORT, DHT_PIN);    // set as input
}

uint8_t DHT_Check_Response (void)
{
	uint8_t Response = 0;
	delay (40);
	if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))
	{
		delay (80);
		if ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN))) Response = 1;
		else Response = -1;
	}
	while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));   // wait for the pin to go low

	return Response;
}


uint8_t DHT_Read (void)
{
    uint8_t i = 0, j = 0;
    for (j = 0; j < 8; j++)
    {
        uint32_t timeout = 0;
        while (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)) && timeout++ < 1000);
        delay (40);
        if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))
        {
            i &= ~(1 << (7 - j)); // Escribe 0 en el bit
        }
        else {
            i |= (1 << (7 - j));  // Escribe 1 en el bit
        }

        timeout = 0;
        while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)) && timeout++ < 1000);
    }
    return i;
}




void DHT_GetData (DHT_DataTypedef *DHT_Data)
{
    DHT_Start ();
	Presence = DHT_Check_Response ();
	Rh_byte1 = DHT_Read ();
	Rh_byte2 = DHT_Read ();
	Temp_byte1 = DHT_Read ();
	Temp_byte2 = DHT_Read ();
	SUM = DHT_Read();

	if (SUM == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))
	{
		DHT_Data->Temperature = Temp_byte1;
		DHT_Data->Humidity = Rh_byte1;

	}
}


uint8_t read_register(uint8_t reg) {
    uint8_t value;
    HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR, reg, 1, &value, 1, HAL_MAX_DELAY);
    return value;
}

void write_register(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, MMA8451_ADDR, reg, 1, &value, 1, HAL_MAX_DELAY);
}


int16_t convert_accel(uint8_t msb, uint8_t lsb) {
    int16_t value = (msb << 8) | lsb;
    value = value >> 2;
    if (value & (1 << 13)) {
        value |= 0xC000;
    }
    return value;
}

void read_accel(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR, OUT_X_MSB, 1, data, 6, HAL_MAX_DELAY);

    *x = convert_accel(data[0], data[1]);
    *y = convert_accel(data[2], data[3]);
    *z = convert_accel(data[4], data[5]);
}






/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == rpm_in_Pin)
    {
        uint32_t now = HAL_GetTick();
        if (now - last_interrupt_time > debounce_ms)
        {
            pulse_count++;
            last_interrupt_time = now;
        }
    }
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
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  last_time = HAL_GetTick();


  HAL_TIM_Base_Start(&htim6);
  //HAL_TIM_Base_Start_IT(&htim7);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // Configurar header de transmisión
  TxHeader.Identifier = ID_NODE_A_TX;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_64;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  int16_t ax, ay, az;
  uint8_t who_am_i = read_register(WHO_AM_I_REG);
  if (who_am_i != 0x1A) {  // El valor esperado para MMA8451
      Error_Handler();
  }

  // Configurar el acelerómetro
  write_register(CTRL_REG1, 0x01);  // Activar a 800Hz en modo activo
  HAL_Delay(10);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      DHT_GetData(&DHT11_Data);
      Temperature = DHT11_Data.Temperature;
      Humidity = DHT11_Data.Humidity;


	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  ADC_VAL=HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);

	  voltage = (float)(ADC_VAL*3.3f)/ 65535.0f;
	  angle = (voltage*235)/3.3;

	  read_accel(&ax, &ay, &az);

	  float ax_ms2 = ax / 4096.0f * 9.81f;
	  float ay_ms2 = ay / 4096.0f * 9.81f;
	  float az_ms2 = az / 4096.0f * 9.81f;

	  uint32_t now = HAL_GetTick();

	  if (now - last_time >= 1000) // Cada 1 segundo
	  {
		  rpm = pulse_count * 60;
		  pulse_count = 0;
		  last_time = now;
	  }

	  //&& (now - last_time >= 1000)

	 if ( (Node_ID == NODE_A_ID)  )
	  {
		  SendFloat(rpm, Humidity, angle, ax_ms2, ay_ms2, az_ms2, Temperature, float2);
		  HAL_Delay(1000);

	  // Cambiar el valor para el próximo envío (opcional)
	  //float1 += 0.1f;
	  }


	  snprintf(msg, sizeof(msg), "Temperature: %.2f, Humidity: %.2f \r\n Angle: %.2f \r\n Ax: %.2f, Ay: %.2f Az: %.2f \r\n RPM: %.f Float1 %.2f \r\n Float2 %.2f \r\n", Temperature, Humidity, angle, ax_ms2, ay_ms2, az_ms2, (float)rpm, float1,float2);
	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	  HAL_Delay(1000);



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 11;
  hfdcan1.Init.NominalTimeSeg1 = 68;
  hfdcan1.Init.NominalTimeSeg2 = 11;
  hfdcan1.Init.DataPrescaler = 16;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 2;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_24;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_24;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_24;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_24;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 64-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  htim7.Init.Prescaler = 64-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0xffff-1;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Output_GPIO_Port, Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Output_Pin */
  GPIO_InitStruct.Pin = Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : rpm_in_Pin */
  GPIO_InitStruct.Pin = rpm_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(rpm_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(rpm_in_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(rpm_in_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
























































































#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"


//CAN BUS
#define NODE_A_ID 1
#define ID_NODE_A_TX 0x100


//Temperature - Humidity
#define DHT_PORT Output_GPIO_Port
#define DHT_PIN Output_Pin

//Accelerometer
#define MMA8451_ADDR        (0x1D << 1) // Dirección I2C (shifted)
#define WHO_AM_I_REG        0x0D
#define CTRL_REG1           0x2A
#define OUT_X_MSB           0x01



//BUFFER message
char msg[300];


//Temperature - Humidity
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, reflex;
uint16_t SUM;
typedef struct {
    float Temperature;
    float Humidity;
} DHT_DataTypedef;
float Temperature, Humidity;

DHT_DataTypedef DHT11_Data;


//Angle
uint16_t ADC_VAL;
float voltage=0;
float angle=0;


//Acceleration
int16_t ax, ay, az;
float ax_g, ay_g, az_g;


//RPM
volatile uint32_t pulse_count = 0;
uint32_t last_time = 0;
uint32_t rpm = 0;

uint32_t last_interrupt_time = 0;
const uint32_t debounce_ms = 10;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t Node_ID = NODE_A_ID;

// Datos CAN
FDCAN_TxHeaderTypeDef TxHeader;


uint8_t TxData[32]; // 3 floats x 4 bytes



float float1 = 3.1416f; // Valor de ejemplo a enviar
float float2 = 2.7182f;
float float3 = 1.34f;
float float4 = 3.70f;
float float5 = 9.18f;
float float6 = 6.79f;
float float7 = 5.93f;
float float8 = 10.99f;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//SEND CAN BUS
void SendFloat(float value1,float value2,float value3,float value4,float value5,float value6,float value7)
{
    // Copiar los floats al buffer
    memcpy(&TxData[0],  &value1, sizeof(float));
    memcpy(&TxData[4],  &value2, sizeof(float));
    memcpy(&TxData[8],  &value3, sizeof(float));
    memcpy(&TxData[12], &value4, sizeof(float));
    memcpy(&TxData[16], &value5, sizeof(float));
    memcpy(&TxData[20], &value6, sizeof(float));
    memcpy(&TxData[24], &value7, sizeof(float));
    //memcpy(&TxData[28], &value8, sizeof(float));

    // Configurar el header (asegúrate de que esto ya está inicializado)
    TxHeader.DataLength = FDCAN_DLC_BYTES_32;  // 12 bytes
    TxHeader.IdType = FDCAN_STANDARD_ID;       // ID estándar

    // Enviar
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {
        // Manejar error de transmisión
        Error_Handler();
    }
}



// ***** CODE for Sensor DHT-11 Temperature Humidity Sensor

void delay (uint16_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim6,0);
	while (__HAL_TIM_GET_COUNTER(&htim6) < delay);
}
uint8_t Presence = 0;
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT_Start (void)
{
	Set_Pin_Output (DHT_PORT, DHT_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 0);   // pull the pin low


	delay (18000);   // wait for 18ms

    HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 1);   // pull the pin high
    delay (20);   // wait for 30us
	Set_Pin_Input(DHT_PORT, DHT_PIN);    // set as input
}

uint8_t DHT_Check_Response (void)
{
	uint8_t Response = 0;
	delay (40);
	if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))
	{
		delay (80);
		if ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN))) Response = 1;
		else Response = -1;
	}
	while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT_Read (void)
{
    uint8_t i = 0, j = 0;
    for (j = 0; j < 8; j++)
    {
        uint32_t timeout = 0;
        while (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)) && timeout++ < 1000);
        delay (40);
        if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))
        {
            i &= ~(1 << (7 - j)); // Escribe 0 en el bit
        }
        else {
            i |= (1 << (7 - j));  // Escribe 1 en el bit
        }

        timeout = 0;
        while ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)) && timeout++ < 1000);
    }
    return i;
}

void DHT_GetData (DHT_DataTypedef *DHT_Data)
{
    DHT_Start ();
	Presence = DHT_Check_Response ();
	Rh_byte1 = DHT_Read ();
	Rh_byte2 = DHT_Read ();
	Temp_byte1 = DHT_Read ();
	Temp_byte2 = DHT_Read ();
	SUM = DHT_Read();

	if (SUM == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))
	{
		DHT_Data->Temperature = Temp_byte1;
		DHT_Data->Humidity = Rh_byte1;

	}
}


// ***** CODE for Acceleration Sensor
uint8_t read_register(uint8_t reg) {
    uint8_t value;
    HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR, reg, 1, &value, 1, HAL_MAX_DELAY);
    return value;
}

void write_register(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, MMA8451_ADDR, reg, 1, &value, 1, HAL_MAX_DELAY);
}

int16_t convert_accel(uint8_t msb, uint8_t lsb) {
    int16_t value = (msb << 8) | lsb;
    value = value >> 2;
    if (value & (1 << 13)) {
        value |= 0xC000;
    }
    return value;
}

void read_accel(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, MMA8451_ADDR, OUT_X_MSB, 1, data, 6, HAL_MAX_DELAY);

    *x = convert_accel(data[0], data[1]);
    *y = convert_accel(data[2], data[3]);
    *z = convert_accel(data[4], data[5]);
}


// ***** CODE Interruption for RPM Sensor
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == rpm_in_Pin)
    {
        uint32_t now = HAL_GetTick();
        if (now - last_interrupt_time > debounce_ms)
        {
            pulse_count++;
            last_interrupt_time = now;
        }
    }
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
  MX_FDCAN1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  //RPM flag
  last_time = HAL_GetTick();
  //Timer 6 flag start DHT-11
  HAL_TIM_Base_Start(&htim6);

  //Starter CAN Bus
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // Configurar header de transmisión
  TxHeader.Identifier = ID_NODE_A_TX;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_64;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;


  //Acceration starter
  int16_t ax, ay, az;
  uint8_t who_am_i = read_register(WHO_AM_I_REG);
  if (who_am_i != 0x1A) {  // El valor esperado para MMA8451
      Error_Handler();
  }
  // Config. Acceleration
  write_register(CTRL_REG1, 0x01);  // Activar a 800Hz en modo activo
  HAL_Delay(10);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //DHT Start
      DHT_GetData(&DHT11_Data);
      Temperature = DHT11_Data.Temperature;
      Humidity = DHT11_Data.Humidity;

      //Angle ADC1
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  ADC_VAL=HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);
	  voltage = (float)(ADC_VAL*3.3f)/ 65535.0f;
	  angle = (voltage*235)/3.3;

	  //Read accel
	  read_accel(&ax, &ay, &az);
	  float ax_ms2 = ax / 4096.0f * 9.81f;
	  float ay_ms2 = ay / 4096.0f * 9.81f;
	  float az_ms2 = az / 4096.0f * 9.81f;

	  //Function RPM
	  uint32_t now = HAL_GetTick();
	  if (now - last_time >= 1000) // Cada 1 segundo
	  {
		  rpm = pulse_count * 60;
		  pulse_count = 0;
		  last_time = now;
	  }

	  //Send CAN Bus Data
	  if ( (Node_ID == NODE_A_ID)  )
	  {
		  SendFloat(Temperature,Humidity,angle,ax_ms2,ay_ms2,az_ms2,(float)rpm);
		  HAL_Delay(1000);

	  // Cambiar el valor para el próximo envío (opcional)
	  //float1 += 0.1f;
	  }

	  //Debugger
	  snprintf(msg, sizeof(msg), "f1: %.2f, f2: %.2f, f3: %.2f  \r\n  f4: %.2f, f5: %.2f \r\n f6: %.2f, f7: %.2f \r\n", Temperature,Humidity,angle,ax_ms2,ay_ms2,az_ms2,float1);
	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	  HAL_Delay(1000);

  }
  /* USER CODE END 3 */
}
