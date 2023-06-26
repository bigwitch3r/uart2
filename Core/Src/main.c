/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx.h"
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Register Definitions for LIS302DL
#define LIS302DL_ADDR     (0x3B)
#define WHO_AM_I          (0x0F)
#define CTRL_REG1         (0x20)
#define CTRL_REG2         (0x21)
#define CTRL_REG3         (0x22)
#define HP_FILTER_RESET   (0x23)
#define STATUS_REG        (0x27)
#define OUT_X             (0x29)
#define OUT_Y             (0x2B)
#define OUT_Z             (0x2D)

// Calibration constants
#define X_OFFSET 18
#define THRESH_LOW -120
#define THRESH_HIGH 120
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char trans_str[64] = {0,};

uint16_t x,y,z;
int16_t x_final, y_final, z_final;
uint16_t rxd,rxdf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void RCC_Init(void);
void GPIO_Init(void);
void SPI_Init(void);
uint16_t SPI_Transmit(uint8_t data);
uint16_t SPI_Receive(uint8_t addr);
void LIS_Init(void);
void LIS_Write(uint8_t addr, uint8_t data);
void LIS_Read(void);
int16_t Convert_To_Val(uint16_t val);
void TIM4_ms_Delay(uint16_t delay);
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
  uint8_t str[]="USART Transmit\r\n";
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  GPIO_Init();
  SPI_Init();
  LIS_Init();
  GPIOE->BSRR |= GPIO_BSRR_BS3;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Call the read function
	  LIS_Read();
	  // Use the Convert_To_Val function to convert raw data into actual data
	  x_final = Convert_To_Val(x) + X_OFFSET;
	  y_final = Convert_To_Val(y);

	  snprintf(trans_str, 63, "{\"x\":%d,\"y\":%d,\"z\":%d}", x_final, y_final, z);
	  HAL_UART_Transmit(&huart2, (uint8_t*)trans_str, strlen(trans_str), 1000);


	  HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void GPIO_Init(){
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configuring PA5, PA6, PA7 in alternate function mode
    GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);

    // Select AF5 for SPI on PA5, PA6, PA7
    GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL5_2 | GPIO_AFRL_AFSEL5_0
                     | GPIO_AFRL_AFSEL6_2 | GPIO_AFRL_AFSEL6_0
                     | GPIO_AFRL_AFSEL7_2 | GPIO_AFRL_AFSEL7_0);

    // Enable GPIOE clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    // Since PE3 is CS, it needs to be configured in Output Mode
    GPIOE->MODER |= GPIO_MODER_MODER3_0;

    GPIOA->OSPEEDR |= ( GPIO_OSPEEDER_OSPEEDR5_0 | GPIO_OSPEEDER_OSPEEDR6_0 |
                        GPIO_OSPEEDER_OSPEEDR7_0);

    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD5_1 | GPIO_PUPDR_PUPD6_1 | GPIO_PUPDR_PUPD7_1);

    // Enable clock for GPIOD and Configure PD12 in output mode
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
}

void SPI_Init(){
    // Enable SPI clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Select the Master Configuration
    SPI1->CR1 |= SPI_CR1_MSTR;

    SPI1->CR1 &= ~SPI_CR1_BIDIMODE;

    SPI1->CR1 &= ~SPI_CR1_RXONLY;

    // Set the Data Frame Format (DFF) to '0' or 8-bit.
    SPI1->CR1 &= ~SPI_CR1_DFF;

    // SSI and SSM bits in the SP1->CR1 register need to be set
    // to '1'
    SPI1->CR1 |= (SPI_CR1_SSI | SPI_CR1_SSM);

    // Setting Baud Rate
    SPI1->CR1 &= ~SPI_CR1_BR;

    // Set the transmission to MSB First Mode
    SPI1->CR1 &= ~SPI_CR1_LSBFIRST;

    // Configure CPOL and CPHASE to '0' and '0', respectively.
    // i.e. Clock is at '0' when idle, and data capture is done
    // on the first clock transition which is the rising edge.
    SPI1->CR1 &= ~SPI_CR1_CPHA;
    SPI1->CR1 &= ~SPI_CR1_CPOL;

    // Enable CRC
    SPI1->CR1 |= SPI_CR1_CRCEN;

    // Enable SPI
    SPI1->CR1 |= SPI_CR1_SPE;

    // Selecting Motorola Format
    SPI1->CR2 = 0x0000;
}

uint16_t SPI_Transmit(uint8_t data){
    while(!((SPI1->SR) & SPI_SR_TXE)){}
    SPI1->DR = data;

    while(!(SPI1->SR & SPI_SR_RXNE)){}
    rxd = SPI1->DR;

    return rxd;
}

uint16_t SPI_Receive(uint8_t addr){
    GPIOE->BSRR |= GPIO_BSRR_BR3;
    addr |= 0x80;
    SPI_Transmit(addr);
    rxdf = SPI_Transmit(0);
    GPIOE->BSRR |= GPIO_BSRR_BS3;
    return rxdf;
}

void LIS_Write(uint8_t addr,uint8_t data){
    // Selecting the LIS accelerometer
    GPIOE->BSRR |= GPIO_BSRR_BR3;

    // Send the Register Address
    SPI_Transmit(addr);

    // Send the data to be written
    SPI_Transmit(data);

    // De-select the accelerometer
    GPIOE->BSRR |= GPIO_BSRR_BS3;
}

void LIS_Init(){
    // Powering on the accelerometer and Enabling the x,y and z axis for acceleration capture
    LIS_Write(CTRL_REG1, 0x47);
}

void LIS_Read(){
    // Reading the data for x-axis
    x = SPI_Receive(OUT_X);

    // Reading the data for y-axis
    y = SPI_Receive(OUT_Y);

    // Reading the data for z-axis
    z = SPI_Receive(OUT_Z);
}

void TIM4_ms_Delay(uint16_t delay){
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //Enable the clock for TIM3
    TIM4->PSC = 16000-1; //Set the clock frequency to 1KHz
    TIM4->ARR = delay; // Get the required delay from user
    TIM4->CNT = 0;
    TIM4->CR1 |= 1; // Start the timer
    while(!(TIM4->SR & 1)){} // Wait for the "Update Interrupt Flag"
    TIM4->SR &= ~(0x0001); // Clear the "Update Interrupt Flag"
}

int16_t Convert_To_Val(uint16_t val){
    if ((val & 0x80) == 0x80){
        val = ~val;
        val += 1;
        val &= 0x00FF;
        val = ( val * 2300 ) / 127;
        return (-1 * val);
    }
    else
        return (( val * 2300 ) / 127);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
