/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Program obslugujacy stacje pogodowa dokonujaca pomiarow
  * 	temperatury, wilgotnosci, cisnienia, naslonecznienia oraz wyswietlajacej te dane
  * 	na wyswietlaczu. Obsluguje takze zegar RTC, z ktorego czas tez jest wyswietlany.
  * 	Program powstaje w ramach przedmiotu PMIK na Politechnice Warszawskiej
  *
  * 	JEST TO WERSJA ROBOCZA, NIE OBSLUGUJE JESZCZE WSZYSTKICH FUNKCJONALNOSCI
  * 		TE BEDA STOPNIOWO DODAWANE
  * @authors		: Artur Skrzypczak, Illia kovalenko
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
#include "ST7735.h"
#include "GFX_FUNCTIONS.h"

#include <stdio.h> //dla wykorzystania sprintf

#include "bme280.h"
#include "bme280_stm32_hal.h"

#include "bh1750.h"

#include <string.h>
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t getdata;

RTC_TimeTypeDef sTime; //2 struktury
RTC_DateTypeDef sDate;
char timeBuf[16]; //godzina
char dateBuf[16]; //data



struct bme280_dev bme;
struct bme280_data sensor_data;
uint8_t bme280_addr = BME280_I2C_ADDR_PRIM;  // 0x76

char debugBuf[32];  // Tablica pomocnicza do tworzenia napisów wyświetlanych na ekranie czujnik bh1750


volatile uint8_t setting_state = 0; // do ustawiania zegara
//0 - brak, 1 - godzina, 2 - minuty, 3 - dzien, 4 - miesiąc, 5 - rok

volatile uint32_t last_button1_press = 0;
volatile uint32_t last_button2_press = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void BT_SendString(char *str);
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
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Uruchomienie przerwań od timera TIM1 (używane do okresowego odświeżania danych)
    HAL_TIM_Base_Start_IT(&htim1);

    // Konfiguracja bieżącego czasu zegara RTC (np. po uruchomieniu mikrokontrolera)
    // Ustawiamy czas: 00:00:00
    sTime.Hours = 0;
    sTime.Minutes = 0;
    sTime.Seconds = 0;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    // Ustawiamy datę: Poniedziałek, 1 stycznia 2025
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 1;
    sDate.Year = 25;
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // Dezaktywacja WakeUp Timer RTC — musi być wykonana przed jego ponowną konfiguracją
    // RTC nie pozwala nadpisać aktywnego timera bez wcześniejszego wyłączenia
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    // Inicjalizacja wyświetlacza ST7735 i czyszczenie ekranu
    ST7735_Init(0);
    fillScreen(BLACK);

    // Konfiguracja struktury czujnika BME280 (wilgotność, temperatura, ciśnienie)
    bme.intf = BME280_I2C_INTF;
    bme.intf_ptr = &bme280_addr;
    bme.read = user_i2c_read;
    bme.write = user_i2c_write;
    bme.delay_us = user_delay_us;

    // Ustawienie parametrów pomiarowych BME280
    struct bme280_settings settings;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.filter = BME280_FILTER_COEFF_2;
    settings.standby_time = BME280_STANDBY_TIME_1000_MS;

    uint8_t settings_sel = BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP |
                           BME280_SEL_OSR_HUM | BME280_SEL_FILTER;

    if (bme280_init(&bme) != BME280_OK) {
        Error_Handler();
    }
    bme280_set_sensor_settings(settings_sel, &settings, &bme);

    // Ponowna dezaktywacja WakeUp Timer (na wypadek jeśli był jeszcze aktywny)
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    // Aktywacja WakeUp Timer RTC z przerwaniem — co określony czas wywoła callback
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 4095, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

    // Inicjalizacja czujnika światła BH1750 w trybie ciągłego pomiaru o wysokiej rozdzielczości
    BH1750_Init(&hi2c2, BH1750_CONT_HIGH_RES_MODE);
    char msg[] = "Hello from STM32\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (getdata)
	      {
	          getdata = 0;//zerowanie flagi od tim1



	          // Odczyt danych z BME280
	          bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &bme);
	          /*Tryb FORCED oznacza, ze czujnik sie budzi, aby wykonac jeden pomiar, po czym
	           * przechodzi w stan uspienia SLEEP. W tej wersji wybudzamy go na poczatku petli,
	           * potem idzie spac, co zmniejsza pobor pradu.
	           * Inna mozliwoscia jest zastosowanie trybu NORMAL, w ktorym bme wykonuje pomiary ciagle,
	           * bez przechodzenia w SLEEP. Wowczas: BME280_POWERMODE_NORMAL*/
	          bme.delay_us(2000 * 1000, bme.intf_ptr);//odczekanie, az sie obudzi i zmierzy
	          if (bme280_get_sensor_data(BME280_ALL, &sensor_data, &bme) == BME280_OK) {

	              char buf[32];//tablica pomocnicza do wyswietlania danych

	              //wyswietlanie temperatury
	              sprintf(buf, "%.1f 'C", sensor_data.temperature);
	              //drawString(10, 30, buf, WHITE);
	              ST7735_WriteString(10, 10, buf, Font_16x26, WHITE, BLACK);

	              //wyswietlanie wilgotnosci
	              sprintf(buf, "Hum: %.0f %%", sensor_data.humidity);
	              //fillRect(0, 100, 128, 10, BLACK);
	              // drawString(25, 100, buf, WHITE);
	              ST7735_WriteString(35, 100, buf, Font_7x10, WHITE, BLACK);

	              //wyswietlanie cisnienia
	              sprintf(buf, "Press: %.1f hPa", sensor_data.pressure / 100.0);
	              //fillRect(0, 120, 128, 10, BLACK);
	              //drawString(5, 120, buf, WHITE);
	              ST7735_WriteString(8, 120, buf, Font_7x10, WHITE, BLACK);


	              // ODCZYT DANYCH Z BH1750 – czujnik nasłonecznienia
	              // Tryb ciągły (CONTINUOUS HIGH RES MODE) pozwala odczytywać dane bez każdorazowego inicjalizowania pomiaru.
	              // Jednak ze względu na prostotę i dokładność, w tym przypadku wykonujemy pomiar ręcznie w każdej iteracji.

	              // Uruchomienie nowego pomiaru w trybie ustawionym przy inicjalizacji
	              BH1750_StartMeasurement();

	              // Odczekanie 180 ms – tyle potrzebuje czujnik BH1750 na wykonanie pomiaru
	              HAL_Delay(180);

	              // Odbiór 2 bajtów danych – surowa wartość nasłonecznienia
	              uint8_t data[2];
	              uint16_t lux;

	              // Odczyt danych z czujnika (adres BH1750 to 0x23 przesunięte w lewo o 1 bit = 0x46)
	              HAL_I2C_Master_Receive(&hi2c2, 0x46, data, 2, 100);

	              // Przetwarzanie danych – łączenie dwóch bajtów w jedną wartość 16-bitową (w luxach)
	              lux = (data[0] << 8) | data[1];

	              // Wyświetlenie zaktualizowaną wartość natężenia światła
	              sprintf(debugBuf, "Lux: %4u lx", lux);
	              ST7735_WriteString(25, 150, debugBuf, Font_7x10, WHITE, BLACK);

	              // Wysłanie danych pomiarowych przez Bluetooth (temperatura, wilgotność, ciśnienie, nasłonecznienie)
	              // Ciśnienie jest przeliczane z Pa na hPa (czyli podzielone przez 100)
	              char btBuf[64];
	              sprintf(btBuf, "Temp: %.1f C, Hum: %.0f%%, Press: %.1f hPa, Lux: %u\r\n",
	                      sensor_data.temperature,
	                      sensor_data.humidity,
	                      sensor_data.pressure / 100.0,
	                      lux);

	              BT_SendString(btBuf);  // Przesłanie przez UART2 do modułu HC-05 (Bluetooth)



	          }
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1) {
		getdata = 1;//flaga, po ktorej nastepuje pobranie wynikow i ich wyswietlanie
	}
}

/**
  * @brief  Funkcja wysyłająca ciąg znaków przez moduł Bluetooth (UART2 -> HC-05)
  * @param  str: wskaźnik do ciągu znaków (null-terminated)
  * @retval brak
  *
  * Funkcja korzysta z biblioteki HAL i wysyła dane przez UART2 z maksymalnym czasem oczekiwania.
  * W projekcie UART2 połączony jest z modułem HC-05 (Bluetooth), więc tekst trafia do komputera/terminala.
  */
void BT_SendString(char *str) {
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}



void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &sDate, RTC_FORMAT_BIN);
	sprintf(timeBuf, "%02d:%02d", sTime.Hours, sTime.Minutes);
	sprintf(dateBuf, "%02d:%02d:%02d", sDate.Date, sDate.Month, sDate.Year);
    //wyswietlanie godziny i daty
    ST7735_WriteString(45, 40, timeBuf, Font_7x10, WHITE, BLACK);
    ST7735_WriteString(35, 60, dateBuf, Font_7x10, WHITE, BLACK);
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t tick = HAL_GetTick();
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    if(GPIO_Pin == GPIO_PIN_10) // Zmiana trybu ustawiania
    {
        if(tick - last_button1_press > 200)
        {
            setting_state = (setting_state + 1) % 6; // 0..5
            last_button1_press = tick;
        }
    }
    else if(GPIO_Pin == GPIO_PIN_11) // Inkrementacja wartości
    {
        if(tick - last_button2_press > 200)
        {
            HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
            HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

            switch(setting_state)
            {
                case 1: // Godzina
                    sTime.Hours = (sTime.Hours + 1) % 24;
                    break;
                case 2: // Minuty
                    sTime.Minutes = (sTime.Minutes + 1) % 60;
                    break;
                case 3: // Dzień
                    sDate.Date = (sDate.Date % 31) + 1;
                    break;
                case 4: // Miesiąc
                    sDate.Month = (sDate.Month % 12) + 1;
                    break;
                case 5: // Rok
                    sDate.Year = (sDate.Year + 1) % 100; // 00 - 99
                    break;
                default:
                    break;
            }

            // Ustaw RTC
            HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
            HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
        	sprintf(timeBuf, "%02d:%02d", sTime.Hours, sTime.Minutes);
        	sprintf(dateBuf, "%02d:%02d:%02d", sDate.Date, sDate.Month, sDate.Year);


            ST7735_WriteString(45, 40, timeBuf, Font_7x10, WHITE, BLACK);
            ST7735_WriteString(35, 60, dateBuf, Font_7x10, WHITE, BLACK);

            last_button2_press = tick;
        }
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
