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
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "LCD.h"


#define LONG_CLICK_MIN 1500  //1.5sec
#define LONG_CLICK_MAX 5000  //5sec

#define DOUBLE_CLICK_MIN 15
#define DOUBLE_CLICK_MAX 120

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

/* USER CODE BEGIN PV */

char temp[100];
char ampm[2][3] = { "AM", "PM" };
RTC_TimeTypeDef sTime = { 0 };
RTC_DateTypeDef sDate = { 0 };

uint16_t ADC_Value;
uint32_t last_time;
uint32_t current_time;
uint32_t time_interval;
uint32_t last_time_interval;
uint8_t cursor_change = 0;


typedef enum{
	UP = 1,
	DOWN,
	RIGHT,
	LEFT,
	BASIC
}Clock_button;

typedef enum{
	NORMAL = 0,
	TIME_SETTING,
	ALARM_TIME_SETTING,
	MUSIC_SELECT
}Clock_mode;

typedef enum{
	Three_Bears = 1,
	Spring_Water
}Clock_music;

typedef struct{
	Clock_mode mode;
	Clock_button button;
	Clock_music music;
}Clock_state;


typedef struct{
	int16_t Hour1;
	int16_t Hour2;
	int16_t Min1;
	int16_t Min2;
	int16_t Sec1;
	int16_t Sec2;
	int16_t Ampm;

}Clock_time;


typedef struct{
	uint16_t one_flag;
	uint16_t alarm_flag;
	uint16_t two_flag;

}Flag;


typedef struct{
  int32_t time;
  GPIO_PinState level;
}ClickInfoDef;


ClickInfoDef click[3];

Clock_state  now_state;

Clock_time buf_time;
Clock_time now_time;
Clock_time alarm_time;


Flag flag;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void ADC_button();
void LCD_Cursor(int rownum, int column);
void blink(uint8_t location);
void set_time(uint16_t Ampm,uint16_t hour1,uint16_t hour2,uint16_t min1,uint16_t min2,uint16_t sec1,uint16_t sec2);
uint16_t value_limit(int16_t value);
void clock();
void Normal_mode();
void Time_setting_mode();
void Alarm_setting_mode();
void Music_select_mode();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart3, &ch, 1, 1000);
	return ch;
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
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	init();
	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);

	now_state.mode = NORMAL;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);


		clock();







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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* RCC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RCC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RCC_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  GPIO_PinState pin;
  int i;


  if(GPIO_Pin == GPIO_PIN_13)
  {
	current_time = HAL_GetTick();
    time_interval = current_time - last_time;
    last_time = current_time;

    pin = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);

//	printf("<%d,%d>\r\n",pin,time_interval);

    if(time_interval<=2) // noise
    {
//    	printf("Noise %d,%d\r\n",pin,time_interval);
    }
    else
    {

      click[2].time = click[1].time;
      click[2].level = click[1].level;

      click[1].time = click[0].time;
      click[1].level = click[0].level;

      click[0].time = time_interval;
      click[0].level = pin;

      if( click[2].level ==GPIO_PIN_RESET && click[1].level == GPIO_PIN_SET &&  click[0].level ==GPIO_PIN_RESET)
      {
    	  for(i=0;i<3;i++)
    	  {
    		  if(click[i].time>= DOUBLE_CLICK_MIN && click[i].time <= DOUBLE_CLICK_MAX)
    		  {
    			  continue;
    		  }
    		  else
    			  break;
    	  }
    	  if(i==3)
    	  {
    		  now_state.mode = MUSIC_SELECT;
    	  }
      }

	  if(click[0].level == GPIO_PIN_RESET && click[0].time >=LONG_CLICK_MIN) // long click
	  {
		  now_state.mode = ALARM_TIME_SETTING;
	  }
	  else if(click[0].level == GPIO_PIN_RESET && click[0].time < LONG_CLICK_MIN && click[0].time > DOUBLE_CLICK_MAX)
	  {
		  now_state.mode ^= TIME_SETTING;
	  }

    }
  }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM2) {
		ADC_button();
	}

}


void ADC_button(){
	HAL_ADC_Start(&hadc1);
	ADC_Value = HAL_ADC_GetValue(&hadc1);

	if(ADC_Value <= 100) now_state.button = UP;
	else if(ADC_Value>= 800 && ADC_Value<=900 ) now_state.button = DOWN;
	else if(ADC_Value>= 1900 && ADC_Value<=2000) now_state.button = LEFT;
	else if(ADC_Value>= 2900 && ADC_Value<=3100) now_state.button = RIGHT;
	else now_state.button = BASIC;
}

void LCD_Cursor(int row, int column) {
	if (row == 1)row = 0x40;
	LCD_SendCommand(LCD_ADDR, 0x80 | (row | column));
}

void set_time(uint16_t Ampm,uint16_t hour1,uint16_t hour2,uint16_t min1,uint16_t min2,uint16_t sec1,uint16_t sec2){



	sTime.TimeFormat = Ampm;
	sTime.Hours = hour1<<4 | hour2;
	sTime.Minutes = min1<<4 | min2;
	sTime.Seconds = sec1<<4 | sec2;

	if(sTime.Hours > 0x12) sTime.Hours = 0x10;


	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
}


void blink(uint8_t location){

	sprintf(temp,"%s %02x:%02x:%02x",sTime.TimeFormat[ampm],sTime.Hours,sTime.Minutes,sTime.Seconds);
	LCD_Cursor(1,0);
	LCD_SendString(LCD_ADDR, temp);

	if(location == 0){
		LCD_Cursor(1,location);
		LCD_SendString(LCD_ADDR, "  ");
	}else{
		LCD_Cursor(1,location);
		LCD_SendData(LCD_ADDR, ' ');
	}
}


uint16_t value_limit(int16_t value){
	if(now_state.button == UP) value++;
	else if(now_state.button == DOWN) value--;
	if(value > 9) value = 0;
	if(value < 0) value = 9;

	return value;
}




void clock(){

	if(now_state.mode == NORMAL) Normal_mode();
	else if(now_state.mode == TIME_SETTING)  Time_setting_mode();
	else if(now_state.mode == ALARM_TIME_SETTING) Time_setting_mode();
	else if(now_state.mode == MUSIC_SELECT) Music_select_mode();
}



void Normal_mode(){

	LCD_Cursor(0,0);
	LCD_SendString(LCD_ADDR, "Jin Ho Clock");
	sprintf(temp,"%s %02x:%02x:%02x",sTime.TimeFormat[ampm],sTime.Hours,sTime.Minutes,sTime.Seconds);
	LCD_Cursor(1,0);
	LCD_SendString(LCD_ADDR, temp);
}

void Time_setting_mode(){

	LCD_Cursor(0,0);
	LCD_SendString(LCD_ADDR, "Time Setting");

	now_time.Ampm = sTime.TimeFormat;
	now_time.Hour1 = (sTime.Hours & 0xf0) >> 4;
	now_time.Hour2 = sTime.Hours & 0x0f;
	now_time.Min1 = (sTime.Minutes & 0xf0) >> 4;
	now_time.Min2 = sTime.Minutes & 0x0f;
	now_time.Sec1 = (sTime.Seconds & 0xf0) >> 4;
	now_time.Sec2 = sTime.Seconds & 0x0f;


	switch (now_state.button) {
	case RIGHT:
		cursor_change++;
		break;
	case LEFT:
		cursor_change--;
		if (cursor_change < 0)cursor_change = 0;
		break;
	default:
		break;
	}


	switch(cursor_change%7){
	case 0:
		blink(0);
		if(now_state.button == UP || now_state.button == DOWN) now_time.Ampm ^= 1;
		break;
	case 1:
		blink(3);
		if(now_state.button == UP || now_state.button == DOWN) now_time.Hour1 ^= 1;
		break;
	case 2:
		blink(4);
		now_time.Hour2 = value_limit(now_time.Hour2);
		break;
	case 3:
		blink(6);
		now_time.Min1 = value_limit(now_time.Min1);
		break;
	case 4:
		blink(7);
		now_time.Min2 = value_limit(now_time.Min2);
		break;
	case 5:
		blink(9);
		now_time.Sec1 = value_limit(now_time.Sec1);
		break;
	case 6:
		blink(10);
		now_time.Sec2 = value_limit(now_time.Sec2);
		break;
	default:
		break;
	}

	if(now_time.Min1 > 5) now_time.Min1 = 0;
	if(now_time.Sec1 > 5) now_time.Sec1 = 0;

	set_time(now_time.Ampm,now_time.Hour1,now_time.Hour2,now_time.Min1,now_time.Min2,now_time.Sec1,now_time.Sec2);



}

void Alarm_setting_mode(){
	LCD_Cursor(0,0);
	LCD_SendString(LCD_ADDR, "Alarm Setting");

	alarm_time.Ampm = sTime.TimeFormat;
	alarm_time.Hour1 = (sTime.Hours & 0xf0) >> 4;
	alarm_time.Hour2 = sTime.Hours & 0x0f;
	alarm_time.Min1 = (sTime.Minutes & 0xf0) >> 4;
	alarm_time.Min2 = sTime.Minutes & 0x0f;
	alarm_time.Sec1 = (sTime.Seconds & 0xf0) >> 4;
	alarm_time.Sec2 = sTime.Seconds & 0x0f;


	switch (now_state.button) {
	case RIGHT:
		cursor_change++;
		break;
	case LEFT:
		cursor_change--;
		if (cursor_change < 0)cursor_change = 0;
		break;
	default:
		break;
	}


	switch(cursor_change%7){
	case 0:
		blink(0);
		if(now_state.button == UP || now_state.button == DOWN) now_time.Ampm ^= 1;
		break;
	case 1:
		blink(3);
		if(now_state.button == UP || now_state.button == DOWN) now_time.Hour1 ^= 1;
		break;
	case 2:
		blink(4);
		now_time.Hour2 = value_limit(now_time.Hour2);
		break;
	case 3:
		blink(6);
		now_time.Min1 = value_limit(now_time.Min1);
		break;
	case 4:
		blink(7);
		now_time.Min2 = value_limit(now_time.Min2);
		break;
	case 5:
		blink(9);
		now_time.Sec1 = value_limit(now_time.Sec1);
		break;
	case 6:
		blink(10);
		now_time.Sec2 = value_limit(now_time.Sec2);
		break;
	default:
		break;
	}

	if(now_time.Min1 > 5) now_time.Min1 = 0;
	if(now_time.Sec1 > 5) now_time.Sec1 = 0;


}



void Music_select_mode(){

}


//void clock_display() {
//
//	// set address to 0x00
//	LCD_SendCommand(LCD_ADDR, 0b10000000);
//	LCD_SendString(LCD_ADDR, "jinho clock");
//
//	// set address to 0x40
//	LCD_SendCommand(LCD_ADDR, 0b11000000);
//	LCD_SendString(LCD_ADDR, temp);
//
//}
//
//void set_time(uint16_t am_pm, uint16_t hours, uint16_t min, uint16_t sec) {
//	sTime.TimeFormat = am_pm;
//	sTime.Hours = hours;
//	sTime.Minutes = min;
//	sTime.Seconds = sec;
//	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//		Error_Handler();
//}
//
//uint8_t adc_polling() {
//
////	uint16_t adc_value;
//
//	HAL_ADC_Start(&hadc1);
//	//HAL_ADC_PollForConversion(&hadc1, 10);
//	adc_value = HAL_ADC_GetValue(&hadc1);
////	HAL_ADC_Stop(&hadc1);
//
//	if (1900 <= adc_value && adc_value <= 2000) {
//		//Left
//		return 1;
//	}
//	if (0 <= adc_value && adc_value <= 100) {
//		//Up
//		return 2;
//	}
//	if (800 <= adc_value && adc_value <= 900) {
//		//Down
//		return 3;
//	}
//	if (2900 <= adc_value && adc_value <= 3000) {
//		//Right
//		return 4;
//	}
//
//	return 0;
//}
//
//void change_timer() {
//
//	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) && one_flag == 1 ) one_flag = 0;
//
//	if (change_flag == 0) {
//		AmPm = sTime.TimeFormat;
//
//		hours_first = (sTime.Hours & 0xf0) >> 4;
//		hours_second = sTime.Hours & 0xf;
//
//		min_first = (sTime.Minutes & 0xf0) >> 4;
//		min_second = sTime.Minutes & 0xf;
//
//		sec_first = (sTime.Seconds & 0xf0) >> 4;
//		sec_second = sTime.Seconds & 0xf;
//		change_flag = 1;
//	}
//
//	button = adc_polling();
//
//	switch (button) {
//	case 1:
//		right_left--;
//		if (right_left <= 0)right_left = 0;
//		break;
//	case 4:
//		right_left++;
//		break;
//	default:
//		break;
//	}
//
//
//
//	if (alarm_flag == 1) {
//		switch (right_left % 7) {
//		case 0:
//			AM_PM_change();
//			alarm_AmPm = AmPm;
//			break;
//		case 1:
//			Hours1_change();
//			alarm_hours_first = hours_first;
//			break;
//		case 2:
//			Hours2_change();
//			alarm_hours_second = hours_second;
//			break;
//		case 3:
//			Min1_change();
//			alarm_min_first = min_first;
//			break;
//		case 4:
//			Min2_change();
//			alarm_min_second = min_second;
//			break;
//		case 5:
//			Sec1_change();
//			alarm_sec_first = sec_first;
//			break;
//		case 6:
//			Sec2_change();
//			alarm_sec_second = sec_second;
//			break;
//		default:
//			break;
//		}
//	}
//	else {
//		switch (right_left % 7) {
//		case 0:
//			AM_PM_change();
//			set_time(AmPm, hours_first << 4 | hours_second,min_first << 4 | min_second, sec_first << 4 | sec_second);
//			break;
//		case 1:
//			Hours1_change();
//			set_time(AmPm, hours_first << 4 | hours_second,min_first << 4 | min_second, sec_first << 4 | sec_second);
//			break;
//		case 2:
//			Hours2_change();
//			set_time(AmPm, hours_first << 4 | hours_second,min_first << 4 | min_second, sec_first << 4 | sec_second);
//			break;
//		case 3:
//			Min1_change();
//			set_time(AmPm, hours_first << 4 | hours_second,min_first << 4 | min_second, sec_first << 4 | sec_second);
//			break;
//		case 4:
//			Min2_change();
//			set_time(AmPm, hours_first << 4 | hours_second,min_first << 4 | min_second, sec_first << 4 | sec_second);
//			break;
//		case 5:
//			Sec1_change();
//			set_time(AmPm, hours_first << 4 | hours_second,min_first << 4 | min_second, sec_first << 4 | sec_second);
//			break;
//		case 6:
//			Sec2_change();
//			set_time(AmPm, hours_first << 4 | hours_second,min_first << 4 | min_second, sec_first << 4 | sec_second);
//			break;
//		default:
//			break;
//		}
//	}
//
//}
//
//
//
//
//
//void AM_PM_change() {
//
//	if(alarm_flag  == 1) {
//		sprintf(temp, "%s %d%d:%d%d:%d%d   AL ", ampm[AmPm], hours_first, hours_second,min_first, min_second, sec_first, sec_second);
//		clock_display();
//	}
//
//	sprintf(temp, "   %d%d:%d%d:%d%d   ", hours_first, hours_second, min_first,min_second, sec_first, sec_second);
//	clock_display();
//	sprintf(temp, "%s %d%d:%d%d:%d%d   ", ampm[AmPm], hours_first, hours_second,min_first, min_second, sec_first, sec_second);
//	clock_display();
//	if (adc_polling() == 2 || adc_polling() == 3) {
//		AmPm ^= 1;
//	}
//
//
//
//}
//
//void Hours1_change() {
//
//	sprintf(temp, "%s  %d:%d%d:%d%d   ", ampm[AmPm], hours_second, min_first,min_second, sec_first, sec_second);
//	clock_display();
//	sprintf(temp, "%s %d%d:%d%d:%d%d   ", ampm[AmPm], hours_first, hours_second,min_first, min_second, sec_first, sec_second);
//	clock_display();
//
//	if (adc_polling() == 2 || adc_polling() == 3) {
//		hours_first ^= 1;
//	}
//
//}
//
//void Hours2_change() {
//
//	sprintf(temp, "%s %d :%d%d:%d%d   ", ampm[AmPm], hours_first, min_first,min_second, sec_first, sec_second);
//	clock_display();
//	sprintf(temp, "%s %d%d:%d%d:%d%d   ", ampm[AmPm], hours_first, hours_second,min_first, min_second, sec_first, sec_second);
//	clock_display();
//
//	if (adc_polling() == 2) {
//		hours_second++;
//		if (hours_first == 1 && hours_second > 2) {
//			hours_first = 0;
//			hours_second = 0;
//		} else if (hours_second > 9) {
//			hours_second = 0;
//		}
//	} else if (adc_polling() == 3) {
//		hours_second--;
//		if (hours_first == 1 && hours_second < 0) {
//			hours_second = 9;
//		} else if (hours_first == 0 && hours_second < 0) {
//			hours_first = 1;
//			hours_second = 2;
//		}
//	}
//
//
//}
//
//void Min1_change() {
//
//	sprintf(temp, "%s %d%d: %d:%d%d   ", ampm[AmPm], hours_first, hours_second,min_second, sec_first, sec_second);
//	clock_display();
//	sprintf(temp, "%s %d%d:%d%d:%d%d   ", ampm[AmPm], hours_first, hours_second,min_first, min_second, sec_first, sec_second);
//	clock_display();
//
//	if (adc_polling() == 2) {
//		min_first++;
//		if (min_first > 5)
//			min_first = 0;
//	} else if (adc_polling() == 3) {
//		min_first--;
//		if (min_first < 0)
//			min_first = 5;
//	}
//
//
//}
//
//void Min2_change() {
//
//	sprintf(temp, "%s %d%d:%d :%d%d   ", ampm[AmPm], hours_first, hours_second,min_first, sec_first, sec_second);
//	clock_display();
//	sprintf(temp, "%s %d%d:%d%d:%d%d   ", ampm[AmPm], hours_first, hours_second,min_first, min_second, sec_first, sec_second);
//	clock_display();
//
//	if (adc_polling() == 2) {
//		min_second++;
//		if (min_second > 9)
//			min_second = 0;
//	} else if (adc_polling() == 3) {
//		min_second--;
//		if (min_second < 0)
//			min_second = 9;
//	}
//
//
//}
//
//void Sec1_change() {
//
//	sprintf(temp, "%s %d%d:%d%d: %d   ", ampm[AmPm], hours_first, hours_second,min_first, min_second, sec_second);
//	clock_display();
//	sprintf(temp, "%s %d%d:%d%d:%d%d   ", ampm[AmPm], hours_first, hours_second,min_first, min_second, sec_first, sec_second);
//	clock_display();
//
//	if (adc_polling() == 2) {
//		sec_first++;
//		if (sec_first > 5)
//			sec_first = 0;
//	} else if (adc_polling() == 3) {
//		sec_first--;
//		if (sec_first < 0)
//			sec_first = 5;
//	}
//
//}
//
//void Sec2_change() {
//
//	sprintf(temp, "%s %d%d:%d%d:%d    ", ampm[AmPm], hours_first, hours_second,min_first, min_second, sec_first);
//	clock_display();
//	sprintf(temp, "%s %d%d:%d%d:%d%d   ", ampm[AmPm], hours_first, hours_second,min_first, min_second, sec_first, sec_second);
//	clock_display();
//
//	if (adc_polling() == 2) {
//		sec_second++;
//		if (sec_second > 9)
//			sec_second = 0;
//	} else if (adc_polling() == 3) {
//		sec_second--;
//		if (sec_second < 0)
//			sec_second = 9;
//	}
//}
//
//void set_change_time(){
//
//}

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
	while (1) {
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
