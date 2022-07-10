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

#define DOUBLE_CLICK_MIN 20
#define DOUBLE_CLICK_MAX 120





//flash 메모리 Bank1,Bank2


/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0     ((uint32_t*)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t*)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t*)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t*)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t*)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t*)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t*)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t*)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t*)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t*)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t*)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t*)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_12     ((uint32_t*)0x08100000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_13     ((uint32_t*)0x08104000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_14     ((uint32_t*)0x08108000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_15     ((uint32_t*)0x0810C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_16     ((uint32_t*)0x08110000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_17     ((uint32_t*)0x08120000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_18     ((uint32_t*)0x08140000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_19     ((uint32_t*)0x08160000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_20     ((uint32_t*)0x08180000) /* Base @ of Sector 8, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_21     ((uint32_t*)0x081A0000) /* Base @ of Sector 9, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_22     ((uint32_t*)0x081C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_23     ((uint32_t*)0x081E0000) /* Base @ of Sector 11, 128 Kbytes */

//23->22->21.... 사용

#define MAGIC_NUM 0xdeadbeef
#define nv_items ((NVitemTypeDef*)ADDR_FLASH_SECTOR_23)




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
uint8_t mode_flag = 0;
uint8_t setting_flag = 0;
uint8_t Alarm_flag = 0;
uint8_t hfsec_flag = 0;

uint32_t timer = 0;
uint32_t timer_hfsec = 0;
uint32_t timer_flag = 0;

uint32_t value;

typedef enum {
	UP = 1, DOWN, RIGHT, LEFT, BASIC
} Clock_button;

typedef enum {
	NORMAL = 0, TIME_SETTING, ALARM_TIME_SETTING, MUSIC_SELECT
} Clock_mode;

typedef enum {
	Three_Bears = 0, Spring_Water
} Clock_music;

typedef struct {
	Clock_mode mode;
	Clock_button button;
	Clock_music music;
} Clock_state;

typedef struct {
	int16_t Hour1;
	int16_t Hour2;
	int16_t Min1;
	int16_t Min2;
	int16_t Sec1;
	int16_t Sec2;
	int16_t Ampm;

} Clock_time;

typedef struct {
	int32_t time;
	GPIO_PinState level;
} ClickInfoDef;

typedef struct{
	uint32_t magic_num;
	Clock_time setting_time;
	Clock_time alarm_time;
	int8_t alarm_music_num;
}NVitemTypeDef;

ClickInfoDef click[3];

Clock_state now_state;

enum notes {
	C = 956, D = 852, E = 758, F = 716, G = 638
};

uint16_t music1_notes[]={
		F,G,G,E,E,D,D,C,C,C,D,E,F,G,G,E,E,D,D,C,C,F,G,D,C,C,C,C,G,D,D,D,G,D,G,G,F,E,C,F,E,C,F,E,D,D,F,G,G,E,E
		,D,D,C,C,C,D,E,F,G,G,E,E,D,D,C,D,D,C,G
};

uint8_t music1_notes_length = sizeof(music1_notes)/sizeof(uint16_t);

NVitemTypeDef default_nvitem = {
		MAGIC_NUM,
		{0,0,0},
		{0,0,0},
		0
};


//uint32_t music2_notes[]={
//
//};

/*
 * 파솔솔미미 레레 도도 도레 미 파솔 솔 미미 레레 도 도 파 솔 레 도 라도도 도라 솔레레 레솔 솔 파
 * 미도 파 미도 파 미 레 레 파솔솔미미 레레 도도 도레 미 파솔 솔 미미 레레 도 레 레 도 솔
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void ADC_button();
void LCD_Cursor(int rownum, int column);
void blink(uint8_t location,Clock_time* buf_time);
void set_time(Clock_time* now_time);
uint16_t Up_Down_value_limit(int16_t value);
void init_time();
void clock(Clock_time* buf_time,Clock_time* now_time,Clock_time* alarm_time);
void Normal_mode(Clock_time* alarm_time);
void Time_or_Alarm_change_mode(Clock_time* buf_time,Clock_time* now_time,Clock_time* alarm_time);
void Music_select_mode();
void Alarm(Clock_time* alarm_time);

void Alarm_Spring_Water_paly();
void Alarm_Three_Bears_play();

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
	MX_ADC1_Init();
	MX_RTC_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	init();
	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Init(&htim3);
	init_time();

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);



	Clock_time buf_time;
	Clock_time now_time;
	Clock_time alarm_time;

	memset(&buf_time,0,sizeof(Clock_time));
	memset(&now_time,0,sizeof(Clock_time));
	memset(&alarm_time,0,sizeof(Clock_time));

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

		clock(&buf_time,&now_time,&alarm_time);

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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	GPIO_PinState pin;
	int i;

	if (GPIO_Pin == GPIO_PIN_13) {
		current_time = HAL_GetTick();
		time_interval = current_time - last_time;
		last_time = current_time;

		pin = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

		if (time_interval <= 2) // noise
				{
		} else {

			click[2].time = click[1].time;
			click[2].level = click[1].level;

			click[1].time = click[0].time;
			click[1].level = click[0].level;

			click[0].time = time_interval;
			click[0].level = pin;

			if (click[2].level == GPIO_PIN_RESET
					&& click[1].level == GPIO_PIN_SET
					&& click[0].level == GPIO_PIN_RESET) {
				for (i = 0; i < 3; i++) {
					if (click[i].time >= DOUBLE_CLICK_MIN
							&& click[i].time <= DOUBLE_CLICK_MAX) {
						continue;
					} else
						break;
				}
				if (i == 3) {
					now_state.mode = MUSIC_SELECT;
				}
			}

			if (click[0].level== GPIO_PIN_RESET&& click[0].time >=LONG_CLICK_MIN) // long click
					{
				now_state.mode = ALARM_TIME_SETTING;
			} else if (click[0].level== GPIO_PIN_RESET&& click[0].time < LONG_CLICK_MIN && click[0].time > DOUBLE_CLICK_MAX) {
				if (now_state.mode == TIME_SETTING
						|| now_state.mode == ALARM_TIME_SETTING
						|| now_state.mode == MUSIC_SELECT) mode_flag = 1;
				else mode_flag = 0;
				if (mode_flag == 0) now_state.mode = TIME_SETTING;
				else if (mode_flag == 1) now_state.mode = NORMAL;
			}

		}
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM2) {
		ADC_button();
		timer++;
		if(hfsec_flag == 1){
			timer_flag++;
			if(timer_flag % 5 == 0){
				hfsec_flag = 0;
			}
		}
	}
}

//ADC값 변화에 따른 버튼
void ADC_button() {
	HAL_ADC_Start(&hadc1);
	ADC_Value = HAL_ADC_GetValue(&hadc1);

	if (ADC_Value <= 100) now_state.button = UP;
	else if (ADC_Value >= 800 && ADC_Value <= 900) now_state.button = DOWN;
	else if (ADC_Value >= 1900 && ADC_Value <= 2000) now_state.button = LEFT;
	else if (ADC_Value >= 2900 && ADC_Value <= 3100) now_state.button = RIGHT;
	else now_state.button = BASIC;
}


//LCD_Cursor 위치 변경(16x2)
void LCD_Cursor(int row, int column) {
	if (row == 1) row = 0x40;
	LCD_SendCommand(LCD_ADDR, 0x80 | (row | column));
}

//am 0시 0분 0초 초기화
void init_time() {
	sTime.TimeFormat = 0;
	sTime.Hours = 0;
	sTime.Minutes = 0;
	sTime.Seconds = 0;

	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
}


//매개변수의 시간으로 setTime
void set_time(Clock_time* now_time) {

	sTime.TimeFormat = now_time->Ampm;
	sTime.Hours = now_time->Hour1 << 4 | now_time->Hour2;
	sTime.Minutes = now_time->Min1 << 4 | now_time->Min2;
	sTime.Seconds = now_time->Sec1 << 4 | now_time->Sec2;

	if (sTime.Hours > 0x12) sTime.Hours = 0x10;

	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
}


//시간,알람시간 변경시 깜빡거림을 표현
void blink(uint8_t location,Clock_time* buf_time) {

	uint16_t AMpm;
	uint16_t hour;
	uint16_t min;
	uint16_t sec;

	AMpm = buf_time->Ampm;
	hour = buf_time->Hour1 << 4 | buf_time->Hour2;
	min = buf_time->Min1 << 4 | buf_time->Min2;
	sec = buf_time->Sec1 << 4 | buf_time->Sec2;

	sprintf(temp, "%s %02x:%02x:%02x", ampm[AMpm], hour, min, sec);
	LCD_Cursor(1, 0);
	LCD_SendString(LCD_ADDR, temp);

	if (timer % 3 == 0) {
		if (location == 0) {
			LCD_Cursor(1, location);
			LCD_SendString(LCD_ADDR, "  ");
		} else {
			LCD_Cursor(1, location);
			LCD_SendData(LCD_ADDR, ' ');
		}
	}

}

//시간, 알람시간 변경모드 에서 버튼에 따른 값 변화 그리고 시계처럼 제한 두는 함수
uint16_t Up_Down_value_limit(int16_t value) {
	if (now_state.button == UP)
		value++;
	else if (now_state.button == DOWN)
		value--;
	if (value > 9)
		value = 0;
	if (value < 0)
		value = 9;

	return value;
}

//모드에 맞게 동작하는 시계
void clock(Clock_time* buf_time,Clock_time* now_time,Clock_time* alarm_time) {

	if (now_state.mode == NORMAL) Normal_mode(alarm_time);
	else if (now_state.mode == TIME_SETTING|| now_state.mode == ALARM_TIME_SETTING) Time_or_Alarm_change_mode(buf_time,now_time,alarm_time);
	else if (now_state.mode == MUSIC_SELECT) Music_select_mode();


}


//기본적으로 시간이 흐르는 모드
void Normal_mode(Clock_time* alarm_time) {

	LCD_Cursor(0, 0);
	LCD_SendString(LCD_ADDR, "Jin Ho Clock ");
	sprintf(temp, "%s %02x:%02x:%02x", sTime.TimeFormat[ampm], sTime.Hours, sTime.Minutes, sTime.Seconds);
	LCD_Cursor(1, 0);
	LCD_SendString(LCD_ADDR, temp);
	if (Alarm_flag == 1) {
		LCD_Cursor(1, 11);
		LCD_SendString(LCD_ADDR, "  AL");
	} else {
		LCD_Cursor(1, 11);
		LCD_SendString(LCD_ADDR, "     ");
	}

	Alarm(alarm_time);
}




//시간 변경 모드(버튼 한번) 또는 알람 시간 변경 모드(버튼 길게 3초이상)
void Time_or_Alarm_change_mode(Clock_time* buf_time,Clock_time* now_time,Clock_time* alarm_time) {

	if (setting_flag == 0) {
		buf_time->Ampm = sTime.TimeFormat;
		buf_time->Hour1 = (sTime.Hours & 0xf0) >> 4;
		buf_time->Hour2 = sTime.Hours & 0x0f;
		buf_time->Min1 = (sTime.Minutes & 0xf0) >> 4;
		buf_time->Min2 = sTime.Minutes & 0x0f;
		buf_time->Sec1 = (sTime.Seconds & 0xf0) >> 4;
		buf_time->Sec2 = sTime.Seconds & 0x0f;

		setting_flag = 1;
	}

	switch (now_state.button) {
	case RIGHT:
		cursor_change++;
		break;
	case LEFT:
		cursor_change--;
		if (cursor_change < 0)
			cursor_change = 0;
		break;
	default:
		break;
	}

	switch (cursor_change % 7) {
	case 0:
		blink(0,buf_time);
		if (now_state.button == UP || now_state.button == DOWN)
			buf_time->Ampm ^= 1;
		break;
	case 1:
		blink(3,buf_time);
		if (now_state.button == UP || now_state.button == DOWN)
			buf_time->Hour1 ^= 1;
		break;
	case 2:
		blink(4,buf_time);
		buf_time->Hour2 = Up_Down_value_limit(buf_time->Hour2);
		break;
	case 3:
		blink(6,buf_time);
		buf_time->Min1 = Up_Down_value_limit(buf_time->Min1);
		if (buf_time->Min1 > 5)
			buf_time->Min1 = 0;
		break;
	case 4:
		blink(7,buf_time);
		buf_time->Min2 = Up_Down_value_limit(buf_time->Min2);
		break;
	case 5:
		blink(9,buf_time);
		buf_time->Sec1 = Up_Down_value_limit(buf_time->Sec1);
		if (buf_time->Sec1 > 5)
			buf_time->Sec1 = 0;
		break;
	case 6:
		blink(10,buf_time);
		buf_time->Sec2 = Up_Down_value_limit(buf_time->Sec2);
		break;
	default:
		break;
	}

	if ((buf_time->Hour1 << 4 | buf_time->Hour2) > 0x12) buf_time->Hour2 = 0;

	if (now_state.mode == TIME_SETTING) {
		LCD_Cursor(0, 0);
		LCD_SendString(LCD_ADDR, "Time Setting");
		LCD_Cursor(1, 11);
		LCD_SendString(LCD_ADDR, "     ");

		now_time = buf_time;
		now_time->Ampm = buf_time->Ampm;
		now_time->Hour1 = buf_time -> Hour1;
		now_time->Hour2 = buf_time -> Hour2;
		now_time->Min1 = buf_time -> Min1;
		now_time->Min2 = buf_time -> Min2;
		now_time->Sec1 = buf_time -> Sec1;
		now_time->Sec2 = buf_time -> Sec2;

		set_time(now_time);
	}

	else if (now_state.mode == ALARM_TIME_SETTING) {
		LCD_Cursor(0, 0);
		LCD_SendString(LCD_ADDR, "Alarm Setting");
		LCD_Cursor(1, 11);
		LCD_SendString(LCD_ADDR, "  AL  ");


		alarm_time->Ampm = buf_time->Ampm;
		alarm_time->Hour1 = buf_time -> Hour1;
		alarm_time->Hour2 = buf_time -> Hour2;
		alarm_time->Min1 = buf_time -> Min1;
		alarm_time->Min2 = buf_time -> Min2;
		alarm_time->Sec1 = buf_time -> Sec1;
		alarm_time->Sec2 = buf_time -> Sec2;


		Alarm_flag ^= 1;
	}

}

//음악 선택하는 모드(더블클릭)
void Music_select_mode() {

	if (now_state.button == UP || now_state.button == DOWN) now_state.music ^= Spring_Water;

	LCD_Cursor(0, 0);
	LCD_SendString(LCD_ADDR, "Music select");
	LCD_Cursor(1, 0);
	if (now_state.music == Three_Bears) sprintf(temp, "%s", "[Three Bears] ");
	else if (now_state.music == Spring_Water) sprintf(temp, "%s", "[Spring_Water] ");
	LCD_SendString(LCD_ADDR, temp);

}

//현재시간과 알람시간이 같을때 울리기
void Alarm(Clock_time* alarm_time) {

	uint16_t a_ampm;
	uint16_t a_hour;
	uint16_t a_min;
	uint16_t a_sec;

	a_ampm = alarm_time->Ampm;
	a_hour = alarm_time->Hour1 << 4 | alarm_time->Hour2;
	a_min = alarm_time->Min1 << 4 | alarm_time->Min2;
	a_sec = alarm_time->Sec1 << 4 | alarm_time->Sec2;


	if ((Alarm_flag == 1) && (sTime.TimeFormat == a_ampm)
			&& (sTime.Hours == a_hour) && (sTime.Minutes == a_min)
			&& (sTime.Seconds == a_sec) && (now_state.mode == NORMAL)) {

		if(now_state.music == Spring_Water) printf("\r\n Spring_Water_play \r\n");
		else printf("\r\n Three_Bears_play \r\n");

		Alarm_flag = 0;
	}
}


//timer 10씩 커질때마다 1초

//알람 노래 Spring_Water_paly
void Alarm_Spring_Water_paly(){


	uint16_t melody = (uint16_t) (1000000 / music[seq].freq);
	uint8_t num = 0;

	if (default_nvitem.alarm_music_num == 0) num = MEL_NUM;
	else if (default_nvitem.alarm_music_num == 1) num = MEL_NUM;
	else if (default_nvitem.alarm_music_num == 2) num = MEL_NUM + 1;

	if (stop == 1) {
		TIM4->ARR = 2000;
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		stop = 0;
	} else {
		if (seq == num) {
			seq = 0;
		} else {
			TIM3->ARR = melody;
			TIM3->CCR2 = melody / 2;
			TIM4->ARR = music[seq].delay * 2000;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			stop = 1;

			seq++;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
		}
	}


	for (int i = 0; i < music1_notes_length; i++) {
		TIM3->ARR = music1_notes[i];
		TIM3->CCR4 = TIM3->ARR / 2;
		HAL_Delay(500);
	}

	TIM3->CCR4 = 0;
	HAL_Delay(10);
	TIM3->CCR4 = TIM3->ARR / 2;



}
//알람 노래 Three_Bears_play
void Alarm_Three_Bears_play(){
	/*
	 for (int i = 0; i < bell_length; i++) {
	 TIM3->ARR = bell[i];
	 TIM3->CCR4 = TIM3->ARR / 2;
	 HAL_Delay(500);
	 TIM3->CCR4 = 0;
	 HAL_Delay(interval[i]);
	 TIM3->CCR4 = TIM3->ARR / 2;
	 }
	 */
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
