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
#include <math.h>
#include <stdbool.h>
#include "ertc-datalogger.h"
#include "SX1509_Registers.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define	HAL_TIMEOUT		1
#define TS	0.01

#define VBATT	12.0
#define V2DUTY	((float)(TIM8_ARR_VALUE+1)/VBATT)
#define DUTY2V	((float)VBATT/(TIM8_ARR_VALUE+1))

#define RPM2RADS	2*M_PI/60
#define PPR_WHEEL 3840.0f

#define TIM3_ARR_VALUE 3840
#define TIM4_ARR_VALUE 3840

#define HAL_TIMEOUT 100

#define I2C_TIMEOUT 200

#define SX1509_I2C_ADDR1 0x3E //	SX1509 Proxy Sensors I2C address
#define SX1509_I2C_ADDR2 0x3F //	SX1509 Keypad I2C address

#define REG_KEY_DATA_1 0x27  // col register
#define REG_KEY_DATA_2 0x28  // row register

#define LINE_CENTER_ERROR_THRESHOLD 0.002f  // 2mm tolerance
#define BASE_SPEED_RPM 100.0f               // Base speed for straight sections
#define MAX_TURN_ADJUSTMENT 50.0f          // Maximum speed adjustment for turns

// [CHG] used to have a precise set of possible error values (instead of the noisy readings we are getting rn)
const float dists[8] = {3.5, 2.5, 1.5, 0.5, -0.5, -1.5,-2.5, -3.5};

const char keypad_map[4][4] = {
		{'1', '2', '3', 'A'},
		{'4', '5', '6', 'B'},
		{'7', '8', '9', 'C'},
		{'*', '0', '#', 'D'}
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
struct ertc_dlog logger;
float kp = 0.26072; // Proportional gain (to be tuned)
//float kp = 10; // Proportional gain (to be tuned)
float ki = 0.86416; // Integral gain (to be tuned)
//float ki = 2; // Integral gain (to be tuned)
float kw = 10;  // Anti Wind up gain
float ref_w1 = 30.0; // Reference speed for motor 1 (rpm)
float ref_w2 = 30.0; // Reference speed for motor 2 (rpm)
float integral_w1 = 0.0, integral_w2 = 0.0; // Integral terms
float prev_count_w1 = 0.0, prev_count_w2 = 0.0; // Previous encoder counts
float sat_err1 = 0.0, sat_err2 = 0.0;
HAL_StatusTypeDef status;
uint8_t linesensor_data = 0;

// [CHG]
float line_kp = 2.0f;  // Proportional gain for line following (to be tuned)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}

struct datalog {
	float ref1;
	float w1;
	float ref2;
	float w2;
	float eSL;
} data;

char get_keypad_key(void) {
	uint8_t col = 0xFF, row = 0xFF;

	// Read column and row register from SX1509
	HAL_I2C_Mem_Read(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_KEY_DATA_1, 1, &col, 1,
			100);
	HAL_I2C_Mem_Read(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_KEY_DATA_2, 1, &row, 1,
			100);

	// Nothing pressed (all bits HIGH)
	if (col == 0xFF || row == 0xFF)
		return '\0';

	// Find the LOW bit (indicating which row and column were pressed)
	// Tried to do both row and column at the same time
	for (int r = 0; r < 4; r++) {
		if (!(row & (1 << r))) {
			for (int c = 0; c < 4; c++) {
				if (!(col & (1 << c))) {
					return keypad_map[3-r][c];
				}
			}
		}
	}

	return '\0';
}

// Set reference speed with keypad
char input_buffer[6] = { 0 };  // To store up to 5 digits
int input_index = 0;

void handle_keypad_input() {
	char key = get_keypad_key();
	// Print the pressed button - safety check
	if (key != '\0') {
		printf("Key pressed: %c\n", key);
	}

	if (key >= '0' && key <= '9') {
		if (input_index < sizeof(input_buffer) - 1) {
			input_buffer[input_index++] = key;
			input_buffer[input_index] = '\0';
		}
	} else if (key == '#') {
		if (input_index > 0) {
			float new_ref = atof(input_buffer);  // ASCII to Float
			// TODO check speed bound on reference
			ref_w1 = new_ref;
			ref_w2 = new_ref;

			// Workaround the impossibility to print float
			int sp_int = (int) (new_ref * 100);
			printf("Reference speed = %d.%02d\n RPM", sp_int / 100,
					sp_int % 100);
		}
		input_index = 0;
		input_buffer[0] = '\0';
	} else if (key == '*') {
		ref_w1 = 0;
		ref_w2 = 0;
		input_index = 0;
		input_buffer[0] = '\0';
		printf("Input set to 0\n");
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	handle_keypad_input();
}

// [CHG] implemented the weights array. Now the error values should be more consistent
float calc_line_error(uint8_t sensors) {
	const int N = 8;
	const float P = 0.008f; // distanza tra i sensori
	float eSL = 0.0f;
	float active_sum = 0.0f;

	for (int n = 0; n < N; n++) {
		uint8_t b_n = (sensors & (1 << n));
		float lambda_n = weights[7-n] * P;
		eSL += b_n * lambda_n;
		active_sum += b_n;
	}

	if (active_sum != 0.0f) {
		eSL /= active_sum;
	} else {
		eSL = 0.0f; // Nessuna linea rilevata
	}

	return eSL;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	static int kLed = 0;
	// 1. Read encoder counts with overflow/underflow handling
	int32_t TIM3_CurrentCount = __HAL_TIM_GET_COUNTER(&htim3);
	int32_t TIM4_CurrentCount = __HAL_TIM_GET_COUNTER(&htim4);
	int32_t TIM3_DiffCount, TIM4_DiffCount;

	/* Speed ctrl routine */
	if(htim->Instance == TIM6)
	{

		// Indicate that the program is running
		if(++kLed >= 10)
		{
			kLed = 0;
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}

		// get line sensor reading

		status =  HAL_I2C_Mem_Read(
				&hi2c1,
				SX1509_I2C_ADDR1 << 1,
				REG_DATA_B,
				1,
				&linesensor_data,
				1,
				I2C_TIMEOUT);

		// compute line error
		float eSL = calc_line_error(linesensor_data);
		int eSL_int = (int) (eSL * 1000);
		//printf("Reference speed = %d.%02d\n RPM", eSL_int / 100, eSL_int % 100);
		printf("Line error = %d\n", eSL_int);

	
		// [CHG] idea: consider different "zones" in the error values, like "very close to 0", "moderately close to 0",
		// "too far from 0" (values need to be tested) and apply different control strenght in each zone. if the error is small 
		// i can set a higher speed for example, if it's higher the speed must be slower.
		// (wrote in pseudocode)

		// Get abs.value of the error
		eSL = (eSL >= 0) ? eSL : -eSL;
		eSL *= 25;
		if (eSL < "very close to zero") {
			ref_w1 = "little multiplier" * BASE_SPEED_RPM - line_kp * eSL;
			ref_w2 = "little multiplier" * BASE_SPEED_RPM + line_kp * eSL;
		}
		else if (eSL < "moderately close to 0") {
			ref_w1 = "medium multiplier" * BASE_SPEED_RPM - line_kp * eSL;
			ref_w2 = "medium multiplier" * BASE_SPEED_RPM + line_kp * eSL;
		}
		else {
			// Line is centered - go straight
			ref_w1 = "high multiplier" * BASE_SPEED_RPM - line_kp * eSL;
			ref_w2 = "high multiplier" * BASE_SPEED_RPM + line_kp * eSL;
		}

		printf("ref_1: %d\n", (int)(ref_w1*1000));
		printf("ref_2: %d\n", (int)(ref_w2*1000));


		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
			if (TIM3_CurrentCount <= prev_count_w1) {
				TIM3_DiffCount = TIM3_CurrentCount - prev_count_w1;
			} else {
				TIM3_DiffCount = -((TIM3_ARR_VALUE + 1) - TIM3_CurrentCount) - prev_count_w1;
			}
		} else {
			if (TIM3_CurrentCount >= prev_count_w1) {
				TIM3_DiffCount = TIM3_CurrentCount - prev_count_w1;
			} else {
				TIM3_DiffCount = ((TIM3_ARR_VALUE + 1) - prev_count_w1) + TIM3_CurrentCount;
			}
		}
		prev_count_w1 = TIM3_CurrentCount;

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
			if (TIM4_CurrentCount <= prev_count_w2) {
				TIM4_DiffCount = TIM4_CurrentCount - prev_count_w2;
			} else {
				TIM4_DiffCount = -((TIM4_ARR_VALUE + 1) - TIM4_CurrentCount) - prev_count_w2;
			}
		} else {
			if (TIM4_CurrentCount >= prev_count_w2) {
				TIM4_DiffCount = TIM4_CurrentCount - prev_count_w2;
			} else {
				TIM4_DiffCount = ((TIM4_ARR_VALUE + 1) - prev_count_w2) + TIM4_CurrentCount;
			}
		}
		prev_count_w2 = TIM4_CurrentCount;

		// 2. Calculate angular speed (rpm)
		float w1 = (TIM3_DiffCount / PPR_WHEEL) * 60.0f / TS;
		float w2 = (TIM4_DiffCount / PPR_WHEEL) * 60.0f / TS;

		// 3. Calculate error
		float error_w1 = ref_w1 - w1;
		float error_w2 = ref_w2 - w2;
		//		printf("%ld, %ld\n", (int32_t)error_w1, (int32_t)error_w2);

		// [CHG]
		// 3.5 Change measurement unit for the error
		error_w1 = error_w1 * RPM2RADS;
		error_w2 = error_w2 * RPM2RADS;

		// 4. Calculate proportional term
		float p_term_w1 = kp * error_w1;
		float p_term_w2 = kp * error_w2;

		// 5. Calculate integral term
		integral_w1 += (ki * error_w1 - kw * sat_err1)* TS;
		integral_w2 += (ki * error_w2 - kw * sat_err2) * TS;

		// 6. Calculate control signal (u)
		float u1 = p_term_w1 + integral_w1;
		float u2 = p_term_w2 + integral_w2;

		// 7. Saturate control signal to avoid exceeding VBATT
		float u1_sat = fminf(fmaxf(u1, -VBATT), VBATT);
		float u2_sat = fminf(fmaxf(u2, -VBATT), VBATT);

		sat_err1 = u1 - u1_sat;
		sat_err2 = u2 - u2_sat;

		int32_t duty1 = u1_sat * V2DUTY;
		int32_t duty2 = u2_sat * V2DUTY;

		/* calculate duty properly */
		if ( duty1 >= 0) { // rotate forward
			/* alternate between forward and coast */
			__HAL_TIM_SET_COMPARE (& htim8 , TIM_CHANNEL_1 , ( uint32_t ) duty1 );
			__HAL_TIM_SET_COMPARE (& htim8 , TIM_CHANNEL_2 , 0) ;
			/* alternate between forward and brake , TIM8_ARR_VALUE is a define */
			// __HAL_TIM_SET_COMPARE (& htim8 , TIM_CHANNEL_1 , ( uint32_t ) TIM8_ARR_VALUE );
			// __HAL_TIM_SET_COMPARE (& htim8 , TIM_CHANNEL_2 , ( uint32_t )( TIM8_ARR_VALUE - duty1 ));

		} else { // rotate backward
			__HAL_TIM_SET_COMPARE (& htim8 , TIM_CHANNEL_1 , 0) ;
			__HAL_TIM_SET_COMPARE (& htim8 , TIM_CHANNEL_2 , ( uint32_t ) - duty1 );
		}

		if ( duty2 >= 0) { // rotate forward
			/* alternate between forward and coast */
			__HAL_TIM_SET_COMPARE (& htim8 , TIM_CHANNEL_3 , ( uint32_t ) duty2 );
			__HAL_TIM_SET_COMPARE (& htim8 , TIM_CHANNEL_4 , 0) ;
			/* alternate between forward and brake , TIM8_ARR_VALUE is a define */
			// __HAL_TIM_SET_COMPARE (& htim8 , TIM_CHANNEL_3 , ( uint32_t ) TIM8_ARR_VALUE );
			// __HAL_TIM_SET_COMPARE (& htim8 , TIM_CHANNEL_4 , ( uint32_t )( TIM8_ARR_VALUE - duty2 ));

		} else { // rotate backward
			__HAL_TIM_SET_COMPARE (& htim8 , TIM_CHANNEL_3 , 0) ;
			__HAL_TIM_SET_COMPARE (& htim8 , TIM_CHANNEL_4 , ( uint32_t ) - duty2 );
		}

		// Log data
		data.ref1 = ref_w1;
		data.w1 = w1;
		data.ref2 = ref_w2;
		data.w2 = w2;
		data.eSL = eSL_int;
		ertc_dlog_send(&logger, &data, sizeof(data));
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
	uint8_t data;

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
	MX_USART3_UART_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM8_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_TIM9_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */

	//logger.uart_handle = huart3; // for serial
	logger.uart_handle = huart2; // for wifi

	/* Reset LCD */
	HAL_GPIO_WritePin(GPIO_OUT_SPI_CS_LCD_GPIO_Port, GPIO_OUT_SPI_CS_LCD_Pin, GPIO_PIN_SET);

	/* Disable EXTI4_IRQ during SX1509 initialization */
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);

	ITM_SendChar('h');

	/* Software reset */
	data = 0x12;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	data = 0x34;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	HAL_Delay(100);

	/* Set KeyPad scanning engine */

	/* Set RegClock to 0x40 (enable internal oscillator; 2MHz freq) */
	data = 0x40;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_CLOCK, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Set Bank A RegDir to 0xF0 (IO[0:3] as out) */
	data = 0xF0;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DIR_A, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Set Bank B RegDir to 0x0F (IO[8:11] as in) */
	data = 0x0F;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DIR_B, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Set Bank A RegOpenDrain to 0x0F (IO[0:3] as open-drain outputs) */
	data = 0x0F;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_OPEN_DRAIN_A, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Set Bank B RegPullup to 0x0F (pull-ups enabled on inputs IO[8:11]) */
	data = 0x0F;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_PULL_UP_B, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Set Bank B RegDebounceEnable to 0x0F (enable debouncing on IO[8:11]) */
	data = 0x0F;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DEBOUNCE_ENABLE_B, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Set RegDebounceConfig to 0x05 (16ms debounce time) */
	data = 0x05;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_DEBOUNCE_CONFIG, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Set RegKeyConfig1 to 0x7D (8s auto-sleep; 32ms scan time per row) */
	data = 0x7D;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_KEY_CONFIG_1, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Set RegKeyConfig2 to 0x1B (4 rows; 4 columns) */
	data = 0x1B;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR2 << 1, REG_KEY_CONFIG_2, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Enable EXTI4_IRQ after SX1509 initialization */
	HAL_Delay(100);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	/* Disable EXTI2_IRQ during SX1509 initialization */
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

	/* Software reset */
	data = 0x12;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	data = 0x34;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_RESET, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	HAL_Delay(100);

	/* Set RegDirA to 0xFF (all IO of Bank A configured as inputs) */
	data = 0xFF; // 0 = out; 1 = in
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_DIR_A, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Set RegDirB to 0xFF (all IO of Bank B configured as inputs) */
	data = 0xFF; // 0 = out; 1 = in
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_DIR_B, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Set RegInterruptMaskA to 0x00 (all IO of Bank A will trigger an interrupt) */
	data = 0x00;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_INTERRUPT_MASK_A, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Set RegSenseHighA to 0xAA (IO[7:4] of Bank A will trigger an interrupt on falling edge) */
	data = 0xAA;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_SENSE_HIGH_A, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Set RegSenseLowA to 0xAA (IO[3:0] of Bank A will trigger an interrupt on falling edge) */
	data = 0xAA;
	status = HAL_I2C_Mem_Write(&hi2c1, SX1509_I2C_ADDR1 << 1, REG_SENSE_LOW_A, 1, &data, 1, I2C_TIMEOUT);
	if (status != HAL_OK)
		printf("I2C communication error (%X).\n", status);

	/* Enable EXTI2_IRQ after SX1509 initialization */
	HAL_Delay(100);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);


	printf("Ready\n");

	/* Reset LCD */
	HAL_GPIO_WritePin(GPIO_OUT_SPI_CS_LCD_GPIO_Port, GPIO_OUT_SPI_CS_LCD_Pin, GPIO_PIN_SET);

	HAL_Delay(1000);

	/* Start encoders timers */
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	/* Start servomotors PWM (avoid floating inputs to servomotors) */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	/* Start motor PWM */
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	/* Start speed ctrl ISR */
	HAL_TIM_Base_Start_IT(&htim6);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		ertc_dlog_update(&logger);


		printf("Line Sensor Data: ");
		for(int i=7;i>=0;i--){
			printf("%d", (linesensor_data >> i) & 0x01);
		}
		printf("\n");

		HAL_Delay(200);

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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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
	hi2c1.Init.Timing = 0x20303E5D;
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
	hi2c2.Init.Timing = 0x20303E5D;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

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
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0;
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
	if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_FORCED_ACTIVE;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

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
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = TIM3_ARR_VALUE;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 15;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 15;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = TIM4_ARR_VALUE;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 15;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 15;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */
	HAL_TIM_MspPostInit(&htim5);

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
	htim6.Init.Prescaler = TIM6_PSC_VALUE;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = TIM6_ARR_VALUE;
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
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = TIM8_PSC_VALUE;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = TIM8_ARR_VALUE;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void)
{

	/* USER CODE BEGIN TIM9_Init 0 */

	/* USER CODE END TIM9_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM9_Init 1 */

	/* USER CODE END TIM9_Init 1 */
	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 0;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 65535;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM9_Init 2 */

	/* USER CODE END TIM9_Init 2 */
	HAL_TIM_MspPostInit(&htim9);

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 9600;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void)
{

	/* USER CODE BEGIN UART5_Init 0 */

	/* USER CODE END UART5_Init 0 */

	/* USER CODE BEGIN UART5_Init 1 */

	/* USER CODE END UART5_Init 1 */
	huart5.Instance = UART5;
	huart5.Init.BaudRate = 115200;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart5) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART5_Init 2 */

	/* USER CODE END UART5_Init 2 */

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
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
	huart2.Init.BaudRate = 1000000;
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
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
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
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_OUT_SPI_CS_SDCARD_Pin|GPIO_OUT_SPI_CS_LCD_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pins : GPIO_OUT_SPI_CS_SDCARD_Pin GPIO_OUT_SPI_CS_LCD_Pin */
	GPIO_InitStruct.Pin = GPIO_OUT_SPI_CS_SDCARD_Pin|GPIO_OUT_SPI_CS_LCD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : GPIO_EXTI3_IMU_IRQ_Pin GPIO_EXTI8_USER_BUT1_IRQ_Pin GPIO_EXTI9_USER_BUT2_IRQ_Pin GPIO_EXTI10_BUMP1_IRQ_Pin
                           GPIO_EXTI11_BUMP2_IRQ_Pin GPIO_EXTI12_BUMP3_IRQ_Pin GPIO_EXTI13_BUMP4_IRQ_Pin */
	GPIO_InitStruct.Pin = GPIO_EXTI3_IMU_IRQ_Pin|GPIO_EXTI8_USER_BUT1_IRQ_Pin|GPIO_EXTI9_USER_BUT2_IRQ_Pin|GPIO_EXTI10_BUMP1_IRQ_Pin
			|GPIO_EXTI11_BUMP2_IRQ_Pin|GPIO_EXTI12_BUMP3_IRQ_Pin|GPIO_EXTI13_BUMP4_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : GPIO_EXTI4_KPAD_IRQ_Pin */
	GPIO_InitStruct.Pin = GPIO_EXTI4_KPAD_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIO_EXTI4_KPAD_IRQ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
	GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
	GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_TXD1_Pin */
	GPIO_InitStruct.Pin = RMII_TXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PG6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
	GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_VBUS_Pin */
	GPIO_InitStruct.Pin = USB_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
	GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
		static uint32_t kLed = 0;
		if(++kLed >= 1000)
		{
			kLed = 0;
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		}
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
