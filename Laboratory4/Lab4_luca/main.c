#define TS	0.01
#define VBATT	12.0/2.0
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
#define BASE_SPEED_RPM 20.0f               // Base speed for straight sections
#define MAX_TURN_ADJUSTMENT 10.0f          // Maximum speed adjustment for turns
#define H_DISTANCE 0.085f                  // Distance from sensor to vehicle center
#define R 0.034f  // Wheel radius in meters 
#define D 0.165f
// Line following control variables
float line_kp = 2.0f;  // Proportional gain for line following (to be tuned)

const char keypad_map[4][4] = {
		{'1', '2', '3', 'A'},
		{'4', '5', '6', 'B'},
		{'7', '8', '9', 'C'},
		{'*', '0', '#', 'D'}
};

/* USER CODE BEGIN PV */
struct ertc_dlog logger;
float kp = 0.26072; // Proportional gain (to be tuned)
float ki = 0.86416; // Integral gain (to be tuned)
float kw = 10;  // Anti Wind up gain
float ref_w1 = 30.0; // Reference speed for motor 1 (rpm)
float ref_w2 = 30.0; // Reference speed for motor 2 (rpm)
float integral_w1 = 0.0, integral_w2 = 0.0; // Integral terms
float prev_count_w1 = 0.0, prev_count_w2 = 0.0; // Previous encoder counts
float sat_err1 = 0.0, sat_err2 = 0.0;
HAL_StatusTypeDef status;
uint8_t linesensor_data = 0;


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
	float w1;
	float w2;
	float u1;
	float u2;
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

/*
 * TODOs:
 * - undertand if we need strictly zero error or we can work with -1 as it is right now
 * - check for outliers: line sensor reading is not very precise and we need to take into account false positive
 */
float calc_line_error(uint8_t sensors) {
	const int N = 8;
	const float P = 0.008f; // distanza tra i sensori
	float eSL = 0.0f;
	float active_sum = 0.0f;

	for (int n = 0; n < N; n++) {
		uint8_t b_n = (sensors & (1 << n));
		float w_n = ((N - 1) / 2.0f - n) * P;
		eSL += b_n * w_n;
		active_sum += b_n;
	}

	if (active_sum > 0.0f) {
		eSL /= active_sum;
	} else {
		eSL = 0.0f; // Nessuna linea rilevata
	}

	return eSL;
}

// unserstand if we need something like this
void emergency_stop(){
	ref_w1 = ref_w2 = 0;
}

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		static int kLed = 0;

		// 1. Read encoder counts with overflow/underflow handling
		int32_t TIM3_CurrentCount = __HAL_TIM_GET_COUNTER(&htim3);
		int32_t TIM4_CurrentCount = __HAL_TIM_GET_COUNTER(&htim4);
		int32_t TIM3_DiffCount, TIM4_DiffCount;

		/* Speed and line control routine */
		if(htim->Instance == TIM6)
		{
			// Indicate that the program is running
			if(++kLed >= 10)
			{
				kLed = 0;
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			}

			// 1. Read line sensor data
			status =  HAL_I2C_Mem_Read(
					&hi2c1,
					SX1509_I2C_ADDR1 << 1,
					REG_DATA_B,
					1,
					&linesensor_data,
					1,
					I2C_TIMEOUT);

			// 2. Calculate line position error
			float eSL = calc_line_error(linesensor_data);
			int eSL_int = (int) (eSL * 1000);
			//printf("Reference speed = %d.%02d\n RPM", eSL_int / 100, eSL_int % 100);
			printf("Line error = %d\n", eSL_int);

			// 3. Simple line following control (alternative to yaw controller)

			// Get abs.value of the error
			float eSL = (eSL >= 0) ? eSL : -eSL;
			if (eSL > LINE_CENTER_ERROR_THRESHOLD) {
				// Line is to the left - turn left
				ref_w1 = BASE_SPEED_RPM - MAX_TURN_ADJUSTMENT * eSL;
				ref_w2 = BASE_SPEED_RPM + MAX_TURN_ADJUSTMENT * eSL;
			} 
			else if (eSL < -LINE_CENTER_ERROR_THRESHOLD) {
				// Line is to the right - turn right
				ref_w1 = BASE_SPEED_RPM + MAX_TURN_ADJUSTMENT * eSL;
				ref_w2 = BASE_SPEED_RPM - MAX_TURN_ADJUSTMENT * eSL;
			} 
			else {
				// Line is centered - go straight
				ref_w1 = ref_w2 = BASE_SPEED_RPM;
			}

			// 3. Yaw controller
			// Calculate yaw error approximation
			float psi_err = eSL / H_DISTANCE;
        
			// Calculate desired yaw rate using P control
			float psi_dot = line_kp * psi_err;
			
			// Set base speed and calculate wheel speeds
			float V = BASE_SPEED_RPM * RPM2RADS * R;  // Convert to m/s
			float V_r = V + psi_dot * D / 2.0f;
			float V_l = V - psi_dot * D / 2.0f;
			
			// Convert back to RPM for motor control
			ref_w1 = V_l / (R * RPM2RADS);
			ref_w2 = V_r / (R * RPM2RADS);

			// Handle under/overflow
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

			// 4. Calculate angular speed (rpm)
			float w1 = (TIM3_DiffCount / PPR_WHEEL) * 60.0f / TS;
			float w2 = (TIM4_DiffCount / PPR_WHEEL) * 60.0f / TS;

			// 5. Calculate speed error
			float error_w1 = ref_w1 - w1;
			float error_w2 = ref_w2 - w2;
			//		printf("%ld, %ld\n", (int32_t)error_w1, (int32_t)error_w2);

			// 6. Calculate proportional term
			float p_term_w1 = kp * error_w1;
			float p_term_w2 = kp * error_w2;

			// 7. Calculate integral term
			integral_w1 += (ki * error_w1 - kw * sat_err1)* TS;
			integral_w2 += (ki * error_w2 - kw * sat_err2) * TS;

			// 8. Calculate control signal (u)
			float u1 = p_term_w1 + integral_w1;
			float u2 = p_term_w2 + integral_w2;

			// 9. Saturate control signal to avoid exceeding VBATT
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
			data.w1 = w1;
			data.w2 = w2;
			data.u1 = u1;
			data.u2 = u2;
			ertc_dlog_send(&logger, &data, sizeof(data));
		}
	}
	/* USER CODE END 0 */

	int main(void)
	{

		uint8_t data;
		logger.uart_handle = huart3; // for serial
		//logger.uart_handle = huart2; // for wifi

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

	