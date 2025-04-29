/*
 * motor_control.c
 *
 *  Created on: Apr 14, 2025
 *      Author: lucag
 */

#include "motor_control.h"

void encoder_update(Encoder *enc, TIM_HandleTypeDef *htim, uint32_t arr_value) {
	uint32_t current_count = __HAL_TIM_GET_COUNTER(htim);
	int32_t diff;

	// evaluate increment of TIM3 counter from previous count
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
		if (current_count <= enc->prev_count)
			diff = current_count - enc->prev_count;
		else
			diff = -((arr_value + 1) - current_count) - enc->prev_count;
	} else {
		if (current_count >= enc->prev_count)
			diff = current_count - enc->prev_count;
		else
			diff = ((arr_value + 1) - enc->prev_count) + current_count;
	}
	enc->prev_count = current_count;

	/* Convert to RPM (Assuming 3840 pulses per revolution)
	 Explanation: diff/3840 gives the fraction of a revolution done during Ts
	 /Ts converts revolution in Ts seconds into revolution in 1 second
	 *60 coverts into revolution per minute (RPM)
	 NB: Everything in kept as float to avoid casting errors
	 */
	enc->speed_rpm = ((float) diff / 3840.0f) * (60.0f / TS);
}

float pi_control(PI_Controller *controller, float measured) {
	float error = controller->ref - measured;
	controller->integral += controller->ki * TS * error;
	float P = controller->kp * error;
	float I = controller->integral;
	return P + I;
}

float pi_control_antiwindup(PI_Controller *controller, float measured) {
	float error = controller->ref - measured;
	float P = controller->kp * error;

	// Tentative integral update
	float new_integral = controller->integral + controller->ki * TS * error;

	float output = P + new_integral;

	// Saturate the output if it's too high or low
	if (output > MAX_CONTROL_OUTPUT) {
		output = MAX_CONTROL_OUTPUT;
		// Only integrate if error would reduce saturation
		if (error < 0)
			controller->integral = new_integral;
	} else if (output < -MAX_CONTROL_OUTPUT) {
		output = -MAX_CONTROL_OUTPUT;
		if (error > 0)
			controller->integral = new_integral;
	} else {
		// No saturation — safe to update integral
		controller->integral = new_integral;
	}

	return output;

}


void set_motor_pwm(int32_t duty, TIM_HandleTypeDef *htim_pwm, uint32_t ch1, uint32_t ch2) {
	// constrain duty cycle
	duty = (duty > MAX_DUTY_BOT) ? MAX_DUTY_BOT : duty;
	duty = (duty < -MAX_DUTY_BOT) ? -MAX_DUTY_BOT : duty;

	printf("%d\n", duty);
	if (duty >= 0) {
		// Forward and coast
		// __HAL_TIM_SET_COMPARE(htim_pwm, ch1, (uint32_t )duty);
		// __HAL_TIM_SET_COMPARE(htim_pwm, ch2, 0);

		// Forward and brake
		__HAL_TIM_SET_COMPARE(htim_pwm, ch1, (uint32_t)TIM8_ARR_VALUE);
		__HAL_TIM_SET_COMPARE(htim_pwm, ch2, (uint32_t)(TIM8_ARR_VALUE - duty));
	} else {
		__HAL_TIM_SET_COMPARE(htim_pwm, ch1, 0);
		__HAL_TIM_SET_COMPARE(htim_pwm, ch2, (uint32_t )(-duty));
	}
}
