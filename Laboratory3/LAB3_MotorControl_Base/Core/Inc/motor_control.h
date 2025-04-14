/*
 * motor_control.h
 *
 *  Created on: Apr 14, 2025
 *      Author: lucag
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "main.h"

#define TS 0.01f
#define VBATT 12.0f
#define TIM8_ARR_VALUE 399  // Max TIM8 value. Used for PWM
//#define TIM3_ARR_VALUE 3840  // Assuming 4X encoder mode
#define V2DUTY ((float)(TIM8_ARR_VALUE + 1) / VBATT)  // Linear relation between duty cycle and provided voltage to motor
#define DUTY2V ((float)VBATT / (TIM8_ARR_VALUE + 1))
#define RPM2RADS ((2.0f * M_PI) / 60.0f)
#define MAX_CONTROL_OUTPUT  VBATT

// Struct to store all PID related things
typedef struct {
    float kp;
    float ki;
    float integral;
    float ref;
} PI_Controller;

// Struct to store all encoder related things
typedef struct {
    int32_t prev_count;
    float speed_rpm;
} Encoder;


void encoder_update(Encoder *enc, TIM_HandleTypeDef *htim, uint32_t arr_value);


float pi_control(PI_Controller *controller, float measured);


void set_motor_pwm(int32_t duty, TIM_HandleTypeDef *htim_pwm, uint32_t ch1, uint32_t ch2);


#endif /* MOTOR_CONTROL_H_ */
