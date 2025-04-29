/*
 * keypad.h
 *
 *  Created on: Apr 14, 2025
 *      Author: lucag
 */

#ifndef KEYPAD_H_
#define KEYPAD_H_

#include "main.h"
#include "motor_control.h"
#include <stdio.h>
#include <string.h>
#include "SX1509_Registers.h"

#define SX1509_I2C_ADDR1 0x3E  // Line sensor
#define SX1509_I2C_ADDR2 0x3F  // Keypad
#define I2C_TIMEOUT 200

// Function to read a key from the keypad (returns '\0' if no key pressed)
char get_keypad_key(void);

#endif /* KEYPAD_H_ */
