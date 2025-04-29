/*
 * keypad.c
 *
 *  Created on: Apr 14, 2025
 *      Author: lucag
 */

#include "keypad.h"
extern I2C_HandleTypeDef hi2c1;  // defined in main.c

#define REG_KEY_DATA_1 0x27  // col register
#define REG_KEY_DATA_2 0x28  // row register

const char keypad_map[4][4] = {
	{'1', '2', '3', 'A'},
	{'4', '5', '6', 'B'},
	{'7', '8', '9', 'C'},
	{'*', '0', '#', 'D'}
};

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
					return keypad_map[r][c];
				}
			}
		}
	}

	return '\0';
}

