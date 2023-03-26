#include <math.h>
#include "utils.h"
#include "shift_parameters.h"

const float GEAR_POT_DISTANCES_mm[] = {
		NEUTRAL_DISTANCE_mm,
		GEAR_1_DISTANCE_mm,
		GEAR_2_DISTANCE_mm,
		GEAR_3_DISTANCE_mm,
		GEAR_4_DISTANCE_mm,
		GEAR_5_DISTANCE_mm
};

// get_current_gear
// Uses the positions declared in GEAR_POT_DISTANCES_mm which are set in shift_parameters.h
// and interpolates between them to determine the gear state
gear_t get_current_gear(float gear_pot_pos)
{
	// Search algorithm searches for if the gear position is less than a gear position distance
	// plus the margin (0.1mm), and if it finds it, then checks if the position is right on the gear
	// or between it and the last one by checking if the position is less than the declared
	// distance minus the margin (0.1mm)
	float gear_position = gear_pot_pos;
	for(int i = 0; i < NUM_GEARS / 2; i++) {
		if (gear_position <= GEAR_POT_DISTANCES_mm[i] + GEAR_POS_MARGIN_mm) {
			if (gear_position <= GEAR_POT_DISTANCES_mm[i] - GEAR_POS_MARGIN_mm) {
				return (gear_t)(i * 2 - 1);
			}
			return (gear_t)(i * 2);
		}
	}
	// not sure how we got here. Return ERROR_GEAR and panic
	return ERROR_GEAR;
}

float get_gear_pot_pos(void)
{
	return gearPosition_mm.data;
}

float get_clutch_pot_pos(void)
{
	return clutchPosition_mm.data;
}

float get_shift_pot_pos(void)
{
	return shifterPosition_mm.data;
}

void set_clutch_solenoid(solenoid_position_t position)
{
	HAL_GPIO_WritePin(CLUTCH_SOL_GPIO_Port, CLUTCH_SOL_Pin, position);
}

void set_slow_drop(bool state)
{
	HAL_GPIO_WritePin(SLOW_CLUTCH_SOL_GPIO_Port, SLOW_CLUTCH_SOL_Pin, state);
}

void set_upshift_solenoid(solenoid_position_t position)
{
	HAL_GPIO_WritePin(UPSHIFT_SOL_GPIO_Port, UPSHIFT_SOL_Pin, position);
}

void set_downshift_solenoid(solenoid_position_t position)
{
	HAL_GPIO_WritePin(DOWNSHIFT_SOL_GPIO_Port, DOWNSHIFT_SOL_Pin, position);
}

void set_spark_cut(bool state)
{
	HAL_GPIO_WritePin(SPK_CUT_GPIO_Port, SPK_CUT_Pin, state);
}
