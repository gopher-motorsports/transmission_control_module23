/*
 * Testing list:
 * 1. Make sure variables update correctly
 * 2. Clutch task - make sure all states covered - fast, slow, bites after distance, different when idle, different if in antistall.
 */


#include <math.h>
#include "utils.h"
#include "shift_parameters.h"

tcm_data_struct_t tcm_data = {.current_gear = ERROR_GEAR};

const float gear_ratios[5] = {
		GEAR_1_WHEEL_RATIO,
		GEAR_2_WHEEL_RATIO,
		GEAR_3_WHEEL_RATIO,
		GEAR_4_WHEEL_RATIO,
		GEAR_5_WHEEL_RATIO
};
const float GEAR_POT_DISTANCES_mm[] = {
		NEUTRAL_DISTANCE_mm,
		GEAR_1_DISTANCE_mm,
		GEAR_2_DISTANCE_mm,
		GEAR_3_DISTANCE_mm,
		GEAR_4_DISTANCE_mm,
		GEAR_5_DISTANCE_mm
};

void update_tcm_data(void)
{
	tcm_data.currently_moving = get_ave_wheel_speed() > MOVING_WHEEL_SPEED_MIN_CUTOFF;
	tcm_data.current_RPM = get_ECU_RPM();
	tcm_data.trans_speed = get_trans_speed();
	tcm_data.wheel_speed = get_ave_wheel_speed();
}

float get_trans_speed() {
	return 0; // TODO Figure out how to do this, prob just set up input capture library to update tcm_data.trans_speed.
}

// reach_target_RPM_spark_cut
//  if the current RPM is higher than the target RPM, spark cut the engine. All
//  safeties are performed in spark_cut
void reach_target_RPM_spark_cut(uint32_t target_rpm)
{
	// if the target RPM is too low, do not spark cut
	if (target_rpm < MIN_SPARK_CUT_RPM)
	{
		safe_spark_cut(false);
	}

	// if the current RPM is higher than the target RPM, spark cut to reach it
	else if (tcm_data.current_RPM > target_rpm)
	{
		safe_spark_cut(true);
	}
	else
	{
		safe_spark_cut(false);
	}
}

// check_buttons_and_set_clutch_sol
//  for use of clutch during shifting. This will make sure the driver is not pressing
//  one of the clutch buttons before closing the clutch
void check_buttons_and_set_clutch_sol(solenoid_position_t position)
{
	// If close clutch request comes in when driver is holding button do not drop clutch
	if (position == SOLENOID_OFF && (swFastClutch_state.data
			                         || swSlowClutch_state.data) )
	{
		set_clutch_solenoid(SOLENOID_ON);
		return;
	}

	set_clutch_solenoid(position);
}

// safe_spark_cut
//  set the spark cut state as inputed. Do not allow spark cutting when we are
//  entering or exiting neutral, or if the current RPM is already too low.
//  NOTE: if we loose RPM from CAN we lose spark cut in this config
void safe_spark_cut(bool state)
{
	// dont allow spark cut while entering or exiting neutral or if we are already
	// below the minimum allowable RPM
	if (tcm_data.target_gear == NEUTRAL || tcm_data.current_gear == NEUTRAL
			|| tcm_data.current_RPM < MIN_SPARK_CUT_RPM)
	{
		set_spark_cut(false);
		return;
	}

	set_spark_cut(state);
}

// get_ave_rpm
//  Returns the the average RPM over a certain amount of time based on
//  the parameter ms_of_samples. Limited to the size of the RPM array
float get_ave_rpm() {
	return 0; // TODO
}

// current_trans_wheel_ratio
//  Used for debugging/integration. Returns the current ratio between
//  wheel speed and trans speed
float current_trans_wheel_ratio(void)
{
	return 0; // TODO
}

// current_RPM_trans_ratio
//  Used for debugging/integration. Returns the current ratio between RPM and
//  trans speed
float current_RPM_trans_ratio(void) {
	float trans_speed = tcm_data.trans_speed;
	return (fabs(trans_speed) < 1e-6f) ? -1.0f : (get_ECU_RPM()/trans_speed);
}

// get_current_gear
// Uses the positions declared in GEAR_POT_DISTANCES_mm which are set in shift_parameters.h
// and interpolates between them to determine the gear state
//TODO: TEST THIS BEFORE ATTEMPTING TO RUN - MATH IS PROBABLY WRONG
gear_t get_current_gear(float gear_pot_pos)
{
	// Search algorithm searches for if the gear position is less than a gear position distance
	// plus the margin (0.1mm), and if it finds it, then checks if the position is right on the gear
	// or between it and the last one by checking if the position is less than the declared
	// distance minus the margin (0.1mm)
	uint8_t gear_position = gear_pot_pos;
	for(int i = 1; i < NUM_GEARS / 2; i++) {
		if (gear_position <= GEAR_POT_DISTANCES_mm[i] + GEAR_POS_MARGIN_mm) {
			if (gear_position <= GEAR_POT_DISTANCES_mm[i] - GEAR_POS_MARGIN_mm) {
				return (gear_t)(i * 2 + 1);
			}
			return (gear_t)(i * 2);
		}
	}
	// not sure how we got here. Return ERROR_GEAR and panic
	return ERROR_GEAR;
}

// calc_target_RPM
//  Using target gear and wheel speed return the RPM we need to hit to enter that
//  gear
U32 calc_target_RPM(gear_t target_gear) {
	// If car isn't moving then there isn't a target RPM
	if (!tcm_data.currently_moving)
	{
		return 0;
	}

	switch (target_gear)
	{
	case GEAR_1:
	case GEAR_2:
	case GEAR_3:
	case GEAR_4:
	case GEAR_5:
		// This formula holds regardless of whether or not the clutch is pressed
		return get_ave_wheel_speed(DEFAULT_WHEEL_SPEED_AVE_TIME_ms) * gear_ratios[target_gear - 1];

	case NEUTRAL:
	case ERROR_GEAR:
	default:
		// If we are in ERROR GEAR or shifting into neutral no target RPM
		return 0;
	}
}

// validate_target_RPM
// check if an inputed RPM is within the acceptable range
bool validate_target_RPM(uint32_t target_rpm, gear_t target_gear, U8 fast_clutch, U8 slow_clutch)
{
	// If we are getting into ERROR_GEAR or NEUTRAL or clutch button pressed valid shift
	// Example we are rolling at 2mph and driver is holding clutch and wants to shift into
	// 5th. Should be allowed and if they drop clutch then anti stall kicks in
	if (	target_gear == ERROR_GEAR 	||
			target_gear == NEUTRAL 		||
			fast_clutch ||
			slow_clutch )
	{
		return true;
	}
	// If target RPM not valid return false
	if (target_rpm < MIN_SPARK_CUT_RPM || MAX_RPM < target_rpm)
	{
		return false;
	}

	// everything is good, return true
	return true;
}

// calc_validate_upshift
//  will check if an upshift is valid in the current state of the car. Will also
//  set the target gear and target RPM if the shift is valid
//  TODO: Deal with in-between gears
bool calc_validate_upshift(gear_t current_gear, U8 fast_clutch, U8 slow_clutch) {
	switch (current_gear)
		{
		case NEUTRAL:
			// Clutch must be pressed to go from NEUTRAL -> 1st
			if (fast_clutch || slow_clutch)
			{
				tcm_data.target_RPM = 0;
				tcm_data.target_gear = GEAR_1;
				return true;
			}
			else
			{
				return false;
			}

		case GEAR_1:
		case GEAR_2:
		case GEAR_3:
		case GEAR_4:
			tcm_data.target_gear = current_gear + 1;
			tcm_data.target_RPM = calc_target_RPM(tcm_data.target_gear);
			// always allow shifts for now
			//return validate_target_RPM();
			return true;

		case GEAR_5:
		case ERROR_GEAR:
		default:
			tcm_data.target_gear = ERROR_GEAR;
			tcm_data.target_RPM = 0;
			return true;
		}
}

// calc_validate_downshift
//  will check if an downshift is valid in the current state of the car. Will also
//  set the target gear and target RPM if the shift is valid
bool calc_validate_downshift(gear_t current_gear, U8 fast_clutch, U8 slow_clutch)
{
	switch (current_gear)
	{
	case GEAR_1:
	case GEAR_2:
	case GEAR_3:
	case GEAR_4:
	case GEAR_5:
		tcm_data.target_gear = current_gear - 1;
		tcm_data.target_RPM = calc_target_RPM(tcm_data.target_gear);
		// for now always allow downshifts, even if the target RPM is too high
		//return validate_target_RPM();
		return true;

	case NEUTRAL:
	case ERROR_GEAR:
	default:
		tcm_data.target_gear = ERROR_GEAR;
		tcm_data.target_RPM = 0;
		return true;
	}
}

void set_clutch_solenoid(solenoid_position_t position)
{
	HAL_GPIO_WritePin(CLUTCH_SOL_GPIO_Port, CLUTCH_SOL_Pin, position);
}

void set_slow_clutch_drop(bool state)
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

float get_ave_wheel_speed() {
	float totalSpeed = wheelSpeedFrontLeft_mph.data + wheelSpeedFrontRight_mph.data + wheelSpeedRearRight_mph.data + wheelSpeedRearLeft_mph.data;
	return totalSpeed / NUM_WHEELS;
}

U32 get_ECU_RPM() {
	return engineRPM_rpm.data;
}
