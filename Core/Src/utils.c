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
	tcm_data.current_RPM = get_RPM();
	tcm_data.trans_speed = get_trans_speed();
	tcm_data.wheel_speed = get_ave_wheel_speed();
}

float get_trans_speed() {
	return 0; // TODO Figure out how to do this, prob just set up input capture library to update tcm_data.trans_speed.
}

float get_ave_wheel_speed() {
	float totalSpeed = wheelSpeedFrontLeft_mph.data + wheelSpeedFrontRight_mph.data + wheelSpeedRearRight_mph.data + wheelSpeedRearLeft_mph.data;
	return totalSpeed / NUM_WHEELS;
}

U32 get_RPM() {
	return engineRPM_rpm.data;
}

// clutch_task
//  for use in the main task. Sets the slow and fast drop accordingly and handles
//  clutch position if in ST_IDLE
void clutch_task(Main_States_t car_state) {
	static bool using_slow_drop = false;
	// normal clutch button must not be pressed when using slow drop. Fast drop is
	// given priority
	if (swFastClutch_state.data) using_slow_drop = false;

	// if the slow drop button is pressed latch the slow drop
	else if (swSlowClutch_state.data) using_slow_drop = true;

	// If either clutch button pressed then enable solenoid. Always turn it on regardless of
	// if we are shifting or not
	if (swFastClutch_state.data || swSlowClutch_state.data)
	{
		set_clutch_solenoid(SOLENOID_ON);
		return;
	}

	// If neither clutch button pressed and we are in IDLE and not in anti stall
	// close clutch solenoid. This will cause clutch presses to latch to the end
	// of a shift
	if (!(swFastClutch_state.data || swSlowClutch_state.data)
			&& car_state == ST_IDLE && tcm_data.anti_stall)
	{
		set_clutch_solenoid(SOLENOID_OFF);

		// if we are using slow drop enable or disable the slow release valve depending
		// on if we are near the bite point
		if (using_slow_drop)
		{
			// when slow dropping, we want to start by fast dropping until the bite point
			if (get_clutch_pot_pos() < CLUTCH_OPEN_POS_MM - CLUTCH_SLOW_DROP_FAST_TO_SLOW_EXTRA_MM)
			{
				set_slow_drop(false);
			}
			else
			{
				set_slow_drop(true);
			}
		}
		else
		{
			// not using slow drop
			set_slow_drop(false);
		}
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
float current_RPM_trans_ratio() {
	return 0; // TODO
}

// get_current_gear
// Uses the positions declared in GEAR_POT_DISTANCES_mm which are set in shift_parameters.h
// and interpolates between them to determine the gear state
//TODO: TEST THIS BEFORE ATTEMPTING TO RUN - MATH IS PROBABLY WRONG
gear_t get_current_gear(Main_States_t current_state)
{
	// Search algorithm searches for if the gear position is less than a gear position distance
	// plus the margin (0.1mm), and if it finds it, then checks if the position is right on the gear
	// or between it and the last one by checking if the position is less than the declared
	// distance minus the margin (0.1mm)
	uint8_t gear_position = get_gear_pot_pos();
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
U32 calc_target_RPM() {
	return 0; // TODO
}

// validate_target_RPM
//  check if an inputed RPM is within the acceptable range
bool validate_target_RPM() {
	return false; // TODO
}

// calc_validate_upshift
//  will check if an upshift is valid in the current state of the car. Will also
//  set the target gear and target RPM if the shift is valid
bool calc_validate_upshift() {
	return false; // TODO
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
