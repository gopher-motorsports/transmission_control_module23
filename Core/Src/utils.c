/*
 * Tuning List
 * 1. Gear wheel ratios - Last year was for wheel speed and is now for transmission speed
 */

#include <math.h>
#include "utils.h"
#include "shift_parameters.h"

tcm_data_struct_t tcm_data = {.current_gear = ERROR_GEAR};

extern U8 error_byte;

const float gear_ratios[5] = {
		GEAR_1_TRANS_RATIO,
		GEAR_2_TRANS_RATIO,
		GEAR_3_TRANS_RATIO,
		GEAR_4_TRANS_RATIO,
		GEAR_5_TRANS_RATIO
};
const float GEAR_POT_DISTANCES_mm[] = {
		NEUTRAL_DISTANCE_mm,
		GEAR_1_DISTANCE_mm,
		GEAR_2_DISTANCE_mm,
		GEAR_3_DISTANCE_mm,
		GEAR_4_DISTANCE_mm,
		GEAR_5_DISTANCE_mm
};

float rpm_arr[RPM_ARRAY_SIZE] = {0};
uint32_t rpm_idx = 0;

void update_tcm_data(void)
{
	update_rpm_arr();
	tcm_data.current_RPM = get_ECU_RPM();
	tcm_data.currently_moving = (tcm_data.trans_speed != 0);
}

// check_buttons_and_set_clutch_sol
//  for use of clutch during shifting. This will make sure the driver is not pressing
//  one of the clutch buttons before closing the clutch
void check_buttons_and_set_clutch_sol(solenoid_position_t position)
{
	// If close clutch request comes in when driver is holding button do not drop clutch
	if (position == SOLENOID_OFF && (tcm_data.sw_fast_clutch
			                         || tcm_data.sw_slow_clutch) )
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

	if (!tcm_data.time_shift_only && (tcm_data.target_gear == NEUTRAL
			|| tcm_data.current_gear == NEUTRAL || tcm_data.current_RPM < MIN_SPARK_CUT_RPM))
	{
		set_spark_cut(false);
		return;
	}

	set_spark_cut(state);
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

// calc_target_RPM
//  Using target gear and wheel speed return the RPM we need to hit to enter that gear
U32 calc_target_RPM(gear_t target_gear) {
	// If car isn't moving then there isn't a target RPM
	if (!tcm_data.currently_moving)
	{
		return 0;
	}

	if (target_gear == NEUTRAL || target_gear == ERROR_GEAR) {
		// If we are in ERROR GEAR or shifting into neutral no target RPM
		return 0;
	}

	return tcm_data.trans_speed * gear_ratios[target_gear - 2];
}

// calc_validate_upshift
//  will check if an upshift is valid in the current state of the car. Will also
//  set the target gear and target RPM if the shift is valid
bool calc_validate_upshift(gear_t current_gear, U8 fast_clutch, U8 slow_clutch)
{
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
			error(NO_CLUTCH_OUT_NUETRAL, &error_byte);
			return false;
		}

	case GEAR_1:
	case GEAR_2:
	case GEAR_3:
	case GEAR_4:
		tcm_data.target_gear = current_gear + 2;
		tcm_data.target_RPM = calc_target_RPM(tcm_data.target_gear);
		// always allow shifts for now
		//return validate_target_RPM();
		return true;
	case GEAR_0_5:
	case GEAR_1_5:
	case GEAR_2_5:
	case GEAR_3_5:
	case GEAR_4_5:
		tcm_data.target_gear = current_gear + 1;
		tcm_data.target_RPM = calc_target_RPM(tcm_data.target_gear);
	case GEAR_5:
		error(GEAR_LIMIT, &error_byte);
		return false;
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
		// Clutch must be pulled to go from 1st back to nuetral
		if (fast_clutch || slow_clutch)
		{
			tcm_data.target_gear = NEUTRAL;
			return true;
		}
		else
		{
			error(NO_CLUTCH_OUT_NUETRAL, &error_byte);
			return false;
		}
	case GEAR_2:
	case GEAR_3:
	case GEAR_4:
	case GEAR_5:
		tcm_data.target_gear = current_gear - 2;
		// for now always allow downshifts, even if the target RPM is too high
		//return validate_target_RPM();
		return true;
	case GEAR_0_5:
	case GEAR_1_5:
	case GEAR_2_5:
	case GEAR_3_5:
	case GEAR_4_5:
		tcm_data.target_gear = current_gear - 1;
	case NEUTRAL:
		error(GEAR_LIMIT, &error_byte);
		return false;
	case ERROR_GEAR:
	default:
		tcm_data.target_gear = ERROR_GEAR;
		tcm_data.target_RPM = 0;
		return true;
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

//

//  Continuously add values to the RPM array whenever the clutch is not open
void update_rpm_arr(void)
{
	// dont update the wheel or RPM array if the clutch is open. This will mean gear calculations
    // will start from the last non-clutch samples, which is probably fine
	if (clutch_open()) return;

	rpm_arr[rpm_idx++] = get_ECU_RPM();
	rpm_idx = rpm_idx % RPM_ARRAY_SIZE;
}

void error(tcm_errors_t tcm_error, U8* error_store_location) {
	HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, 1);
	*error_store_location |= (1 << tcm_error);
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
	// Make sure other solenoid is off so they don't push on each other - priority to most recent call
	if (position == SOLENOID_ON) {
		set_downshift_solenoid(SOLENOID_OFF);
	}
	HAL_GPIO_WritePin(UPSHIFT_SOL_GPIO_Port, UPSHIFT_SOL_Pin, position);
}

void set_downshift_solenoid(solenoid_position_t position)
{
	// Make sure other solenoid is off so they don't push on each other - priority to most recent call
	if (position == SOLENOID_ON) {
		set_upshift_solenoid(SOLENOID_OFF);
	}
	HAL_GPIO_WritePin(DOWNSHIFT_SOL_GPIO_Port, DOWNSHIFT_SOL_Pin, position);
}

void set_spark_cut(bool state)
{
	HAL_GPIO_WritePin(SPK_CUT_GPIO_Port, SPK_CUT_Pin, !state);
	tcm_data.spark_cut = state;
}

// get_ave_rpm
//  Returns the the average RPM over a certain amount of time based on
//  the parameter ms_of_samples. Limited to the size of the RPM array
float get_ave_rpm(U32 ms_of_samples)
{
	int32_t starting_index;
	float total = 0.0f;

	// if more data points before are needed than the size of the array, cut it
	// off
	if (ms_of_samples > RPM_ARRAY_SIZE)
	{
		ms_of_samples = RPM_ARRAY_SIZE;
	}
	if (ms_of_samples == 0)
	{
		ms_of_samples = 1;
	}

	// find the first point to average from. Rollover is possible
	starting_index = rpm_idx - ms_of_samples;
	if (starting_index > 0)
	{
		starting_index += RPM_ARRAY_SIZE;
	}

	// count up the most recent values. Rollover must be accounted for
	for (U32 c = 0; c < ms_of_samples; c++)
	{
		total += rpm_arr[(starting_index + c) % RPM_ARRAY_SIZE];
	}

	// return the average
	return (total / ms_of_samples);
}

float temp1, temp2;

// get_current_gear
//  returns the current gear the car is in. This is done first by checking the neutral
//  sensor, then by using the transmission speed to find the correct ratio from the
//  RPM to the wheels. If a gear has been established, it is unlikely that we
//  changed gears so use more samples and take the closest based on the ratio.
//  If the gear is not established then the gear ratios must be closer to the gear

float minimum_rpm_difference = 15000.0f;
float temp_diff;
float trans_speed;
float ave_rpm;
float theoredical_rpm;
uint8_t best_gear = 0;

gear_t get_current_gear()
{
#ifdef NO_GEAR_POT

	minimum_rpm_difference = 15000.0f;
	// If we are currently shifting just use the last know gear
	if (main_state == ST_HDL_UPSHIFT || main_state == ST_HDL_DOWNSHIFT)
	{
		// no established gear during shifting
		tcm_data.gear_established = false;
		return tcm_data.current_gear;
	}

	// if the clutch is open return the last gear (shifting updates the current gear
	// on a success so you should still be able to reasonably go up and down)
	// same if we are not moving. We have no data in order to change the gear
	if (clutch_open()) //|| !tcm_data.currently_moving)
	{
		// no change to established gear, if we were good before we are still good
		// and vice versa
		return tcm_data.current_gear;
	}

	if (tcm_data.gear_established)
	{
		// if the gear is established, use a much longer set of samples and take the
		// closest gear
		temp1 = ave_rpm = get_ave_rpm(GEAR_ESTABLISHED_NUM_SAMPLES_ms);
		temp2 = trans_speed = tcm_data.trans_speed; // TODO - need custom amount of samples
		for (uint8_t c = 0; c < NUM_OF_GEARS; c++)
		{
			theoredical_rpm = trans_speed * gear_ratios[c];
			temp_diff = fabs(theoredical_rpm - ave_rpm);
			if (temp_diff < minimum_rpm_difference)
			{
				minimum_rpm_difference = temp_diff;
				best_gear = c + 1;
			}
		}

		// we have found the minimum difference. Just return this gear. Multiply by 2 to not use in between gears.
		return (gear_t)(best_gear * 2);
	}
	else
	{
		// if the gear is not established, we must be close enough to a gear to establish
		// it. This will use a smaller subset of samples
		ave_rpm = get_ave_rpm(GEAR_NOT_ESTABLISHED_NUM_SAMPLES_ms);
		trans_speed = tcm_data.trans_speed; // TODO - need custom amount of samples
		for (uint8_t c = 0; c < NUM_OF_GEARS; c++)
		{
			theoredical_rpm = trans_speed * gear_ratios[c];
			temp_diff = fabs(theoredical_rpm - ave_rpm);
			if (temp_diff < minimum_rpm_difference)
			{
				minimum_rpm_difference = temp_diff;
				best_gear = c + 1;
			}
		}

		// we have found the minimum difference. If it is within tolerance then
		// return the gear and establish. Otherwise return error gear and do not
		// establish
		if (minimum_rpm_difference / ave_rpm <= GEAR_ESTABLISH_TOLERANCE_percent)
		{
			tcm_data.gear_established = true;
			// Multiply by 2 to not use in between gears.
			return (gear_t)(best_gear * 2);
		}
		else
		{
			// no gear is good enough. Return error gear
			return ERROR_GEAR;
		}
	}

	// not sure how we got here. Return ERROR_GEAR and panic
	return ERROR_GEAR;
#else
	// Search algorithm searches for if the gear position is less than a gear position distance
	// plus the margin (0.1mm), and if it finds it, then checks if the position is right on the gear
	// or between it and the last one by checking if the position is less than the declared
	// distance minus the margin (0.1mm)
	float gear_position = get_gear_pot_pos();
	for(int i = 0; i < NUM_GEARS / 2; i++) {
		if (gear_position >= GEAR_POT_DISTANCES_mm[i] - GEAR_POS_MARGIN_mm) {
			if (gear_position >= GEAR_POT_DISTANCES_mm[i] + GEAR_POS_MARGIN_mm) {
				return (gear_t)(i * 2 - 1);
			}
			return (gear_t)(i * 2);
		}
	}
	// not sure how we got here. Return ERROR_GEAR and panic
	return ERROR_GEAR;
#endif
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

U32 get_ECU_RPM()
{
	return engineRPM_rpm.data;
}

bool clutch_open(void)
{
	return get_clutch_pot_pos() > CLUTCH_OPEN_POS_MM;
}

// Functions currently only used for debugging

// current_RPM_trans_ratio
//  Used for debugging/integration. Returns the current ratio between RPM and
//  trans speed
float get_current_RPM_trans_ratio(void) {
	float trans_speed = tcm_data.trans_speed;
	return (fabs(trans_speed) < 1e-6f) ? -1.0f : (get_ECU_RPM()/trans_speed);
}
