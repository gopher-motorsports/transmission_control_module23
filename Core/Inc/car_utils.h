/*
 * car_utils.h
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */

#ifndef INC_CAR_UTILS_H_
#define INC_CAR_UTILS_H_

#include <stdbool.h>
#include "main_task.h"


typedef enum
{
	NEUTRAL,
	GEAR_1,
	GEAR_2,
	GEAR_3,
	GEAR_4,
	GEAR_5,
	ERROR_GEAR
} gear_t;

typedef struct shift_struct
{
	uint32_t current_RPM;
	uint32_t target_RPM;

	uint32_t trans_speed;

	float wheel_speed;

	gear_t current_gear;
	gear_t target_gear;

	// Is the car moving?
	bool currently_moving;
	// Gear established - Used for determining gear upon startup
	bool gear_established;
	// Are we using the clutch for this shift?
	bool using_clutch;
	// Has the shift been successful
	bool successful_shift;
	// Anti Stall
	bool anti_stall;
} shift_struct_t;


typedef enum
{
	SOLENOID_OFF,
	SOLENOID_ON

} solenoid_position_t;

extern shift_struct_t car_shift_data;

void update_car_shift_struct(void);
void clutch_task(U8_CAN_STRUCT fast_clutch, U8_CAN_STRUCT slow_clutch, Main_States_t car_state, bool anti_stall_active);
void check_buttons_and_set_clutch_sol(solenoid_position_t position, U8_CAN_STRUCT fast_clutch, U8_CAN_STRUCT slow_clutch);
void safe_spark_cut(bool state);
void reach_target_RPM_spark_cut(uint32_t target_rpm);
bool anti_stall(U8_CAN_STRUCT fast_clutch, U8_CAN_STRUCT slow_clutch, gear_t current_gear);
gear_t get_current_gear(Main_States_t current_state);
uint32_t calc_target_RPM(gear_t target_gear);
bool validate_target_RPM(uint32_t target_rpm, gear_t target_gear, U8_CAN_STRUCT fast_clutch, U8_CAN_STRUCT slow_clutch);
bool calc_validate_upshift(gear_t current_gear, U8_CAN_STRUCT fast_clutch, U8_CAN_STRUCT slow_clutch);
bool calc_validate_downshift(gear_t current_gear, U8_CAN_STRUCT fast_clutch, U8_CAN_STRUCT slow_clutch);
float get_ave_wheel_speed(uint32_t ms_of_samples);
float get_ave_rpm(uint32_t ms_of_samples);
void update_wheel_arr(void);
void update_rpm_arr(void);
float get_wheel_speed(void);
float current_trans_wheel_ratio(void);
float current_RPM_trans_ratio(void);
uint32_t get_trans_speed(void);

// HW functions
void set_clutch_solenoid(solenoid_position_t position);
void set_slow_drop(bool state);
void set_upshift_solenoid(solenoid_position_t position);
void set_downshift_solenoid(solenoid_position_t position);

uint32_t get_RPM(void);
float get_clutch_pot_pos(void);
float get_shift_pot_pos(void);
bool clutch_open(void);
void set_spark_cut(bool state);


#endif /* INC_CAR_UTILS_H_ */
