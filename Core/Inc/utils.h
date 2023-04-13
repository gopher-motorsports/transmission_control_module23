#ifndef UTILS_H_
#define UTILS_H_

#include <stdbool.h>
#include "main_task.h"

typedef enum
{
	NEUTRAL = 0,
	GEAR_0_5,
	GEAR_1,
	GEAR_1_5,
	GEAR_2,
	GEAR_2_5,
	GEAR_3,
	GEAR_3_5,
	GEAR_4,
	GEAR_4_5,
	GEAR_5,
	ERROR_GEAR,
	NUM_GEARS
} gear_t;

typedef struct tcm_data_struct
{
	uint32_t current_RPM;
	uint32_t target_RPM;
	float trans_speed;
	float wheel_speed;

	gear_t current_gear;
	gear_t target_gear;

	U8 sw_fast_clutch;
	U8 sw_slow_clutch;
	Pending_Shift_t pending_shift;

	bool currently_moving;	// Is the car moving?
	bool gear_established;	// Gear established - Used for determining gear upon startup
	bool using_clutch;		// Are we using the clutch for this shift?
	bool successful_shift;	// Has the shift been successful
	bool anti_stall;		// Anti Stall
	bool clutchless_downshift;
	bool time_shift_only;
} tcm_data_struct_t;

typedef enum
{
	SOLENOID_OFF,
	SOLENOID_ON

} solenoid_position_t;

extern tcm_data_struct_t tcm_data;

void update_tcm_data(void);
float get_trans_speed();
float get_ave_wheel_speed();
U32 get_ECU_RPM();
void reach_target_RPM_spark_cut(uint32_t target_rpm);
void check_buttons_and_set_clutch_sol(solenoid_position_t position);
void safe_spark_cut(bool state);
float get_current_trans_wheel_ratio(void);
float get_current_RPM_trans_ratio();
gear_t get_current_gear();
U32 calc_target_RPM(gear_t target_gear);
bool validate_target_RPM(uint32_t target_rpm, gear_t target_gear, U8 fast_clutch, U8 slow_clutch);
bool calc_validate_upshift(gear_t current_gear, U8 fast_clutch, U8 slow_clutch);
bool calc_validate_downshift(gear_t current_gear, U8 fast_clutch, U8 slow_clutch);

void set_clutch_solenoid(solenoid_position_t position);
void set_slow_clutch_drop(bool state);
void set_upshift_solenoid(solenoid_position_t position);
void set_downshift_solenoid(solenoid_position_t position);
void set_spark_cut(bool state);
float get_gear_pot_pos(void);
float get_clutch_pot_pos(void);
float get_shift_pot_pos(void);

#endif /* INC_UTILS_H_ */
