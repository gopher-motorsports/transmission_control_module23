#ifndef UTILS_H_
#define UTILS_H_

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

typedef struct tcm_data_struct
{
	uint32_t current_RPM;
	uint32_t target_RPM;
	uint32_t trans_speed;

	float wheel_speed;

	gear_t current_gear;
	gear_t target_gear;

	bool currently_moving;	// Is the car moving?
	bool gear_established;	// Gear established - Used for determining gear upon startup
	bool using_clutch;		// Are we using the clutch for this shift?
	bool successful_shift;	// Has the shift been successful
	bool anti_stall;		// Anti Stall
} tcm_data_struct_t;

typedef enum
{
	zero,
	SENSE_OUT_OVERCURRENT_3V3,
	SENSE_OUT_OVERCURRENT_5V,
	SHIFT_POSITION_TIMEOUT,
	CLUTCH_POSITION_TIMEOUT
} tcm_errors_t;

typedef enum
{
	SOLENOID_OFF,
	SOLENOID_ON

} solenoid_position_t;

extern tcm_data_struct_t tcm_data;

void error(tcm_errors_t tcm_error, U8* error_store_location);
void set_clutch_solenoid(solenoid_position_t position);
void set_slow_drop(bool state);
void set_upshift_solenoid(solenoid_position_t position);
void set_downshift_solenoid(solenoid_position_t position);
void set_spark_cut(bool state);

#endif /* INC_UTILS_H_ */
