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

typedef enum
{
	SOLENOID_OFF,
	SOLENOID_ON

} solenoid_position_t;

gear_t get_current_gear(float gear_pot_pos);
float get_gear_pot_pos(void);
void set_clutch_solenoid(solenoid_position_t position);
void set_slow_drop(bool state);
void set_upshift_solenoid(solenoid_position_t position);
void set_downshift_solenoid(solenoid_position_t position);
void set_spark_cut(bool state);

#endif /* INC_UTILS_H_ */
