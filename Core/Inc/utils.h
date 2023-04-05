#ifndef UTILS_H_
#define UTILS_H_

#include <stdbool.h>
#include "main_task.h"

typedef enum
{
	SOLENOID_OFF,
	SOLENOID_ON

} solenoid_position_t;

void set_clutch_solenoid(solenoid_position_t position);
void set_slow_drop(bool state);
void set_upshift_solenoid(solenoid_position_t position);
void set_downshift_solenoid(solenoid_position_t position);
void set_spark_cut(bool state);

#endif /* INC_UTILS_H_ */
