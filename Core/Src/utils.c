#include <math.h>
#include "utils.h"
#include "shift_parameters.h"

tcm_data_struct_t tcm_data = {.current_gear = ERROR_GEAR};

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
