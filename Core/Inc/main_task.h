// GopherCAN_devboard_example.h
//  Header file for GopherCAN_devboard_example.c

#ifndef GOPHERCAN_DEVBOARD_EXAMPLE_H
#define GOPHERCAN_DEVBOARD_EXAMPLE_H

#include "GopherCAN.h"

typedef enum
{
	ST_IDLE = 0,
	ST_HDL_UPSHIFT,
	ST_HDL_DOWNSHIFT
} Main_States_t;

typedef enum
{
	NONE = 0,
	UPSHIFT,
	DOWNSHIFT
} Pending_Shift_t;

typedef enum
{
	ST_U_BEGIN_SHIFT = 0,
	ST_U_LOAD_SHIFT_LVR,
	ST_U_EXIT_GEAR,
	ST_U_SPARK_RETURN,
	ST_U_ENTER_GEAR,
	ST_U_FINISH_SHIFT,
} Upshift_States_t;

typedef enum
{
	ST_D_BEGIN_SHIFT = 0,
	ST_D_LOAD_SHIFT_LVR,
	ST_D_EXIT_GEAR,
	ST_D_ENTER_GEAR,
	ST_D_HOLD_CLUTCH,
	ST_D_FINISH_SHIFT
} Downshift_States_t;

void init(CAN_HandleTypeDef* hcan_ptr);
void can_buffer_handling_loop();
void main_loop();

#define GEAR_UPDATE_TIME_MS 25

#endif
