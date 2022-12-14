// GopherCAN_devboard_example.h
//  Header file for GopherCAN_devboard_example.c

#ifndef MAIN_TASK_H
#define MAIN_TASK_H

#include "GopherCAN.h"

void init_main_task();
void can_buffer_handling_loop();

#include <stdint.h>

typedef enum
{
	ST_IDLE = 0,
	ST_HDL_UPSHIFT,
	ST_HDL_DOWNSHIFT
} Main_States_t;

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

typedef struct
{
	// Total shifts
	uint32_t TOTAL_SHIFTS;

	// Failed Clutch Disengagement
	uint32_t F_CLUTCH_OPEN;

	// Total number of failed shifts
	uint32_t FS_Total;

	// Number of shifts that can't be executed due to RPM
	uint32_t F_RPM;

	// Number of failed exits and enters
	uint32_t F_D_EXIT;
	uint32_t F_D_ENTER;

	// Number of failed gear exits upshift
	uint32_t F_U_EXIT_NO_CLUTCH;
	uint32_t F_U_EXIT_NO_CLUTCH_AND_SPARK_RETURN;
	uint32_t F_U_EXIT_WITH_CLUTCH;

	// Number of failed gear enters upshift
	uint32_t F_U_ENTER_NO_CLUTCH;
	uint32_t F_U_ENTER_WITH_CLUTCH;

	uint32_t F_RETURN_LEVER;
} logs_t;


int main_task(void);

#define HEARTBEAT_LED_TIME_ms 500
#define DISPLAY_UPDATE_TIME_ms 100
#define GEAR_UPDATE_TIME_ms 25

#define MIN_LAP_TIME_ms 2000

#endif
