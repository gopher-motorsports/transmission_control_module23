// GopherCAN_devboard_example.h
//  Header file for GopherCAN_devboard_example.c

#ifndef MAIN_TASK_H_1
#define MAIN_TASK_H_1

#include "GopherCAN.h"
#include "gopher_sense.h"

#define FAST_CLUTCH_BUTTON (swFastClutch_state.data)
#define SLOW_CLUTCH_BUTTON (swSlowClutch_state.data)
#define TIME_SHIFT_ONLY_BUTTON (swButon2_state.data)
#define CLUTCHLESS_DOWNSHIFT_BUTTON (swButon3_state.data)
#define UPSHIFT_BUTTON (swUpshift_state.data)
#define DOWNSHIFT_BUTTON (swDownshift_state.data)

// Use this to define what module this board will be
#define THIS_MODULE_ID TCM_ID
#define PRINTF_HB_MS_BETWEEN 1000
#define HEARTBEAT_MS_BETWEEN 500
#define TCM_DATA_UPDATE_MS_BETWEEN 10

// Fault LED times
#define OVERCURRENT_FAULT_LED_MS 10000
#define SHIFT_SAFETY_FAULT_LED_MS 1000
#define SHIFT_TIMEOUT_FAULT_LED_MS 1000

//#define AUTO_SHIFT_LEVER_RETURN
#define CAN_CLUTCHLESS_DOWNSHIFT
#define CAN_CHANGE_FROM_TIME_SHIFT
//#define SHIFT_DEBUG
//#define RUN_TIME_STATS
//#define SMART_SLOW_CLUTCH

typedef enum
{
	ST_IDLE = 0,
	ST_HDL_UPSHIFT,
	ST_HDL_DOWNSHIFT
} Main_States_t;

extern Main_States_t main_state;

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
	ST_U_EXTRA_PUSH,
	ST_U_FINISH_SHIFT,
} Upshift_States_t;

typedef enum
{
	ST_D_BEGIN_SHIFT = 0,
	ST_D_LOAD_SHIFT_LVR,
	ST_D_EXIT_GEAR,
	ST_D_ENTER_GEAR,
	ST_D_HOLD_CLUTCH,
	ST_D_EXTRA_PUSH,
	ST_D_FINISH_SHIFT
} Downshift_States_t;

void init(CAN_HandleTypeDef* hcan_ptr);
void can_buffer_handling_loop();
void main_loop();

#define GEAR_UPDATE_TIME_MS 25

#endif
