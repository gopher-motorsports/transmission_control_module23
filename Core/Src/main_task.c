// GopherCAN_devboard_example.c
//  This is a bare-bones module file that can be used in order to make a module main file

#include "main_task.h"
#include "main.h"
#include "utils.h"
#include "gopher_sense.h"
#include <stdio.h>
#include <stdbool.h>

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;

// Use this to define what module this board will be
#define THIS_MODULE_ID TCM_ID
#define PRINTF_HB_MS_BETWEEN 1000
#define HEARTBEAT_MS_BETWEEN 500
#define OVERCURRENT_FAULT_LED_TIME_MS 10000

// some global variables for examples
U8 last_button_state = 0;
U8 error_byte = 0;

// the CAN callback function used in this example
static void change_led_state(U8 sender, void* UNUSED_LOCAL_PARAM, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
static void init_error(void);
static void updateAndQueueParams(void);
static void checkForErrors(void);

// init
//  What needs to happen on startup in order to run GopherCAN
void init(CAN_HandleTypeDef* hcan_ptr)
{
	example_hcan = hcan_ptr;

	// initialize CAN
	// NOTE: CAN will also need to be added in CubeMX and code must be generated
	// Check the STM_CAN repo for the file "F0xx CAN Config Settings.pptx" for the correct settings
	if (init_can(GCAN0, example_hcan, THIS_MODULE_ID, BXTYPE_MASTER))
	{
		init_error();
	}

	// Set the function pointer of SET_LED_STATE. This means the function change_led_state()
	// will be run whenever this can command is sent to the module
	if (add_custom_can_func(SET_LED_STATE, &change_led_state, TRUE, NULL))
	{
		init_error();
	}

	// Clear the error byte, so it has to keep being triggered if the error is persistent (and doesn't require a function to turn it off again)
	error_byte = 0;
}


// can_buffer_handling_loop
//  This loop will handle CAN RX software task and CAN TX hardware task. Should be
//  called every 1ms or as often as received messages should be handled
void can_buffer_handling_loop()
{
	// handle each RX message in the buffer
	if (service_can_rx_buffer())
	{
		// an error has occurred
	}

	// handle the transmission hardware for each CAN bus
	service_can_tx(example_hcan);
}


// main_loop
//  another loop. This includes logic for sending a CAN command. Designed to be
//  called every 10ms
void main_loop()
{
	static uint32_t last_heartbeat = 0;
	static U32 last_print_hb = 0;

	if (HAL_GetTick() - last_heartbeat > HEARTBEAT_MS_BETWEEN)
	{
		last_heartbeat = HAL_GetTick();
		HAL_GPIO_TogglePin(HBEAT_GPIO_Port, HBEAT_Pin);
	}

	checkForErrors();
	updateAndQueueParams();

	// send the current tick over UART every second
	if (HAL_GetTick() - last_print_hb >= PRINTF_HB_MS_BETWEEN)
	{
		printf("Current tick: %lu\n", HAL_GetTick());
		last_print_hb = HAL_GetTick();
	}
}

static void checkForErrors(void) {
	// TODO: Scale this to include different modes and things for error states

	static U32 led_on_start_time = 0;
	static U32 time_on_ms = 0;
	static bool led_on = false;
	if (!HAL_GPIO_ReadPin(SWITCH_FAULT_3V3_GPIO_Port, SWITCH_FAULT_3V3_Pin)) {
		error(SENSE_OUT_OVERCURRENT_3V3, &error_byte);
		led_on = true;
		led_on_start_time = HAL_GetTick();
		// See if this is currently the  led priority
		if (error_byte > (1 << SENSE_OUT_OVERCURRENT_3V3)) {
			time_on_ms = OVERCURRENT_FAULT_LED_TIME_MS;
		}
	}

	if (!HAL_GPIO_ReadPin(SWITCH_FAULT_5V_GPIO_Port, SWITCH_FAULT_5V_Pin)) {
		error(SENSE_OUT_OVERCURRENT_5V, &error_byte);
		led_on = true;
		led_on_start_time = HAL_GetTick();
		// See if this is currently the highest led priority
		if (error_byte > (1 << SENSE_OUT_OVERCURRENT_5V)) {
			time_on_ms = OVERCURRENT_FAULT_LED_TIME_MS;
		}
	}

	if(led_on) {
		if (HAL_GetTick() - led_on_start_time >= time_on_ms) {
			HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, 0);
			led_on = false;
		} else {
			HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, 1);
		}
	}
}

// Updates gcan variables
static void updateAndQueueParams(void) {
	update_and_queue_param_float(&counterShaftSpeed_rpm, tcm_data.trans_speed);
	update_and_queue_param_u16(&tcmTargetRPM_rpm, tcm_data.target_RPM); // Sends it to be logged
	update_and_queue_param_u8(&tcmCurrentGear_state, tcm_data.current_gear);
	update_and_queue_param_u8(&tcmCurrentlyMoving_state, tcm_data.currently_moving);
	update_and_queue_param_u8(&tcmAntiStallActive_state, tcm_data.anti_stall);
	update_and_queue_param_u8(&tcmUsingClutch_state, tcm_data.using_clutch);
	update_and_queue_param_u8(&tcmError_state, error_byte);

	// TODO logic for sending the current state of the shifts
//	switch (car_Main_State)
//	{
//	default:
//	case ST_IDLE:
//		// not shifting, send 0
//		update_and_queue_param_u8(&tcmShiftState_state, 0);
//		break;
//
//	case ST_HDL_UPSHIFT:
//		// send the upshift state
//		update_and_queue_param_u8(&tcmShiftState_state, car_Upshift_State);
//		break;
//
//	case ST_HDL_DOWNSHIFT:
//		// send the downshift state
//		update_and_queue_param_u8(&tcmShiftState_state, car_Downshift_State);
//		break;
//	}
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

// can_callback_function example

// change_led_state
//  a custom function that will change the state of the LED specified
//  by parameter to remote_param. In this case parameter is a U16*, but
//  any data type can be pointed to, as long as it is configured and casted
//  correctly
static void change_led_state(U8 sender, void* parameter, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
	//HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, !!remote_param);
	return;
}


// init_error
//  This function will stay in an infinite loop, blinking the LED in a 0.5sec period. Should only
//  be called from the init function before the RTOS starts
void init_error(void)
{
	while (1)
	{
		HAL_GPIO_TogglePin(HBEAT_GPIO_Port, HBEAT_Pin);
		HAL_Delay(250);
	}
}

// end of GopherCAN_example.c
