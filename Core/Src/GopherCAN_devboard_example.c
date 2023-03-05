// GopherCAN_devboard_example.c
//  This is a bare-bones module file that can be used in order to make a module main file

#include "GopherCAN_devboard_example.h"
#include "main.h"
#include <stdio.h>

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;


// Use this to define what module this board will be
#define THIS_MODULE_ID TCM_ID
#define PRINTF_HB_MS_BETWEEN 1000
#define HEARTBEAT_MS_BETWEEN 500


// some global variables for examples
U8 last_button_state = 0;
static float ADCReadValue1 = 0;
static float ADCReadValue2 = 0;
static float ADCReadValue3 = 0;

// the CAN callback function used in this example
static void change_led_state(U8 sender, void* UNUSED_LOCAL_PARAM, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
static void init_error(void);

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
	static uint32_t last_test_tick = 0;
	static uint32_t last_test_tick2 = 0;
	static U32 last_print_hb = 0;

	if (HAL_GetTick() - last_heartbeat > HEARTBEAT_MS_BETWEEN)
	{
		last_heartbeat = HAL_GetTick();
		HAL_GPIO_TogglePin(HBEAT_GPIO_Port, HBEAT_Pin);
	}

	// Periodic pin output testing code
//	if (HAL_GetTick() - last_test_tick > 1000)
//	{
//		last_test_tick = HAL_GetTick();
//		HAL_GPIO_TogglePin(UPSHIFT_SOL_GPIO_Port, UPSHIFT_SOL_Pin);
//		HAL_GPIO_TogglePin(SPK_CUT_GPIO_Port, SPK_CUT_Pin);
//		HAL_GPIO_TogglePin(AUX1_C_GPIO_Port, AUX1_C_Pin);
//		HAL_GPIO_TogglePin(AUX2_C_GPIO_Port, AUX2_C_Pin);
//		HAL_GPIO_TogglePin(AUX1_T_GPIO_Port, AUX1_T_Pin);
//
//	}

	if (HAL_GetTick() - last_test_tick2 > 10)
	{
		ADCReadValue3 = get_gear_pot_pos();
		ADCReadValue1 = get_shift_pot_pos();
		ADCReadValue2 = get_clutch_pot_pos();
	}

	// send the current tick over UART every second
	if (HAL_GetTick() - last_print_hb >= PRINTF_HB_MS_BETWEEN)
	{
		printf("Current tick: %lu\n", HAL_GetTick());
		last_print_hb = HAL_GetTick();
	}
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
