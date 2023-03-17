// GopherCAN_devboard_example.c
//  This is a bare-bones module file that can be used in order to make a module main file

#include "main_task.h"
#include "main.h"
#include "utils.h"
#include "gopher_sense.h"
#include <stdio.h>
#include "shift_parameters.h"
#include <string.h>

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;

// Use this to define what module this board will be
#define THIS_MODULE_ID TCM_ID
#define PRINTF_HB_MS_BETWEEN 1000
#define HEARTBEAT_MS_BETWEEN 500
#define TCM_DATA_UPDATE_MS_BETWEEN 10

// some global variables for examples
static Main_States_t main_State = ST_IDLE;
static Upshift_States_t upshift_State;
static Downshift_States_t downshift_State;
U8 last_button_state = 0;
static Upshift_States_t lastUpshiftState;
static Downshift_States_t lastDownshiftState;
static U32 lastShiftingChangeTick = 0;
static U32 lastPinChangeTick = 0;

// the CAN callback function used in this example
static void change_led_state(U8 sender, void* UNUSED_LOCAL_PARAM, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
static void init_error(void);
static void updateAndQueueParams(void);
static void run_upshift_sm(void);
static void run_downshift_sm(void);
static void check_driver_inputs(void);
static void clutch_task(U8 fastClutch, U8 slowClutch);
static void shifting_task();
static void readPinOutputs();
static void setArtificialInputs();

#define NUM_PINS 8

static U8 GPIOPin[8] = {0};
static U8 lastGPIOPin[8] = {0};

//static U8 SpkCutPinRead;
//static U8 SlowClutchPinRead;
//static U8 ClutchSolPinRead;
//static U8 DownshiftSolPinRead;
//static U8 UpshiftSolPinRead;
//static U8 Aux1CPinRead;
//static U8 Aux2CPinRead;
//static U8 Aux1TPinRead;

static void readPinOutputs() {
//	SpkCutPinRead = HAL_GPIO_ReadPin(SPK_CUT_GPIO_Port, SPK_CUT_Pin, 0);
//	SlowClutchPinRead = HAL_GPIO_ReadPin(SLOW_CLUTCH_SOL_GPIO_Port, SLOW_CLUTCH_SOL_Pin, 0);
//	ClutchSolPinRead = HAL_GPIO_ReadPin(CLUTCH_SOL_GPIO_Port, CLUTCH_SOL_Pin, 0);
//	DownshiftSolPinRead = HAL_GPIO_ReadPin(DOWNSHIFT_SOL_GPIO_Port, DOWNSHIFT_SOL_Pin, 0);
//	UpshiftSolPinRead = HAL_GPIO_ReadPin(UPSHIFT_SOL_GPIO_Port, UPSHIFT_SOL_Pin, 0);
//	Aux1CPinRead = HAL_GPIO_ReadPin(AUX1_C_GPIO_Port, AUX1_C_Pin, 0);
//	Aux2CPinRead = HAL_GPIO_ReadPin(AUX2_C_GPIO_Port, AUX2_C_Pin, 0);
//	Aux1TPinRead = HAL_GPIO_ReadPin(AUX1_T_GPIO_Port, AUX1_T_Pin, 0);

	GPIOPin[0] = HAL_GPIO_ReadPin(SPK_CUT_GPIO_Port, SPK_CUT_Pin);
	GPIOPin[1] = HAL_GPIO_ReadPin(SLOW_CLUTCH_SOL_GPIO_Port, SLOW_CLUTCH_SOL_Pin);
	GPIOPin[2] = HAL_GPIO_ReadPin(CLUTCH_SOL_GPIO_Port, CLUTCH_SOL_Pin);
	GPIOPin[3] = HAL_GPIO_ReadPin(DOWNSHIFT_SOL_GPIO_Port, DOWNSHIFT_SOL_Pin);
	GPIOPin[4] = HAL_GPIO_ReadPin(UPSHIFT_SOL_GPIO_Port, UPSHIFT_SOL_Pin);
	GPIOPin[5] = HAL_GPIO_ReadPin(AUX1_C_GPIO_Port, AUX1_C_Pin);
	GPIOPin[6] = HAL_GPIO_ReadPin(AUX2_C_GPIO_Port, AUX2_C_Pin);
	GPIOPin[7] = HAL_GPIO_ReadPin(AUX1_T_GPIO_Port, AUX1_T_Pin);

	for(int i = 0; i < NUM_PINS; i++) {
		if(GPIOPin[i] != lastGPIOPin[i]) {
			char pinName[20];

			switch(i) {
				case 1:
					strncpy(pinName, "SPK_CUT", 20*sizeof(char));
					break;
				case 2:
					strncpy(pinName, "SLOW_CLUTCH_SOL", 20*sizeof(char));
					break;
				case 3:
					strncpy(pinName, "DOWNSHIFT_SOL", 20*sizeof(char));
					break;
				case 4:
					strncpy(pinName, "UPSHIFT_SOL", 20*sizeof(char));
					break;
				case 5:
					strncpy(pinName, "AUX1_C", 20*sizeof(char));
					break;
				case 6:
					strncpy(pinName, "AUX2_C", 20*sizeof(char));
					break;
				case 7:
					strncpy(pinName, "AUX1_T", 20*sizeof(char));
					break;
				default:
					break;
			}

			printf("%s Toggled: %u\n", pinName, GPIOPin[i]);
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance From Last Pin Change: %lu\n", HAL_GetTick() - lastPinChangeTick);
			lastPinChangeTick = HAL_GetTick();
		}
		lastGPIOPin[i] = GPIOPin[i];
	}
}

static void setArtificialInputs() {

}



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
	static U32 lastHeartbeat = 0;
	static U32 lastPrintHB = 0;
	static U32 lastTCMDataUpdate = 0;
	static uint32_t last_gear_update = 0;

	if (HAL_GetTick() - lastHeartbeat > HEARTBEAT_MS_BETWEEN)
	{
		lastHeartbeat = HAL_GetTick();
		HAL_GPIO_TogglePin(HBEAT_GPIO_Port, HBEAT_Pin);
	}

	if (HAL_GetTick() - last_gear_update >= GEAR_UPDATE_TIME_MS)
	{
		last_gear_update = HAL_GetTick();
		tcm_data.current_gear = get_current_gear(gearPosition_mm.data);
	}

	updateAndQueueParams();
	check_driver_inputs();
	shifting_task();

	// send the current tick over UART every second
	if (HAL_GetTick() - lastPrintHB >= PRINTF_HB_MS_BETWEEN)
	{
		printf("Current tick: %lu\n", HAL_GetTick());
		lastPrintHB = HAL_GetTick();
	}

	if (HAL_GetTick() - lastTCMDataUpdate >= TCM_DATA_UPDATE_MS_BETWEEN) {
		// Update shift struct with relevant data
		update_tcm_data();
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
//	update_and_queue_param_u8(&tcmTimeShiftOnly_state, tcm_data.time_shift_only);
//	update_and_queue_param_u8(&tcmClutchlessDownshift_state, tcm_data.clutchless_downshift);

	switch (main_State)
	{
	default:
	case ST_IDLE:
		// not shifting, send 0
		update_and_queue_param_u8(&tcmShiftState_state, 0);
		break;

	case ST_HDL_UPSHIFT:
		// send the upshift state
		update_and_queue_param_u8(&tcmShiftState_state, upshift_State);
		break;

	case ST_HDL_DOWNSHIFT:
		// send the downshift state
		update_and_queue_param_u8(&tcmShiftState_state, downshift_State);
		break;
	}

//	if(upshift_State != lastUpshiftState) {
//		printf("=== UPSHIFT STATE CHANGE ===");
//
//	}
}

static void check_driver_inputs() {
	if(swButon0_state.data) {
		tcm_data.time_shift_only = !tcm_data.time_shift_only;
	}

	if(swButon1_state.data) {
		tcm_data.clutchless_downshift = !tcm_data.clutchless_downshift;
	}

	// Check if clutch buttons are pressed and then run clutch task, benefit of being able to be skipped over if used for special input sequences
	if(swFastClutch_state.data || swSlowClutch_state.data) {
		clutch_task(swFastClutch_state.data, swSlowClutch_state.data);
	}
}

static void shifting_task() {
	switch (main_State)
	{
	case ST_IDLE:
		// in idle state, make sure we are not spark cutting and not pushing
		// the solenoid
		set_spark_cut(false);

		// NOTE: potentially use the opposite solenoid to push the shift lever
		// back into a neutral position quicker. This would use more air but
		// would allow us to shift again much sooner. We would need to make
		// sure the shift position is valid before doing this to prevent
		// always pushing the shift lever into a shifting position during
		// idle state

		// TODO WARNING this will mean shifting will not work if the shift
		// pot gets disconnected as the lever will push one way
		if (get_shift_pot_pos() > LEVER_NEUTRAL_POS_MM + LEVER_NEUTRAL_TOLERANCE)
		{
			// shifter pos is too high. Bring it back down
			set_upshift_solenoid(SOLENOID_OFF);
			set_downshift_solenoid(SOLENOID_ON);
		}
		else if (get_shift_pot_pos() < LEVER_NEUTRAL_POS_MM - LEVER_NEUTRAL_TOLERANCE)
		{
			// shifter pos is too low. Bring it up
			set_upshift_solenoid(SOLENOID_ON);
			set_downshift_solenoid(SOLENOID_OFF);
		}
		else
		{
			// we good. Levers off
			set_upshift_solenoid(SOLENOID_OFF);
			set_downshift_solenoid(SOLENOID_OFF);
		}

		// start a downshift if there is one pending. This means that a new
		// shift can be queued during the last shift
		if (swDownshift_state.data)
		{
			swDownshift_state.data = 0;
			// TODO: Re-implement
//			if (calc_validate_downshift(car_shift_data.current_gear, sw_clutch_fast, sw_clutch_slow))
//			{
				main_State = ST_HDL_DOWNSHIFT;
				downshift_State = ST_D_BEGIN_SHIFT;
//			}
		}

		// same for upshift. Another shift can be queued
		if (swUpshift_state.data)
		{
			swUpshift_state.data = 0;
			// TODO: Re-implement
//			if (calc_validate_upshift(car_shift_data.current_gear, sw_clutch_fast, sw_clutch_slow))
//			{
				main_State = ST_HDL_UPSHIFT;
				upshift_State = ST_U_BEGIN_SHIFT;
//			}
		}
		break;

	case ST_HDL_UPSHIFT:
		run_upshift_sm();
		break;

	case ST_HDL_DOWNSHIFT:
		run_downshift_sm();
		break;
	}
}

// clutch_task
//  for use in the main task. Sets the slow and fast drop accordingly and handles
//  clutch position if in ST_IDLE
static void clutch_task(U8 fastClutch, U8 slowClutch) {
	static bool using_slow_drop = false;
	// normal clutch button must not be pressed when using slow drop. Fast drop is
	// given priority
	if (fastClutch) using_slow_drop = false;

	// if the slow drop button is pressed latch the slow drop
	else if (slowClutch) using_slow_drop = true;

	// If either clutch button pressed then enable solenoid. Always turn it on regardless of
	// if we are shifting or not
	if (fastClutch || slowClutch)
	{
		set_clutch_solenoid(SOLENOID_ON);
		return;
	}

	// If neither clutch button pressed and we are in IDLE and not in anti stall
	// close clutch solenoid. This will cause clutch presses to latch to the end
	// of a shift
	if (!(fastClutch || slowClutch)
			&& main_State == ST_IDLE && tcm_data.anti_stall)
	{
		set_clutch_solenoid(SOLENOID_OFF);

		// if we are using slow drop enable or disable the slow release valve depending
		// on if we are near the bite point
		if (using_slow_drop)
		{
			// when slow dropping, we want to start by fast dropping until the bite point
			if (get_clutch_pot_pos() < CLUTCH_OPEN_POS_MM - CLUTCH_SLOW_DROP_FAST_TO_SLOW_EXTRA_MM)
			{
				set_slow_clutch_drop(false);
			}
			else
			{
				set_slow_clutch_drop(true);
			}
		}
		else
		{
			// not using slow drop
			set_slow_clutch_drop(false);
		}
	}
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

// run_upshift_sm
//  The big boi upshift state machine
static void run_upshift_sm(void)
{
	static uint32_t initial_gear;
	static uint32_t begin_shift_tick;
	static uint32_t begin_exit_gear_tick;
	static uint32_t begin_enter_gear_tick;
	static uint32_t begin_exit_gear_spark_return_tick;
	static uint32_t finish_shift_start_tick;
	static uint32_t spark_cut_start_time;

	// calculate the target RPM at the start of each cycle through the loop
	tcm_data.target_RPM = calc_target_RPM(tcm_data.target_gear);

	switch (upshift_State)
	{
	case ST_U_BEGIN_SHIFT:
		// at the beginning of the upshift, start pushing on the shift lever
		// to "preload". This means that there will be force on the shift
		// lever when the spark is cut and the gear can immediately disengage
		// NOTE: We really dont care about what the clutch is doing in an
		// upshift

		begin_shift_tick = HAL_GetTick(); // Log that first begin shift tick
		initial_gear = get_current_gear(main_State);
		set_upshift_solenoid(SOLENOID_ON); // start pushing upshift

		// reset information about the shift
		tcm_data.successful_shift = true;

		// move on to waiting for the "preload" time to end
		upshift_State = ST_U_LOAD_SHIFT_LVR;

		// Debug
		printf("=== Upshift State: LOAD_SHIFT_LVR\n");
		printf("How: Completed begin shift steps\n");
		printf("Current Tick: %lu\n", HAL_GetTick());
		printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
		lastShiftingChangeTick = HAL_GetTick();
		break;

	case ST_U_LOAD_SHIFT_LVR:
		// we want to push on the solenoid, but not spark cut to preload the shift lever
		set_upshift_solenoid(SOLENOID_ON);
		safe_spark_cut(false);

		// wait for the preload time to be over
		if (HAL_GetTick() - begin_shift_tick > UPSHIFT_SHIFT_LEVER_PRELOAD_TIME_MS)
		{
			// preload time is over. Start spark cutting to disengage the
			// gears and moving the RPM to match up with the next gearS
			spark_cut_start_time = HAL_GetTick(); // Begin timer for making sure we shift for a minimum amount of time
			begin_exit_gear_tick = HAL_GetTick();
			safe_spark_cut(true);

			// move on to waiting to exit gear
			upshift_State = ST_U_EXIT_GEAR;

			// Debug
			printf("=== Upshift State: EXIT_GEAR\n");
			printf("How: Preload time completed\n");
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
			lastShiftingChangeTick = HAL_GetTick();
		}
		break;

	case ST_U_EXIT_GEAR:
		// wait until the threshold for the lever leaving the last gear has
		// been met or the timeout hits. During this time spark should be cut
		// to change engine driving the wheels to the wheels driving the
		// engine. If all goes well, the gear should be left at that midpoint
		set_upshift_solenoid(SOLENOID_ON);

		// right now just leave spark cut on for this section of code instead
		// of trying to actively rev match. Rev matching should be cutting
		// spark anyway as we have not left the lower gear yet
		//reach_target_RPM_spark_cut(tcm_data.target_RPM);
		safe_spark_cut(true);

		// check which shift mode we're in (to determine whether sensors
		// assisted shifting should be used)
		if (!tcm_data.time_shift_only)
		{
			// wait for the shift lever to reach the leaving threshold
			if (get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM)
			{
				// the shifter position is above the threshold to exit. The
				// transmission is now in a false neutral position
				upshift_State = ST_U_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();
				break;
			}

			// the shift lever has not moved far enough. Check if it has been
			// long enough to timeout yet
			if (HAL_GetTick() - begin_exit_gear_tick > UPSHIFT_EXIT_TIMEOUT_MS)
			{
				// the shift lever did not exit the previous gear. Attempt to
				// return spark to attempt to disengage
				begin_exit_gear_spark_return_tick = HAL_GetTick();
				safe_spark_cut(false);
				upshift_State = ST_U_SPARK_RETURN;

				// Debug
				printf("=== Upshift State: SPARK_RETURN\n");
				printf("How: Shift lever movement timeout passed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
			}
		}
		else
		{
			if (HAL_GetTick() - begin_exit_gear_tick > UPSHIFT_EXIT_GEAR_TIME_MS) {
				upshift_State = ST_U_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

				// Debug
				printf("=== Time Shit Only Upshift State: ENTER_GEAR");
				printf("How: Exit gear time completed");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
			}
		}
		break;

	// Unused in time based shifting TODO: Check if this should be the case
	case ST_U_SPARK_RETURN:
		// A common cause of a failed disengagement is the transition from
		// the engine driving the wheels to the wheels driving the engine
		// too quickly, meaning there is no way to exit the gear while
		// the spark is cut. In order to leave the gear we must return spark
		// briefly
		set_upshift_solenoid(SOLENOID_ON);
		safe_spark_cut(true);

		// the shifter has moved above the threshold of exiting the gear. Spark
		// must be cut again to reach the correct RPM for the next gear. If
		// enough time has passed return spark anyway
		if (get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM ||
				HAL_GetTick() - begin_exit_gear_spark_return_tick > UPSHIFT_EXIT_SPARK_RETURN_MS)
		{
			// Debug
			printf("=== Upshift State: ENTER_GEAR");
			if(get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM) {
				printf("How: Shifter moved above threshold of exiting");
			} else {
				printf("How: Spark return timed out");
			}
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
			lastShiftingChangeTick = HAL_GetTick();

			// If spark return successfully releases then continue onto the
			// next phase of the shift. If it was not successful (timeout) move
			// on anyway
			safe_spark_cut(true);
			upshift_State = ST_U_ENTER_GEAR;
			begin_enter_gear_tick = HAL_GetTick();

		}
		break;

	case ST_U_ENTER_GEAR:
		// we are in a false neutral and waiting to enter the gear. Likely
		// the RPMs will need to drop a little more to make it to the next
		// gear
		set_upshift_solenoid(SOLENOID_ON);

		// right now the code is always spark cutting. This section it is
		// less desirable to always spark cut because the revs may drop below
		// what they need to be, but it is ok as they will come back up
		// when this section is over
		//reach_target_RPM_spark_cut(tcm_data.target_RPM);
		safe_spark_cut(true);

		if (!tcm_data.time_shift_only)
		{
			// check if the shifter position is above the threshold to complete
			// a shift
			if (get_shift_pot_pos() > UPSHIFT_ENTER_POS_MM)
			{
				// shift position says we are done shifting
				upshift_State = ST_U_FINISH_SHIFT;
				finish_shift_start_tick = HAL_GetTick();

				// Debug
				printf("=== Upshift State: FINISH_SHIFT");
				printf("How: Shifter position moved past threshold");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
				break;
			}

			// check if we are out of time for this state
			if (HAL_GetTick() - begin_enter_gear_tick > UPSHIFT_ENTER_TIMEOUT_MS)
			{
				// at this point the shift was probably not successful. Note this so we
				// dont increment the gears and move on
				tcm_data.successful_shift = false;
				upshift_State = ST_U_FINISH_SHIFT;
				finish_shift_start_tick = HAL_GetTick();

				// Debug
				printf("=== Upshift State: FINISH_SHIFT");
				printf("How: Enter Gear timed out");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
			}
		}
		else
		{
			if (HAL_GetTick() - begin_enter_gear_tick > UPSHIFT_ENTER_GEAR_TIME_MS)
			{
				upshift_State = ST_U_FINISH_SHIFT;
				finish_shift_start_tick = HAL_GetTick();

				// Debug
				printf("=== Timed Shift Only Upshift State: FINISH_SHIFT");
				printf("How: Enter Gear time completed");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
			}
		}
		break;

	case ST_U_FINISH_SHIFT:
		// wrapping up the upshift. First make sure we have been been cutting
		// spark for long enough (to prevent a case where the shifter position
		// is stuck in the success region from stopping shifting), then work
		// on ending the shift while still pushing on the shift lever
		set_upshift_solenoid(SOLENOID_ON);

		// make sure enough time has passed from the beginning of the shift
		if (HAL_GetTick() - spark_cut_start_time < UPSHIFT_MIN_TIME)
		{
			// not enough time has passed. Keep spark cutting for a bit longer
			safe_spark_cut(true);
			break;
		}
		else
		{
			// enough time has passed to finish the shift. Call the shift
			// over but keep pushing as there is no downside other than the
			// extra time the shift lever will take to return, preventing from
			// starting another shift
			safe_spark_cut(false);
			tcm_data.using_clutch = false;

			if (!tcm_data.time_shift_only)
			{
				if (tcm_data.successful_shift)
				{
					tcm_data.current_gear = tcm_data.target_gear;
				}
				else
				{
					tcm_data.current_gear = ERROR_GEAR;
					tcm_data.gear_established = false;
				}
			}
			// if time based shifting and the gear didn't correctly increase,
			// declare unsuccessful
			else
			{
				tcm_data.current_gear = get_current_gear(main_State);
				if (!(tcm_data.current_gear > initial_gear))
				{
					tcm_data.successful_shift = false;
					tcm_data.gear_established = false;
				}
			}

			// check if we can disable the solenoid and return to to idle
			if (HAL_GetTick() - finish_shift_start_tick >= UPSHIFT_EXTRA_PUSH_TIME)
			{
				// done with the upshift state machine
				set_upshift_solenoid(SOLENOID_OFF);
				main_State = ST_IDLE;

				// Debug
				printf("=== Main State: IDLE\n");
				printf("How: Extra time passed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
			}
		}
		break;
	}
}

// run_downshift_sm
//  The big boi downshift state machine
static void run_downshift_sm(void)
{
	static uint32_t initial_gear;
	static uint32_t begin_shift_tick;
	static uint32_t begin_exit_gear_tick;
	static uint32_t begin_enter_gear_tick;
	static uint32_t begin_hold_clutch_tick;
	static uint32_t finish_shift_start_tick;

	// calculate the target rpm at the start of each cycle
	tcm_data.target_RPM = calc_target_RPM(tcm_data.target_gear);

	switch (downshift_State)
	{
	case ST_D_BEGIN_SHIFT:
		// at the beginning of a shift reset all of the variables and start
		// pushing on the clutch and downshift solenoid. Loading the shift
		// lever does not seem to be as important for downshifts, but still
		// give some time for it
		begin_shift_tick = HAL_GetTick();
		initial_gear = get_current_gear(main_State);

		set_downshift_solenoid(SOLENOID_ON);

		tcm_data.using_clutch = !tcm_data.clutchless_downshift; // EXPIREMENTAL: Uncomment the next line to only clutch during a downshift if the clutch is held during the start of the shift
		//tcm_data.using_clutch = (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button);

		set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);

		// reset the shift parameters
		tcm_data.successful_shift = true; // TODO figure out what happens with this during time based

		// move on to loading the shift lever
		downshift_State = ST_D_LOAD_SHIFT_LVR;
		
		// Debug
		printf("=== Downshift State: LOAD_SHIFT_LVR\n");
		printf("How: Completed begin shift steps\n");
		printf("Current Tick: %lu\n", HAL_GetTick());
		printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
		lastShiftingChangeTick = HAL_GetTick();
		break;

	case ST_D_LOAD_SHIFT_LVR:
		// load the shift lever. This is less important as we do not seem to
		// have issues leaving the gear during a downshift
		set_downshift_solenoid(SOLENOID_ON);
		set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);

		// EXPIREMENTAL: spark cut during this preload time and tell drivers
		// to blip when they start the shift. This will allow drivers to worry
		// less about timing their blips perfectly because the TCM will do it
		safe_spark_cut(true);

		if ((HAL_GetTick() - begin_shift_tick > DOWNSHIFT_SHIFT_LEVER_PRELOAD_TIME_MS))
		{
			// done with preloading. Start allowing blips and move on to trying
			// to exit the gear
			safe_spark_cut(false);
			begin_exit_gear_tick = HAL_GetTick();
			downshift_State = ST_D_EXIT_GEAR;

			// Debug
			printf("=== Downshift State: EXIT_GEAR\n");
			printf("How: Preloading time completed\n");
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
			lastShiftingChangeTick = HAL_GetTick();
		}
		break;

	case ST_D_EXIT_GEAR:
		// this is the region to blip in to leave the gear. Usually we dont
		// have much of an issue leaving. Dont spark cut under any circumstances
		// as we need the blip to bring the RPM up if we have not left the gear
		// yet
		set_downshift_solenoid(SOLENOID_ON);
		set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);
		safe_spark_cut(false);

		if (!tcm_data.time_shift_only)
		{
			// wait for the shift lever to be below the downshift exit threshold
			if (get_shift_pot_pos() < DOWNSHIFT_EXIT_POS_MM)
			{
				// we have left the last gear and are in a false neutral. Move on to
				// the next part of the shift
				downshift_State = ST_D_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();


				// Debug
				printf("=== Downshift State: ENTER_GEAR\n");
				printf("Shift Lever below Downshift Exit Threshold\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
				break;
			}

			// check if this state has timed out
			if (HAL_GetTick() - begin_exit_gear_tick > DOWNSHIFT_EXIT_TIMEOUT_MS)
			{
				// We could not release the gear for some reason. Keep trying anyway
				downshift_State = ST_D_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

				// if we were not using the clutch before, start using it now because
				// otherwise we're probably going to fail the shift
				tcm_data.using_clutch = true;

				// Debug
				printf("=== Downshift State: ENTER_GEAR\n");
				printf("How: Last State Timeout - Use Clutch\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
			}
		}
		else
		{
			if (HAL_GetTick() - begin_exit_gear_tick > DOWNSHIFT_EXIT_GEAR_TIME_MS) {
				downshift_State = ST_D_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

				// Debug
				printf("=== Time Shift Only Downshift State: ENTER_GEAR\n");
				printf("How: Exit Gear time completed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
			}
		}
		break;

	case ST_D_ENTER_GEAR:
		// now we are in a false neutral position we want to keep pushing the
		// shift solenoid, but now dynamically spark cut if the blip is too big
		// and the RPM goes too high to enter the next gear
		set_downshift_solenoid(SOLENOID_ON);
		set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);
		//reach_target_RPM_spark_cut(tcm_data.target_RPM); // TODO

		if (!tcm_data.time_shift_only)
		{
			// check if the shift position has moved enough to consider the shift
			// finished
			if (get_shift_pot_pos() < DOWNSHIFT_ENTER_POS_MM)
			{
				// the clutch lever has moved enough to finish the shift. Turn off
				// any spark cutting and move on to finishing the shift
				safe_spark_cut(false);
				downshift_State = ST_D_FINISH_SHIFT;
				finish_shift_start_tick = HAL_GetTick();

				// Debug
				printf("=== Downshift State: FINISH_SHIFT\n");
				printf("How: Enter gear shift pot moved far enough\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
				break;
			}

			// check for a timeout entering the gear
			if (HAL_GetTick() - begin_enter_gear_tick > DOWNSHIFT_ENTER_TIMEOUT_MS)
			{
				// the shift failed to enter the gear. We want to keep the clutch
				// open for some extra time to try and give the driver the chance
				// to rev around and find a gear. Call this shift a failure
				tcm_data.using_clutch = true;
				set_clutch_solenoid(SOLENOID_ON);
				safe_spark_cut(false);
				tcm_data.successful_shift = false;
				downshift_State = ST_D_HOLD_CLUTCH;
				begin_hold_clutch_tick = HAL_GetTick();

				// Debug
				printf("=== Downshift State: HOLD_CLUTCH\n");
				printf("How: Waiting for shift pot timed out\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
			}
		}
		else
		{
			if (HAL_GetTick() - begin_enter_gear_tick > DOWNSHIFT_ENTER_GEAR_TIME_MS) {
				downshift_State = ST_D_FINISH_SHIFT;
				finish_shift_start_tick = HAL_GetTick();

				// Debug
				printf("=== Time Shift Only Downshift State: FINISH_SHIFT\n");
				printf("How: Enter Gear time completed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
			}
		}
		break;

	case ST_D_HOLD_CLUTCH:
		// some extra time to hold the clutch open. This is in the case that
		// the shift lever does not hit the threshold and might needs some
		// input from the driver to hit the right revs
		set_clutch_solenoid(SOLENOID_ON);
		set_downshift_solenoid(SOLENOID_ON);
		safe_spark_cut(false);

		// check if we are done giving the extra time
		if (HAL_GetTick() - begin_hold_clutch_tick > DOWNSHIFT_FAIL_EXTRA_CLUTCH_HOLD)
		{
			// done giving the extra clutch. Move on to finishing the shift
			downshift_State = ST_D_FINISH_SHIFT;
			finish_shift_start_tick = HAL_GetTick();

			// Debug
			printf("=== Downshift State: FINISH_SHIFT\n");
			printf("How: Hold Clutch time completed\n");
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
			lastShiftingChangeTick = HAL_GetTick();
		}
		break;

	case ST_D_FINISH_SHIFT:
		// winding down the downshift. Make sure enough time has passed with
		// the clutch open if we are using the clutch, otherwise end the shift
		// but keep pushing on the shift solenoid for a little bit longer to
		// ensure the shift completes
		set_downshift_solenoid(SOLENOID_ON);

		// check if we have been in the shift for long enough. This is
		// to prevent a failure mode where the shifter is inaccurate and in
		// a position that makes it seem like the shift finished right
		// away
		if (HAL_GetTick() - begin_shift_tick < DOWNSHIFT_MIN_SHIFT_TIME)
		{
			// keep the shift going, clutch included if that is needed. No spark
			// cut though, we want to give drivers control of the shift
			set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);
			safe_spark_cut(false);
			break;
		}
		else
		{
			// enough time has passed to finish the shift. Call the shift
			// over but keep pushing as there is no downside other than the
			// extra time the shift lever will take to return, preventing from
			// starting another shift
			safe_spark_cut(false);
			set_clutch_solenoid(SOLENOID_OFF);
			tcm_data.using_clutch = false;

			if (!tcm_data.time_shift_only)
			{
				if (tcm_data.successful_shift)
				{
					tcm_data.current_gear = tcm_data.target_gear;
				}
				else
				{
					tcm_data.current_gear = ERROR_GEAR;
					tcm_data.gear_established = false;
				}

				// check if we can disable the solenoid and return to to idle
				if (HAL_GetTick() - finish_shift_start_tick >= DOWNSHIFT_EXTRA_PUSH_TIME)
				{
					// done with the downshift state machine
					set_upshift_solenoid(SOLENOID_OFF);
					main_State = ST_IDLE;

					// Debug
					printf("=== Main State: Idle\n");
					printf("How: Finish Shift extra push time completed\n");
					printf("Current Tick: %lu\n", HAL_GetTick());
					printf("Distance from Last Occurence: %lu\n", HAL_GetTick() - lastShiftingChangeTick);
					lastShiftingChangeTick = HAL_GetTick();
				}
			}
			else
			{
				tcm_data.current_gear = get_current_gear(main_State);
				if (!(tcm_data.current_gear < initial_gear))
				{
					tcm_data.successful_shift = false;
					tcm_data.gear_established = false;
				}
			}

		}
		break;
	}
}

// end of GopherCAN_example.c
