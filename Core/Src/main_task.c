// GopherCAN_devboard_example.c
//  This is a bare-bones module file that can be used in order to make a module main file
//TODO - Fix clutch activating upshift bug

#include "main_task.h"
#include "main.h"
#include "gopher_sense.h"
#include "pulse_sensor.h"
#include "utils.h"
#include <stdio.h>
#include <stdbool.h>
#include "shift_parameters.h"
#include <string.h>
#include <stdbool.h>
#include <cmsis_os.h>

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;

// some global variables for examples
Main_States_t main_state = ST_IDLE;

static Upshift_States_t upshift_state, next_upshift_state;
static Downshift_States_t downshift_state, next_downshift_state;
U32 initialization_start_time_ms = 0;
U8 last_button_state = 0;
U8 error_byte = 0;
bool initial_input_skip = true;

#ifdef SHIFT_DEBUG
// All upper shifting debug statements
static U32 lastShiftingChangeTick = 0;
#endif
// Array for the amount of time each error should latch the
U16 error_led_on_times[] = {
	OVERCURRENT_FAULT_LED_MS, // SENSE_OUT_OVERCURRENT_3V3
	OVERCURRENT_FAULT_LED_MS, // SENSE_OUT_OVERCURRENT_5V
	SHIFT_SAFETY_FAULT_LED_MS,	//
	SHIFT_SAFETY_FAULT_LED_MS,	//
	SHIFT_TIMEOUT_FAULT_LED_MS,
	SHIFT_TIMEOUT_FAULT_LED_MS,
	0,
	0
};

// the CAN callback function used in this example
static void change_led_state(U8 sender, void* UNUSED_LOCAL_PARAM, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
static void init_error(void);
static void updateAndQueueParams(void);
static void run_upshift_sm(void);
static void run_downshift_sm(void);
static void check_driver_inputs(void);
static void clutch_task();
static void shifting_task();
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

	initialization_start_time_ms = HAL_GetTick();

	// Lock param sending for everything - uncomment for stress testing CAN
//	lock_param_sending(&counterShaftSpeed_rpm.info);
//	lock_param_sending(&tcmTargetRPM_rpm.info);
//	lock_param_sending(&tcmCurrentGear_state.info);
//	lock_param_sending(&tcmCurrentlyMoving_state.info);
//	lock_param_sending(&tcmAntiStallActive_state.info);
//	lock_param_sending(&tcmError_state.info);
//	lock_param_sending(&tcmUsingClutch_state.info);
//	lock_param_sending(&tcmUsingClutch_state.info);

	lock_param_sending(&tcmTimeShiftOnly_state.info);
#ifdef CAN_CLUTCHLESS_DOWNSHIFT
	lock_param_sending(&tcmClutchlessDownshift_state.info);
#endif

	tcm_data.time_shift_only = true;
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
#ifdef NO_GEAR_POT
	static uint32_t last_gear_update = 0;
#endif

#ifdef RUN_TIME_STATS
	static char taskBuffer[250];
#endif
	static uint32_t lastTaskUtilizationUpdate = 0;
	if (HAL_GetTick() -  lastTaskUtilizationUpdate > 1000)
	{
		lastTaskUtilizationUpdate = HAL_GetTick();

#ifdef RUN_TIME_STATS
		// TESTING: Get runtime stats
		vTaskGetRunTimeStats(taskBuffer);
		printf("%s\n", taskBuffer);
#endif
	}

	if (HAL_GetTick() - lastHeartbeat > HEARTBEAT_MS_BETWEEN)
	{
		lastHeartbeat = HAL_GetTick();
		HAL_GPIO_TogglePin(HBEAT_GPIO_Port, HBEAT_Pin);
	}

#ifdef NO_GEAR_POT
	if (HAL_GetTick() - last_gear_update >= GEAR_UPDATE_TIME_MS)
	{
		last_gear_update = HAL_GetTick();
		tcm_data.current_gear = get_current_gear();
	}
#else
	tcm_data.current_gear = get_current_gear();
#endif

	check_pulse_sensors();
	update_tcm_data();

	if (!initial_input_skip) {
		check_driver_inputs();
	} else if (HAL_GetTick() - initialization_start_time_ms > 2000) {
		initial_input_skip = false;
	}

	shifting_task();
	clutch_task();

	// send the current tick over UART every second
	if (HAL_GetTick() - lastPrintHB >= PRINTF_HB_MS_BETWEEN)
	{
		printf("Current tick: %lu\n", HAL_GetTick());
		lastPrintHB = HAL_GetTick();
	}

	checkForErrors();
	updateAndQueueParams();

	// Clear the error byte, so it has to keep being triggered if the error is persistent (and doesn't require a function to turn it off again)
	error_byte = 0;
}

static void checkForErrors(void) {
	static U32 led_on_start_time = 0;
	static U32 time_on_ms = 0;
	static bool led_on = false;

	// Can expand into for loop if more pins to read from in the future, but for now just check the 2 fault input pins.
	if (!HAL_GPIO_ReadPin(SWITCH_FAULT_3V3_GPIO_Port, SWITCH_FAULT_3V3_Pin)) {
		error(SENSE_OUT_OVERCURRENT_3V3, &error_byte);
	}

	if (!HAL_GPIO_ReadPin(SWITCH_FAULT_5V_GPIO_Port, SWITCH_FAULT_5V_Pin)) {
		error(SENSE_OUT_OVERCURRENT_5V, &error_byte);
	}

	if (error_byte > 0 && !led_on) {
		U8 index = 0;
		// Get the index of the lowest bit. Cannot become infinite because we're only here if error_byte >0
		while (!(error_byte & (1 << index))) index++;

		time_on_ms = error_led_on_times[index];

		led_on_start_time = HAL_GetTick();
		led_on = true;
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

static float shift_mode_2 = 0;

// Updates gcan variables
static void updateAndQueueParams(void) {
	update_and_queue_param_float(&counterShaftSpeed_rpm, tcm_data.trans_speed);
	update_and_queue_param_float(&tcmCurrentGear_state, tcm_data.current_gear);
	update_and_queue_param_u16(&tcmTargetRPM_rpm, tcm_data.target_RPM);
	update_and_queue_param_u8(&tcmCurrentlyMoving_state, tcm_data.currently_moving);
	update_and_queue_param_u8(&tcmAntiStallActive_state, tcm_data.anti_stall);
	update_and_queue_param_u8(&tcmUsingClutch_state, tcm_data.using_clutch);
	update_and_queue_param_u8(&tcmTimeShiftOnly_state, tcm_data.time_shift_only);
	update_and_queue_param_u8(&tcmClutchlessDownshift_state, shift_mode_2); // tcm_data.clutchless_downshift);
	update_and_queue_param_u8(&tcmTargetGear_state, tcm_data.target_gear);
	update_and_queue_param_u8(&tcmSpkCut_state, tcm_data.spark_cut);
	update_and_queue_param_u8(&tcmClutchlessDownshift_state, tcm_data.clutchless_downshift);
	update_and_queue_param_u8(&tcmSuccessfulShift_state, tcm_data.successful_shift);
	update_and_queue_param_u8(&tcmError_state, error_byte);
	update_and_queue_param_u16(&tcmNumShifts_ul, tcm_data.num_shifts);
	update_and_queue_param_u16(&tcmNumSuccessfulShifts_ul, tcm_data.num_successful_shifts);

	switch (main_state)
	{
	default:
	case ST_IDLE:
		// not shifting, send 0
		update_and_queue_param_u8(&tcmShiftState_state, 0);
		break;

	case ST_HDL_UPSHIFT:
		// send the upshift state
		update_and_queue_param_u8(&tcmShiftState_state, upshift_state);
		break;

	case ST_HDL_DOWNSHIFT:
		// send the downshift state
		update_and_queue_param_u8(&tcmShiftState_state, downshift_state);
		break;
	}
}

static float last_upshift_button = 0;
static float last_downshift_button = 0;

#ifdef CAN_CHANGE_FROM_TIME_SHIFT
static float last_timeShiftOnly_button = 0;
#endif

#ifdef CAN_CLUTCHLESS_DOWNSHIFT
static float last_clutchlessDownshift_button = 0;
#endif

static void check_driver_inputs() {
	tcm_data.sw_fast_clutch = FAST_CLUTCH_BUTTON;
	tcm_data.sw_slow_clutch = SLOW_CLUTCH_BUTTON;

#ifdef CAN_CHANGE_FROM_TIME_SHIFT
	if((last_timeShiftOnly_button == 0) && (TIME_SHIFT_ONLY_BUTTON == 1)) {
		tcm_data.time_shift_only = !tcm_data.time_shift_only;
	}
	last_timeShiftOnly_button = TIME_SHIFT_ONLY_BUTTON;
#endif

#ifdef CAN_CLUTCHLESS_DOWNSHIFT
	if((last_clutchlessDownshift_button == 0) && (CLUTCHLESS_DOWNSHIFT_BUTTON == 1)) {
		// tcm_data.clutchless_downshift = !tcm_data.clutchless_downshift; // commented out for shift testing and instead used for 2nd shift parameter mode
		shift_mode_2 = !shift_mode_2;
	}
	last_clutchlessDownshift_button = CLUTCHLESS_DOWNSHIFT_BUTTON;
#endif

	// Check button was released before trying shifting again - falling edge
	if ((last_upshift_button == 0) && (UPSHIFT_BUTTON == 1)) {
		if (tcm_data.pending_shift == DOWNSHIFT) {
			tcm_data.pending_shift = NONE;
		} else {
			tcm_data.pending_shift = UPSHIFT;
		}
	}
	last_upshift_button = UPSHIFT_BUTTON;

	// Check button was released before trying shifting again - falling edge
	if ((last_downshift_button == 0) && (DOWNSHIFT_BUTTON == 1)) {
		if(tcm_data.pending_shift == UPSHIFT) {
			tcm_data.pending_shift = NONE;
		} else {
			tcm_data.pending_shift = DOWNSHIFT;
		}
	}
	last_downshift_button = DOWNSHIFT_BUTTON;
}

static void shifting_task() {
	switch (main_state)
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

#ifdef AUTO_SHIFT_LEVER_RETURN
		// TODO WARNING this will mean shifting will not work if the shift
		// pot gets disconnected as the lever will push one way
		if (!tcm_data.time_shift_only) {
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
		}
#endif

		// start a shift if there is one pending. This means that a new
		// shift can be queued during the last shift. Always allow shifts in time shifting mode
		if(tcm_data.pending_shift == UPSHIFT) {
			if (calc_validate_upshift(tcm_data.current_gear, tcm_data.sw_fast_clutch,
					                  tcm_data.sw_slow_clutch) ||
					tcm_data.time_shift_only)
			{
				main_state = ST_HDL_UPSHIFT;
				upshift_state = ST_U_BEGIN_SHIFT;
			}
		} else if (tcm_data.pending_shift == DOWNSHIFT) {
			if (calc_validate_downshift(tcm_data.current_gear, tcm_data.sw_fast_clutch,
					                    tcm_data.sw_slow_clutch) ||
					tcm_data.time_shift_only)
			{
				main_state = ST_HDL_DOWNSHIFT;
				downshift_state = ST_D_BEGIN_SHIFT;
			}
		}
		tcm_data.pending_shift = NONE;
		break;

	case ST_HDL_UPSHIFT:
		run_upshift_sm();
		upshift_state = next_upshift_state; // Set state to next state variable per Sebastion's request
		break;

	case ST_HDL_DOWNSHIFT:
		run_downshift_sm();
		downshift_state = next_downshift_state;
		break;
	}
}

// clutch_task
//  for use in the main task. Sets the slow and fast drop accordingly and handles
//  clutch position if in ST_IDLE
static void clutch_task() {
	static bool using_slow_drop = false;
	// normal clutch button must not be pressed when using slow drop. Fast drop is
	// given priority
	if (tcm_data.sw_fast_clutch) using_slow_drop = false;

	// if the slow drop button is pressed latch the slow drop
	else if (tcm_data.sw_slow_clutch) using_slow_drop = true;

	// If either clutch button pressed then enable solenoid. Always turn it on regardless of
	// if we are shifting or not
	if (tcm_data.sw_fast_clutch || tcm_data.sw_slow_clutch)
	{
		set_clutch_solenoid(SOLENOID_ON);
		return;
	} else if (main_state == ST_IDLE && !tcm_data.anti_stall) {
		// If neither clutch button pressed and we are in IDLE and not in anti stall
		// close clutch solenoid. This will cause clutch presses to latch to the end
		// of a shift
		set_clutch_solenoid(SOLENOID_OFF);

#ifdef SMART_SLOW_CLUTCH
		// if we are using slow drop enable or disable the slow release valve depending
		// on if we are near the bite point
		if (using_slow_drop)
		{
			// TODO: Check if we want a way to make this not closed loop
			// when slow dropping, we want to start by fast dropping until the bite point
			if (get_clutch_pot_pos() > (CLUTCH_OPEN_POS_MM + CLUTCH_SLOW_DROP_FAST_TO_SLOW_EXTRA_MM))
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
#else
		if (using_slow_drop) {
			set_slow_clutch_drop(true);
		} else {
			set_slow_clutch_drop(false);
		}
#endif
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
	static uint32_t begin_extra_push_tick;
	static uint32_t begin_exit_gear_spark_return_tick;

	// calculate the target RPM at the start of each cycle through the loop
	tcm_data.target_RPM = calc_target_RPM(tcm_data.target_gear);

	switch (upshift_state)
	{
	case ST_U_BEGIN_SHIFT:
		// at the beginning of the upshift, start pushing on the shift lever
		// to "preload". This means that there will be force on the shift
		// lever when the spark is cut and the gear can immediately disengage
		// NOTE: We really dont care about what the clutch is doing in an
		// upshift

		begin_shift_tick = HAL_GetTick(); // Log that first begin shift tick

#ifdef NO_GEAR_POT
		tcm_data.current_gear = get_current_gear();
#endif

		initial_gear = tcm_data.current_gear;
		tcm_data.successful_shift = false;

		set_upshift_solenoid(SOLENOID_ON); // start pushing upshift

		// move on to waiting for the "preload" time to end
		next_upshift_state = ST_U_LOAD_SHIFT_LVR;

#ifdef SHIFT_DEBUG
		// Debug
		printf("=== Upshift State: LOAD_SHIFT_LVR\n");
		printf("How: Completed begin shift steps\n");
		printf("Current Tick: %lu\n", HAL_GetTick());
		printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
		lastShiftingChangeTick = HAL_GetTick();
#endif
		break;

	case ST_U_LOAD_SHIFT_LVR:
		// we want to push on the solenoid, but not spark cut to preload the shift lever
		set_upshift_solenoid(SOLENOID_ON);
		safe_spark_cut(false);

		// wait for the preload time to be over
		if (HAL_GetTick() - begin_shift_tick > UPSHIFT_SHIFT_LEVER_PRELOAD_TIME_MS(shift_mode_2))
		{
			// preload time is over. Start spark cutting to disengage the
			// gears and moving the RPM to match up with the next gearS
			begin_exit_gear_tick = HAL_GetTick();
			safe_spark_cut(true);

			// move on to waiting to exit gear
			next_upshift_state = ST_U_EXIT_GEAR;

#ifdef SHIFT_DEBUG
			// Debug
			printf("=== Upshift State: EXIT_GEAR\n");
			printf("How: Preload time completed\n");
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
			lastShiftingChangeTick = HAL_GetTick();
#endif
		}
		break;

	case ST_U_EXIT_GEAR: // **************************************************
		// wait until the threshold for the lever leaving the last gear has
		// been met or the timeout hits. During this time spark should be cut
		// to change engine driving the wheels to the wheels driving the
		// engine. If all goes well, the gear should be left at that midpoint
		set_upshift_solenoid(SOLENOID_ON);

		safe_spark_cut(true);

		// check which shift mode we're in (to determine whether sensors
		//  should be used)
		if (!tcm_data.time_shift_only)
		{
#ifdef NO_GEAR_POT
			// wait for the shift lever to reach the leaving threshold
			if (get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM)
			{
#else
			// Check if we're now in between gears
			if (tcm_data.current_gear > initial_gear)
			{
#endif
				// the shifter position is above the threshold to exit. The
				// transmission is now in a false neutral position
				next_upshift_state = ST_U_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Upshift State: ENTER_GEAR\n");
#ifdef NO_GEAR_POT
				printf("Shift lever moved far enough\n");

#else
				printf("Gear state increased since initial signaling in between gears\n");
#endif
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
				break;
			}

			// the shift lever has not moved far enough. Check if it has been
			// long enough to timeout yet
			if (HAL_GetTick() - begin_exit_gear_tick > UPSHIFT_EXIT_TIMEOUT_MS(shift_mode_2))
			{
				// the shift lever did not exit the previous gear. Attempt to
				// return spark to attempt to disengage
				error(SHIFT_STATE_TIMEOUT, &error_byte);
				begin_exit_gear_spark_return_tick = HAL_GetTick();
				safe_spark_cut(false);
				next_upshift_state = ST_U_SPARK_RETURN;

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Upshift State: SPARK_RETURN\n");
				printf("How: Shift lever movement timeout passed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		else
		{
			if (HAL_GetTick() - begin_exit_gear_tick > UPSHIFT_EXIT_GEAR_TIME_MS(shift_mode_2)) {
				next_upshift_state = ST_U_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Time Shit Only Upshift State: ENTER_GEAR\n");
				printf("How: Exit gear time completed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		break;

	// Unused in time based shifting
	case ST_U_SPARK_RETURN: // ********************************************
		// A common cause of a failed disengagement is the transition from
		// the engine driving the wheels to the wheels driving the engine
		// too quickly, meaning there is no way to exit the gear while
		// the spark is cut. In order to leave the gear we must return spark
		// briefly
		set_upshift_solenoid(SOLENOID_ON);
		safe_spark_cut(true);

#ifdef NO_GEAR_POT
		// the shifter has moved above the threshold of exiting the gear. Spark
		// must be cut again to reach the correct RPM for the next gear. If
		// enough time has passed return spark anyway
		if (get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM
#else
		if (tcm_data.current_gear > initial_gear
#endif
			|| HAL_GetTick() - begin_exit_gear_spark_return_tick > UPSHIFT_EXIT_SPARK_RETURN_MS(shift_mode_2))
		{

#ifdef SHIFT_DEBUG
			// Debug
			printf("=== Upshift State: ENTER_GEAR\n");

#ifdef NO_GEAR_POT
			if(get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM) {
				printf("How: Shifter moved above threshold of exiting\n");
			} else {
				printf("How: Spark return timed out\n");
			}
#else
			printf("Gear state increased since initial signaling in between gears\n");
#endif
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
			lastShiftingChangeTick = HAL_GetTick();
#endif

			// If spark return successfully releases then continue onto the
			// next phase of the shift. If it was not successful (timeout) move
			// on anyway
			safe_spark_cut(true);
			next_upshift_state = ST_U_ENTER_GEAR;
			begin_enter_gear_tick = HAL_GetTick();

		}
		break;

	case ST_U_ENTER_GEAR: // *********************************************
		// we are in a false neutral and waiting to enter the gear. Likely
		// the RPMs will need to drop a little more to make it to the next
		// gear
		set_upshift_solenoid(SOLENOID_ON);

		if (!tcm_data.time_shift_only)
		{
			reach_target_RPM_spark_cut(tcm_data.target_RPM);

#ifdef NO_GEAR_POT
			// check if the shifter position is above the threshold to complete
			// a shift
			if (get_shift_pot_pos() > UPSHIFT_ENTER_POS_MM)
			{
#else
			// Check if we are in the next up gear (+2 gear states as +1 is in-between state)
			if (tcm_data.current_gear == (initial_gear + 2))
			{
#endif

				begin_extra_push_tick = HAL_GetTick();
				next_upshift_state = ST_U_EXTRA_PUSH;
				// shift position says we are done shifting
//				next_upshift_state = ST_U_FINISH_SHIFT;

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Upshift State: FINISH_SHIFT\n");
#ifdef NO_GEAR_POT
				printf("How: Shifter position moved past threshold\n");
#else
				printf("Gear has increased 2 states and now in the desired gear\n");
#endif
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
				break;
			}

			// check if we are out of time for this state
			if (HAL_GetTick() - begin_enter_gear_tick > UPSHIFT_ENTER_TIMEOUT_MS(shift_mode_2))
			{
				error(SHIFT_STATE_TIMEOUT, &error_byte);
				// at this point the shift was probably not successful. Note this so we
				// dont increment the gears and move on
				tcm_data.successful_shift = false;
				begin_extra_push_tick = HAL_GetTick();
				next_upshift_state = ST_U_EXTRA_PUSH;
//				next_upshift_state = ST_U_FINISH_SHIFT;

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Upshift State: FINISH_SHIFT\n");
				printf("How: Enter Gear timed out\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		else
		{
			// Continuous spark cutting is less desirable because the revs
			// may drop below what they need to be, but it is ok as they
			// will come back up when this section is over
			set_spark_cut(true);

			if (HAL_GetTick() - begin_enter_gear_tick > UPSHIFT_ENTER_GEAR_TIME_MS(shift_mode_2))
			{
				begin_extra_push_tick = HAL_GetTick();
				next_upshift_state = ST_U_EXTRA_PUSH;

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Timed Shift Only Upshift State: FINISH_SHIFT\n");
				printf("How: Enter Gear time completed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		break;

	case ST_U_EXTRA_PUSH:
		safe_spark_cut(false);

		set_upshift_solenoid(SOLENOID_ON);

		if (HAL_GetTick() - begin_extra_push_tick > UPSHIFT_EXTRA_PUSH_TIME_MS(shift_mode_2))
		{
			next_upshift_state = ST_U_FINISH_SHIFT;
		}
		break;

	case ST_U_FINISH_SHIFT: // **********************************************
		// The shift is over, make sure we're no longer
		// spark cutting and aren't using the clutch
		set_upshift_solenoid(SOLENOID_ON);

		safe_spark_cut(false);
		tcm_data.using_clutch = false;
		tcm_data.num_shifts += 1;

#ifdef NO_GEAR_POT
		// Only grab the current gear if we don't have a gear pot
		// because if we have one its already updating every cycle
		tcm_data.current_gear = get_current_gear();
#endif

		// Determine if the shift was successful by checking if we changed gear correctly
		tcm_data.successful_shift = tcm_data.current_gear >= initial_gear + 2;
		tcm_data.gear_established = tcm_data.successful_shift;
		if (tcm_data.successful_shift) {
			tcm_data.num_successful_shifts += 1;
		}

		// done with the upshift state machine
		set_upshift_solenoid(SOLENOID_OFF);
		main_state = ST_IDLE;

#ifdef SHIFT_DEBUG
		// Debug
		printf("=== Main State: IDLE\n");
		printf("How: Finish Shift functions completed\n");
		printf("Current Tick: %lu\n", HAL_GetTick());
		printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
		lastShiftingChangeTick = HAL_GetTick();
#endif
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
	static uint32_t begin_extra_push_time_tick;
	static uint32_t begin_hold_clutch_tick;

	// calculate the target rpm at the start of each cycle
	tcm_data.target_RPM = calc_target_RPM(tcm_data.target_gear);

	switch (downshift_state)
	{
	case ST_D_BEGIN_SHIFT: // **********************************************
		// at the beginning of a shift reset all of the variables and start
		// pushing on the clutch and downshift solenoid. Loading the shift
		// lever does not seem to be as important for downshifts, but still
		// give some time for it
		begin_shift_tick = HAL_GetTick();

#ifdef NO_GEAR_POT
		tcm_data.current_gear = get_current_gear();
#endif
		initial_gear = tcm_data.current_gear;
		tcm_data.successful_shift = false;

		set_downshift_solenoid(SOLENOID_ON);

		tcm_data.using_clutch = !tcm_data.clutchless_downshift; // EXPIREMENTAL: Uncomment the next line to only clutch during a downshift if the clutch is held during the start of the shift
		//tcm_data.using_clutch = (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button);

		set_slow_clutch_drop(SOLENOID_OFF);
		set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);

		// move on to loading the shift lever
		next_downshift_state = ST_D_LOAD_SHIFT_LVR;

#ifdef SHIFT_DEBUG
		// Debug
		printf("=== Downshift State: LOAD_SHIFT_LVR\n");
		printf("How: Completed begin shift steps\n");
		printf("Current Tick: %lu\n", HAL_GetTick());
		printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
		lastShiftingChangeTick = HAL_GetTick();
#endif
		break;

	case ST_D_LOAD_SHIFT_LVR: // **********************************************
		// load the shift lever. This is less important as we do not seem to
		// have issues leaving the gear during a downshift
		set_downshift_solenoid(SOLENOID_ON);
		set_slow_clutch_drop(SOLENOID_OFF);
		set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);

		// EXPIREMENTAL: spark cut during this preload time and tell drivers
		// to blip when they start the shift. This will allow drivers to worry
		// less about timing their blips perfectly because the TCM will do it
		safe_spark_cut(true);

		if ((HAL_GetTick() - begin_shift_tick > DOWNSHIFT_SHIFT_LEVER_PRELOAD_TIME_MS(shift_mode_2)))
		{
			// done with preloading. Start allowing blips and move on to trying
			// to exit the gear
			safe_spark_cut(false);
			begin_exit_gear_tick = HAL_GetTick();
			next_downshift_state = ST_D_EXIT_GEAR;

#ifdef SHIFT_DEBUG
			// Debug
			printf("=== Downshift State: EXIT_GEAR\n");
			printf("How: Preloading time completed\n");
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
			lastShiftingChangeTick = HAL_GetTick();
#endif
		}
		break;

	case ST_D_EXIT_GEAR: // **********************************************
		// this is the region to blip in to leave the gear. Usually we dont
		// have much of an issue leaving. Dont spark cut under any circumstances
		// as we need the blip to bring the RPM up if we have not left the gear
		// yet
		set_downshift_solenoid(SOLENOID_ON);
		set_slow_clutch_drop(SOLENOID_OFF);
		set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);
		safe_spark_cut(false);

		if (!tcm_data.time_shift_only)
		{
#ifdef NO_GEAR_POT
			// wait for the shift lever to be below the downshift exit threshold
			if (get_shift_pot_pos() < DOWNSHIFT_EXIT_POS_MM)
			{
#else
			// Check if we've moved out of the initial gear and are now in between in the correct direction
			if (tcm_data.current_gear < initial_gear)
			{
#endif
				// we have left the last gear and are in a false neutral. Move on to
				// the next part of the shift
				next_downshift_state = ST_D_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Downshift State: ENTER_GEAR\n");
#ifdef NO_GEAR_POT
				printf("Shift Lever below Downshift Exit Threshold\n");
#else
				printf("Gear state moved below initial gear\n");
#endif
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
				break;
			}

			// check if this state has timed out
			if (HAL_GetTick() - begin_exit_gear_tick > DOWNSHIFT_EXIT_TIMEOUT_MS(shift_mode_2))
			{
				error(SHIFT_STATE_TIMEOUT, &error_byte);

				// We could not release the gear for some reason. Keep trying anyway
				next_downshift_state = ST_D_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

				// if we were not using the clutch before, start using it now because
				// otherwise we're probably going to fail the shift. Don't use if we're
				// using clutchless downshift though
				tcm_data.using_clutch = !tcm_data.clutchless_downshift;

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Downshift State: ENTER_GEAR\n");
				printf("How: Last State Timeout - Use Clutch\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		else
		{
			if (HAL_GetTick() - begin_exit_gear_tick > DOWNSHIFT_EXIT_GEAR_TIME_MS(shift_mode_2)) {
				next_downshift_state = ST_D_ENTER_GEAR;
				begin_enter_gear_tick = HAL_GetTick();

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Time Shift Only Downshift State: ENTER_GEAR\n");
				printf("How: Exit Gear time completed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		break;

	case ST_D_ENTER_GEAR: // **********************************************
		// now we are in a false neutral position we want to keep pushing the
		// shift solenoid, but now dynamically spark cut if the blip is too big
		// and the RPM goes too high to enter the next gear
		set_downshift_solenoid(SOLENOID_ON);
		set_slow_clutch_drop(SOLENOID_OFF);
		set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);

		if (!tcm_data.time_shift_only)
		{
			// Spark cut to make sure the driver is reaching the right rpm while blipping
			reach_target_RPM_spark_cut(tcm_data.target_RPM);

#ifdef NO_GEAR_POT
			// check if the shift position has moved enough to consider the shift
			// finished
			if (get_shift_pot_pos() < DOWNSHIFT_ENTER_POS_MM)
			{
#else
			if (tcm_data.current_gear == (initial_gear - 2))
			{
#endif
				// the clutch lever has moved enough to finish the shift. Turn off
				// any spark cutting and move on to finishing the shift
				safe_spark_cut(false);
				begin_extra_push_time_tick = HAL_GetTick();
				next_downshift_state = ST_D_EXTRA_PUSH;

#ifdef SHIFT_DEBUG
//				// Debug
				printf("=== Downshift State: FINISH_SHIFT\n");
#ifdef NO_GEAR_POT
				printf("Shift Lever below Downshift Exit Threshold\n");
#else
				printf("Gear has decreased 2 states and now in the desired gear\n");
#endif
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
				break;
			}

			// check for a timeout entering the gear
			if (HAL_GetTick() - begin_enter_gear_tick > DOWNSHIFT_ENTER_TIMEOUT_MS(shift_mode_2))
			{
				tcm_data.successful_shift = false;
				error(SHIFT_STATE_TIMEOUT, &error_byte);

				// the shift failed to enter the gear. We want to keep the clutch
				// open for some extra time to try and give the driver the chance
				// to rev around and find a gear. Call this shift a failure
				tcm_data.using_clutch = true;
				set_slow_clutch_drop(SOLENOID_OFF);
				set_clutch_solenoid(SOLENOID_ON);
				safe_spark_cut(false);
				next_downshift_state = ST_D_HOLD_CLUTCH;
				begin_hold_clutch_tick = HAL_GetTick();

#ifdef SHIFT_DEBUG
				//Debug
				printf("=== Downshift State: HOLD_CLUTCH\n");
				printf("How: Waiting for shift pot timed out\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		else
		{
			// No spark cut without being confident in the trans speed

			if (HAL_GetTick() - begin_enter_gear_tick > DOWNSHIFT_ENTER_GEAR_TIME_MS(shift_mode_2)) {
				begin_extra_push_time_tick = HAL_GetTick();
				next_downshift_state = ST_D_EXTRA_PUSH;
				safe_spark_cut(false);

#ifdef SHIFT_DEBUG
				// Debug
				printf("=== Time Shift Only Downshift State: FINISH_SHIFT\n");
				printf("How: Enter Gear time completed\n");
				printf("Current Tick: %lu\n", HAL_GetTick());
				printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
				lastShiftingChangeTick = HAL_GetTick();
#endif
			}
		}
		break;

	case ST_D_HOLD_CLUTCH: // **********************************************
		// some extra time to hold the clutch open. This is in the case that
		// the shift lever does not hit the threshold and might needs some
		// input from the driver to hit the right revs
		// Should still happen in clutchless downshift because we only use
		// mode to save air but would prefer not to fail shift
		set_slow_clutch_drop(SOLENOID_OFF);
		set_clutch_solenoid(SOLENOID_ON);
		set_downshift_solenoid(SOLENOID_ON);
		safe_spark_cut(false);

		// check if we are done giving the extra time TODO Make sure this gets tuned (currently long apparently)
		if (HAL_GetTick() - begin_hold_clutch_tick > DOWNSHIFT_FAIL_EXTRA_CLUTCH_HOLD(shift_mode_2))
		{
			// done giving the extra clutch. Move on to finishing the shift
			begin_extra_push_time_tick = HAL_GetTick();
			next_downshift_state = ST_D_EXTRA_PUSH;

#ifdef SHIFT_DEBUG
			// Debug
			printf("=== Downshift State: FINISH_SHIFT\n");
			printf("How: Hold Clutch time completed\n");
			printf("Current Tick: %lu\n", HAL_GetTick());
			printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
			lastShiftingChangeTick = HAL_GetTick();
#endif
		}
		break;

	case ST_D_EXTRA_PUSH:
		safe_spark_cut(false);

		set_downshift_solenoid(SOLENOID_ON);

		if (HAL_GetTick() - begin_extra_push_time_tick > DOWNSHIFT_EXTRA_PUSH_TIME_MS(shift_mode_2))
		{
			next_downshift_state = ST_D_FINISH_SHIFT;
		}
		break;

	case ST_D_FINISH_SHIFT: // **********************************************
		// winding down the downshift. Make sure enough time has passed with
		// the clutch open if we are using the clutch, otherwise end the shift
		// but keep pushing on the shift solenoid for a little bit longer to
		// ensure the shift completes
		set_downshift_solenoid(SOLENOID_ON);

		safe_spark_cut(false);
		set_slow_clutch_drop(SOLENOID_OFF);
		set_clutch_solenoid(SOLENOID_OFF);
		tcm_data.using_clutch = false;
		tcm_data.num_shifts += 1;

#ifdef NO_GEAR_POT
		tcm_data.current_gear = get_current_gear();
#endif

		// Determine if the shift was successful by checking if we changed gear correctly
		tcm_data.successful_shift = tcm_data.current_gear <= initial_gear - 2;
		tcm_data.gear_established = tcm_data.successful_shift;
		if (tcm_data.successful_shift) {
			tcm_data.num_successful_shifts += 1;
		}

		// done with the downshift state machine
		set_downshift_solenoid(SOLENOID_OFF);
		main_state = ST_IDLE;

#ifdef SHIFT_DEBUG
		// Debug
		printf("=== Main State: Idle\n");
		printf("How: Finish Shift functions completed\n");
		printf("Current Tick: %lu\n", HAL_GetTick());
		printf("Distance from Last Occurrence: %lu //\n", HAL_GetTick() - lastShiftingChangeTick);
		lastShiftingChangeTick = HAL_GetTick();
#endif
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
	//HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, !!remote_param);
	return;
}


// init_error
//  This function will stay in an infinite loop, blinking the LED in a 0.5sec period. Should only
//  be called from the init function before the RTOS starts
void init_error(void)
{
	while (1)
	{
		HAL_GPIO_TogglePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin);
		HAL_Delay(250);
	}
}

// end of GopherCAN_example.c
