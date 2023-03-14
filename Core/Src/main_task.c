// GopherCAN_devboard_example.c
//  This is a bare-bones module file that can be used in order to make a module main file

#include "main_task.h"
#include "main.h"
#include "utils.h"
#include "gopher_sense.h"
#include <stdio.h>
#include "shift_parameters.h"

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;

// Use this to define what module this board will be
#define THIS_MODULE_ID TCM_ID
#define PRINTF_HB_MS_BETWEEN 1000
#define HEARTBEAT_MS_BETWEEN 500
#define TCM_DATA_UPDATE_MS_BETWEEN 10

// some global variables for examples
static Main_States_t car_Main_State = ST_IDLE;
static Upshift_States_t car_Upshift_State;
static Downshift_States_t car_Downshift_State;
U8 last_button_state = 0;

// the CAN callback function used in this example
static void change_led_state(U8 sender, void* UNUSED_LOCAL_PARAM, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
static void init_error(void);
static void updateAndQueueParams(void);
static void run_upshift_sm(void);
static void run_downshift_sm(void);
static void clutch_task(void);

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

	if (HAL_GetTick() - lastHeartbeat > HEARTBEAT_MS_BETWEEN)
	{
		lastHeartbeat = HAL_GetTick();
		HAL_GPIO_TogglePin(HBEAT_GPIO_Port, HBEAT_Pin);
	}

	updateAndQueueParams();
	clutch_task();

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
	update_and_queue_param_u8(&tcmShiftState_state, tcm_data.shift_mode);

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

// clutch_task
//  for use in the main task. Sets the slow and fast drop accordingly and handles
//  clutch position if in ST_IDLE
static void clutch_task() {
	static bool using_slow_drop = false;
	// normal clutch button must not be pressed when using slow drop. Fast drop is
	// given priority
	if (swFastClutch_state.data) using_slow_drop = false;

	// if the slow drop button is pressed latch the slow drop
	else if (swSlowClutch_state.data) using_slow_drop = true;

	// If either clutch button pressed then enable solenoid. Always turn it on regardless of
	// if we are shifting or not
	if (swFastClutch_state.data || swSlowClutch_state.data)
	{
		set_clutch_solenoid(SOLENOID_ON);
		return;
	}

	// If neither clutch button pressed and we are in IDLE and not in anti stall
	// close clutch solenoid. This will cause clutch presses to latch to the end
	// of a shift
	if (!(swFastClutch_state.data || swSlowClutch_state.data)
			&& car_Main_State == ST_IDLE && tcm_data.anti_stall)
	{
		set_clutch_solenoid(SOLENOID_OFF);

		// if we are using slow drop enable or disable the slow release valve depending
		// on if we are near the bite point
		if (using_slow_drop)
		{
			// when slow dropping, we want to start by fast dropping until the bite point
			if (get_clutch_pot_pos() < CLUTCH_OPEN_POS_MM - CLUTCH_SLOW_DROP_FAST_TO_SLOW_EXTRA_MM)
			{
				set_slow_drop(false);
			}
			else
			{
				set_slow_drop(true);
			}
		}
		else
		{
			// not using slow drop
			set_slow_drop(false);
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
	static uint32_t begin_shift_tick;
	static uint32_t begin_exit_gear_tick;
	static uint32_t begin_enter_gear_tick;
	static uint32_t begin_exit_gear_spark_return_tick;
	static uint32_t finish_shift_start_tick;
	static uint32_t spark_cut_start_time;

	// calculate the target RPM at the start of each cycle through the loop
	tcm_data.target_RPM = calc_target_RPM(tcm_data.target_gear);

	switch (car_Upshift_State)
	{
	case ST_U_BEGIN_SHIFT:
		// at the beginning of the upshift, start pushing on the shift lever
		// to "preload". This means that there will be force on the shift
		// lever when the spark is cut and the gear can immediately disengage
		// NOTE: We really dont care about what the clutch is doing in an
		// upshift

		begin_shift_tick = HAL_GetTick(); // Log that first begin shift tick
		set_upshift_solenoid(SOLENOID_ON); // start pushing upshift

		// reset information about the shift
		tcm_data.successful_shift = true;

		// move on to waiting for the "preload" time to end
		car_Upshift_State = ST_U_LOAD_SHIFT_LVR;
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
			car_Upshift_State = ST_U_EXIT_GEAR;
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

		// wait for the shift lever to reach the leaving threshold
		if (get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM)
		{
			// the shifter position is above the threshold to exit. The
			// transmission is now in a false neutral position
			car_Upshift_State = ST_U_ENTER_GEAR;
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
			car_Upshift_State = ST_U_SPARK_RETURN;
		}
		break;

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
			// If spark return successfully releases then continue onto the
			// next phase of the shift. If it was not successful (timeout) move
			// on anyway
			safe_spark_cut(true);
			car_Upshift_State = ST_U_ENTER_GEAR;
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

		// check if the shifter position is above the threshold to complete
		// a shift
		if (get_shift_pot_pos() > UPSHIFT_ENTER_POS_MM)
		{
			// shift position says we are done shifting
			car_Upshift_State = ST_U_FINISH_SHIFT;
			finish_shift_start_tick = HAL_GetTick();
			break;
		}

		// check if we are out of time for this state
		if (HAL_GetTick() - begin_enter_gear_tick > UPSHIFT_ENTER_TIMEOUT_MS)
		{
			// at this point the shift was probably not successful. Note this so we
			// dont increment the gears and move on
			tcm_data.successful_shift = false;
			car_Upshift_State = ST_U_FINISH_SHIFT;
			finish_shift_start_tick = HAL_GetTick();
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
			if (HAL_GetTick() - finish_shift_start_tick >= UPSHIFT_EXTRA_PUSH_TIME)
			{
				// done with the upshift state machine
				set_upshift_solenoid(SOLENOID_OFF);
				car_Main_State = ST_IDLE;
			}
		}
		break;
	}
}

// run_downshift_sm
//  The big boi downshift state machine
static void run_downshift_sm(void)
{
	static uint32_t begin_shift_tick;
	static uint32_t begin_exit_gear_tick;
	static uint32_t begin_enter_gear_tick;
	static uint32_t begin_hold_clutch_tick;
	static uint32_t finish_shift_start_tick;

	// calculate the target rpm at the start of each cycle
	tcm_data.target_RPM = calc_target_RPM(tcm_data.target_gear);

	switch (car_Downshift_State)
	{
	case ST_D_BEGIN_SHIFT:
	// at the beginning of a shift reset all of the variables and start
	// pushing on the clutch and downshift solenoid. Loading the shift
	// lever does not seem to be as important for downshifts, but still
	// give some time for it
	begin_shift_tick = HAL_GetTick();

	set_downshift_solenoid(SOLENOID_ON);

	tcm_data.using_clutch = tcm_data.shift_mode != CLUTCHLESS_DOWNSHIFT; // EXPIREMENTAL: Uncomment the next line to only clutch during a downshift if the clutch is held during the start of the shift
	//tcm_data.using_clutch = (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button);

	set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);

	// reset the shift parameters
	tcm_data.successful_shift = true;

	// move on to loading the shift lever
	car_Downshift_State = ST_D_LOAD_SHIFT_LVR;
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
		car_Downshift_State = ST_D_EXIT_GEAR;

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

	// wait for the shift lever to be below the downshift exit threshold
	if (get_shift_pot_pos() < DOWNSHIFT_EXIT_POS_MM)
	{
		// we have left the last gear and are in a false neutral. Move on to
		// the next part of the shift
		car_Downshift_State = ST_D_ENTER_GEAR;
		begin_enter_gear_tick = HAL_GetTick();
		break;
	}

	// check if this state has timed out
	if (HAL_GetTick() - begin_exit_gear_tick > DOWNSHIFT_EXIT_TIMEOUT_MS)
	{
		// We could not release the gear for some reason. Keep trying anyway
		car_Downshift_State = ST_D_ENTER_GEAR;
		begin_enter_gear_tick = HAL_GetTick();

		// if we were not using the clutch before, start using it now because
		// otherwise we're probably going to fail the shift
		tcm_data.using_clutch = true;
	}
	break;

	case ST_D_ENTER_GEAR:
	// now we are in a false neutral position we want to keep pushing the
	// shift solenoid, but now dynamically spark cut if the blip is too big
	// and the RPM goes too high to enter the next gear
	set_downshift_solenoid(SOLENOID_ON);
	set_clutch_solenoid(tcm_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);
	reach_target_RPM_spark_cut(tcm_data.target_RPM);

	// check if the shift position has moved enough to consider the shift
	// finished
	if (get_shift_pot_pos() < DOWNSHIFT_ENTER_POS_MM)
	{
		// the clutch lever has moved enough to finish the shift. Turn off
		// any spark cutting and move on to finishing the shift
		safe_spark_cut(false);
		car_Downshift_State = ST_D_FINISH_SHIFT;
		finish_shift_start_tick = HAL_GetTick();
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
		car_Downshift_State = ST_D_HOLD_CLUTCH;
		begin_hold_clutch_tick = HAL_GetTick();
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
		car_Downshift_State = ST_D_FINISH_SHIFT;
		finish_shift_start_tick = HAL_GetTick();
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
			car_Main_State = ST_IDLE;
		}
	}
	break;
	}
}

// end of GopherCAN_example.c
