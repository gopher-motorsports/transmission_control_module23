/*
 * main_task.c
 *
 *  Created on: Nov, 2022
 *      Author: sebas & Nate Redfern
 */
#include <main_task.h>
#include "main.h"
#include "DAM.h"
#include "car_utils.h"
#include "display.h"
#include "shift_parameters.h"

// Use this to define what module this board will be
#define THIS_MODULE_ID TCM_ID

extern CAN_HandleTypeDef hcan1;
extern bool using_slow_drop;

static void run_upshift_sm(void);
static void run_downshift_sm(void);

static Main_States_t car_Main_State = ST_IDLE;
static Upshift_States_t car_Upshift_State;
static Downshift_States_t car_Downshift_State;
static uint32_t last_lap_time = 0;
static uint32_t fastest_lap_time = (uint32_t)(-1);
static int last_fastest_delta_time = 0;

void init_main_task()
{
	init_can(&hcan1, TCM_ID, BXTYPE_MASTER);

	return;
}



int main_task(void)
{
	// Heartbeat LED
	static uint32_t last_heartbeat;
	if (HAL_GetTick() - last_heartbeat > 500)
	{
		last_heartbeat = HAL_GetTick();
		HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
	}

	update_and_queue_param_u32(&tcm_target_rpm, car_shift_data.target_RPM);
	update_and_queue_param_u8(&tcm_current_gear, car_shift_data.current_gear);
	update_and_queue_param_u8(&tcm_target_gear, car_shift_data.target_gear);
	update_and_queue_param_u8(&tcm_currently_moving, car_shift_data.currently_moving);
	update_and_queue_param_u8(&tcm_successful_shift, car_shift_data.successful_shift);
	update_and_queue_param_u32(&tcm_trans_rpm, car_shift_data.trans_speed);
	update_and_queue_param_u8(&tcm_using_clutch, car_shift_data.using_clutch);
	update_and_queue_param_u8(&tcm_anti_stall, car_shift_data.anti_stall);

	// logic for sending the current state of the shifts
	switch (car_Main_State)
	{
	default:
	case ST_IDLE:
		// not shifting, send 0
		update_and_queue_param_u8(&tcm_shift_state, 0);
		break;

	case ST_HDL_UPSHIFT:
		// send the upshift state
		update_and_queue_param_u8(&tcm_shift_state, car_Upshift_State);
		break;

	case ST_HDL_DOWNSHIFT:
		// send the downshift state
		update_and_queue_param_u8(&tcm_shift_state, car_Downshift_State);
		break;
	}

	// check for the lap timer signal. Use logic to make sure a clean signal
	// is held to make it easier to send to the display
	static uint32_t last_lap_beacon = 0;
	if (HAL_GetTick() - last_lap_beacon <= MIN_LAP_TIME_ms)
	{
		// keep the lap beacon at high to make a clean signal
		update_and_queue_param_u8(&tcm_lap_timer, 1);
	}
	else
	{
		// there has not been a new lap within the buffer zone
		if (!tcm_lap_timer.data)
		{
			// a new lap has been detected
			update_and_queue_param_u8(&tcm_lap_timer, 1);
			last_lap_time = HAL_GetTick() - last_lap_beacon;
			last_fastest_delta_time = last_lap_time - fastest_lap_time;
			if(fastest_lap_time != 0 && last_lap_time < fastest_lap_time) {
				fastest_lap_time = last_lap_time;
			}
			last_lap_beacon = HAL_GetTick();
			send_lap_time_data(last_lap_time, fastest_lap_time);
		}
		else
		{
			// no new lap, keep sending low
			update_and_queue_param_u8(&tcm_lap_timer, 0);
		}
	}



	// Update shift struct with relevant data
	update_car_shift_struct();

	// gear calculation handling
	update_wheel_arr();
	update_rpm_arr();

	// only check the gear every 25ms
	static uint32_t last_gear_update = 0;
	if (HAL_GetTick() - last_gear_update >= GEAR_UPDATE_TIME_ms)
	{
		last_gear_update = HAL_GetTick();
		car_shift_data.current_gear = get_current_gear(car_Main_State);
	}


	// Anti Stall maybe needed maybe not
	//anti_stall();

	// Update Display
	static uint32_t last_display_update = 0;
	if (HAL_GetTick() - last_display_update >= DISPLAY_UPDATE_TIME_ms)
	{
		last_display_update = HAL_GetTick();
		send_display_data(0 /*last_fastest_delta_time*/);
	}

	// handle the clutch based on the state of the car and the buttons
	//TODO: rewrite for new CAN buttons
	clutch_task(sw_clutch_fast, sw_clutch_slow, car_Main_State, car_shift_data.anti_stall);

	switch (car_Main_State)
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
		if (sw_downshift.data)
		{
			sw_downshift.data = 0;
			if (calc_validate_downshift(car_shift_data.current_gear, sw_clutch_fast, sw_clutch_slow))
			{
				car_Main_State = ST_HDL_DOWNSHIFT;
				car_Downshift_State = ST_D_BEGIN_SHIFT;
			}
		}

		// same for upshift. Another shift can be queued
		if (sw_upshift.data)
		{
			sw_upshift.data = 0;
			if (calc_validate_upshift(car_shift_data.current_gear, sw_clutch_fast, sw_clutch_slow))
			{
				car_Main_State = ST_HDL_UPSHIFT;
				car_Upshift_State = ST_U_BEGIN_SHIFT;
			}
		}
		break;

	case ST_HDL_UPSHIFT:
		run_upshift_sm();
		break;

	case ST_HDL_DOWNSHIFT:
		run_downshift_sm();
		break;
	}

	return 0;
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
	car_shift_data.target_RPM = calc_target_RPM(car_shift_data.target_gear);

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
		car_shift_data.successful_shift = true;

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
		//reach_target_RPM_spark_cut(car_shift_data.target_RPM);
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
		//reach_target_RPM_spark_cut(car_shift_data.target_RPM);
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
			car_shift_data.successful_shift = false;
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
			car_shift_data.using_clutch = false;
			if (car_shift_data.successful_shift)
			{
				car_shift_data.current_gear = car_shift_data.target_gear;
			}
			else
			{
				car_shift_data.current_gear = ERROR_GEAR;
				car_shift_data.gear_established = false;
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
	car_shift_data.target_RPM = calc_target_RPM(car_shift_data.target_gear);

	switch (car_Downshift_State)
	{
	case ST_D_BEGIN_SHIFT:
		// at the beginning of a shift reset all of the variables and start
		// pushing on the clutch and downshift solenoid. Loading the shift
		// lever does not seem to be as important for downshifts, but still
		// give some time for it
		begin_shift_tick = HAL_GetTick();
		set_downshift_solenoid(SOLENOID_ON);

		// we will always be using the clutch for downshifting
		car_shift_data.using_clutch = true; // EXPIREMENTAL: Uncomment the next line to only clutch during a downshift if the clutch is held during the start of the shift
		//car_shift_data.using_clutch = (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button);
		set_clutch_solenoid(car_shift_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);

		// reset the shift parameters
		car_shift_data.successful_shift = true;

		// move on to loading the shift lever
		car_Downshift_State = ST_D_LOAD_SHIFT_LVR;
		break;

	case ST_D_LOAD_SHIFT_LVR:
		// load the shift lever. This is less important as we do not seem to
		// have issues leaving the gear during a downshift
		set_downshift_solenoid(SOLENOID_ON);
		set_clutch_solenoid(car_shift_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);

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
		set_clutch_solenoid(car_shift_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);
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

			// if we were not using the clutch before, start using it now
			car_shift_data.using_clutch = true;
		}
		break;

	case ST_D_ENTER_GEAR:
		// now we are in a false neutral position we want to keep pushing the
		// shift solenoid, but now dynamically spark cut if the blip is too big
		// and the RPM goes too high to enter the next gear
		set_downshift_solenoid(SOLENOID_ON);
		set_clutch_solenoid(car_shift_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);
		reach_target_RPM_spark_cut(car_shift_data.target_RPM);

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
			car_shift_data.using_clutch = true;
			set_clutch_solenoid(SOLENOID_ON);
			safe_spark_cut(false);
			car_shift_data.successful_shift = false;
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
			set_clutch_solenoid(car_shift_data.using_clutch ? SOLENOID_ON : SOLENOID_OFF);
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
			car_shift_data.using_clutch = false;
			if (car_shift_data.successful_shift)
			{
				car_shift_data.current_gear = car_shift_data.target_gear;
			}
			else
			{
				car_shift_data.current_gear = ERROR_GEAR;
				car_shift_data.gear_established = false;
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


void can_buffer_handling_loop()
{
	// handle each RX message in the buffer
	if (service_can_rx_buffer())
	{
		// an error has occurred
	}

	// handle the transmission hardware for each CAN bus
	service_can_tx_hardware(&hcan1);
}


