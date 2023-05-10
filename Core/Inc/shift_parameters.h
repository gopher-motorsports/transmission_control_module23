/*
 * shift_parameters.h
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */

#ifndef INC_SHIFT_PARAMETERS_H_
#define INC_SHIFT_PARAMETERS_H_

//#define AUTO_SHIFT_LEVER_RETURN
#define NO_GEAR_POT
//#define SHIFT_DEBUG

// shifting defines for both upshift and downshift
#define LEVER_NEUTRAL_POS_MM 37.0f
#define LEVER_NEUTRAL_TOLERANCE 0.8
#define TARGET_RPM_TOLERANCE 0.03f

// upshift defines
#define UPSHIFT_SHIFT_LEVER_PRELOAD_TIME_MS 60 // long preload to get lots of load on the shift lever
#define UPSHIFT_EXIT_TIMEOUT_MS 25 // short time to exit. If the shift lever wasn't pushed far enough quickly it is probably stuck
#define UPSHIFT_EXIT_SPARK_RETURN_MS 10 // we dont want the spark return to be too long
#define UPSHIFT_ENTER_TIMEOUT_MS 30 // data shows this does not take very long, so have a pretty short timeout here.
#define UPSHIFT_EXIT_POS_MM 42.8f
#define UPSHIFT_ENTER_POS_MM 47.3f
#define UPSHIFT_EXIT_GEAR_TIME_MS 40
#define UPSHIFT_ENTER_GEAR_TIME_MS 40
#define UPSHIFT_EXTRA_PUSH_TIME_MS 100
#define DOWNSHIFT_EXIT_GEAR_TIME_MS 60
#define DOWNSHIFT_ENTER_GEAR_TIME_MS 60

// downshift defines
#define DOWNSHIFT_SHIFT_LEVER_PRELOAD_TIME_MS 20 // preloads seem to be less important for downshifts as exiting gear is almost always successful
#define DOWNSHIFT_EXIT_TIMEOUT_MS 30 // short time to exit. Data shows we can exit pretty easily
#define DOWNSHIFT_ENTER_TIMEOUT_MS 75
#define DOWNSHIFT_FAIL_EXTRA_CLUTCH_HOLD 50 // some extra time is given in addition because the clutch takes some time to return to the bite point
#define DOWNSHIFT_EXIT_POS_MM 34.0f
#define DOWNSHIFT_ENTER_POS_MM 28.2f
#define DOWNSHIFT_EXTRA_PUSH_TIME_MS 100

// clutch defines
#define CLUTCH_OPEN_POS_MM 2.3f
#define CLUTCH_SLOW_DROP_FAST_TO_SLOW_EXTRA_MM 3.0f
#define CLUTCH_OPEN_TIMEOUT_MS 300

// transmission, wheel speed, and RPM array defines
#define RPM_ARRAY_SIZE 250
#define WHEEL_SPEED_ARRAY_SIZE RPM_ARRAY_SIZE
#define TRANS_ARR_SIZE 100
#define TRANS_TO_ENGINE_RATIO 7.5f
#define WHEEL_TO_TRANS_RATIO 20.0f

// gear establishing and calculating defines
#define DEFAULT_WHEEL_SPEED_AVE_TIME_ms 25
#define GEAR_NOT_ESTABLISHED_NUM_SAMPLES_ms 5
#define GEAR_ESTABLISHED_NUM_SAMPLES_ms 20
#define GEAR_ESTABLISH_TOLERANCE_percent 0.03f
#define NUM_OF_GEARS 5
#define GEAR_1_TRANS_RATIO 4.62f
#define GEAR_2_TRANS_RATIO 3.67f
#define GEAR_3_TRANS_RATIO 3.21f
#define GEAR_4_TRANS_RATIO 2.86f
#define GEAR_5_TRANS_RATIO 2.60f

#define GEAR_POS_MARGIN_mm 0.1
#define NEUTRAL_DISTANCE_mm 0
#define GEAR_1_DISTANCE_mm 1
#define GEAR_2_DISTANCE_mm 2
#define GEAR_3_DISTANCE_mm 3
#define GEAR_4_DISTANCE_mm 4
#define GEAR_5_DISTANCE_mm 5

// RPM cutoffs
#define MAX_RPM 14000
#define MIN_SPARK_CUT_RPM 3000
#define ANTI_STALL_RPM 1700
#define MAX_CRANK_RPM 700

// misc defines
#define MOVING_WHEEL_SPEED_MIN_CUTOFF 2.0f
#define BUTTON_DEBOUNCE_MS 20

#endif /* INC_SHIFT_PARAMETERS_H_ */
