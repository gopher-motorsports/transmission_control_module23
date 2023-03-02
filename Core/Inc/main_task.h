// GopherCAN_devboard_example.h
//  Header file for GopherCAN_devboard_example.c

#ifndef GOPHERCAN_DEVBOARD_EXAMPLE_H
#define GOPHERCAN_DEVBOARD_EXAMPLE_H

#include "GopherCAN.h"

void init(CAN_HandleTypeDef* hcan_ptr);
void can_buffer_handling_loop();
void main_loop();

float get_gear_pot_pos(void);
float get_clutch_pot_pos(void);
float get_shift_pot_pos(void);

#endif
