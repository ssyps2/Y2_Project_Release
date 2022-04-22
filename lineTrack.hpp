#ifndef LINE_TRACK_HPP
#define LINE_TRACK_HPP

#include "main.hpp"

#define LEFT_MOTOR_BASE_SPEED   24
#define RIGHT_MOTOR_BASE_SPEED  20
#define SCREEN_CENTER_PLACE     480/2
#define MAXIMUM_SPEED           90

/* Functions for Users */
void chassisInit(vehicleControl_t *robot);

void trackLine(vehicleControl_t *robot);


#endif
