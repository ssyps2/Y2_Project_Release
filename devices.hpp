#ifndef DEVICES_HPP
#define DEVICES_HPP

#include "main.hpp"

/* Ultrasonic related Define */
#define TRIG_PIN 20
#define ECHO_PIN 21

/* LCD related Define */
#define LCD_RS	 22
#define LCD_E	 27
#define LCD_D4	 18
#define LCD_D5 	 23
#define LCD_D6	 24
#define LCD_D7 	 25

/* Servo Motor related Define */


/* Functions for Users */
void deviceInit(vehicleControl_t *robot);

float distanceDetect();

void lcdDisplay(vehicleControl_t *robot);

void servoControl(vehicleControl_t *robot);

#endif
