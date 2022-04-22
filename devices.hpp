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
#define SERVO_PIN 6

/* LED related Define */
#define LED_RED   13
#define LED_BLUE  19


/* Functions for Users */
void deviceInit(vehicleControl_t *robot);

float distanceDetect(vehicleControl_t *robot);

void servoControl(int cmdAngle);


void playAudio(vehicleControl_t *robot);

void alarmLED(vehicleControl_t *robot);

#endif
