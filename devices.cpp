#ifdef __cplusplus
extern "C"
{
#include "lcd.h"
#include "wiringPi.h"
#include "sys/time.h"
//#include "SoftPwm.h"
}
#endif

#include "devices.hpp"


float distance, duration;

struct timeval timeBegin, timeEnd;

/* Declaration of Functions */
void deviceInit(vehicleControl_t *robot){
    //lcd init
    lcdInit(2, 16, 4, LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7, 0, 0, 0, 0);
    lcdPosition(robot->lcd, 0, 0);
    lcdPuts(robot->lcd, "Distance");
    delay(50);
    lcdPosition(robot->lcd, 8, 1);
    lcdPuts(robot->lcd, "cm");
    delay(50);

    //ultrasonic init
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    puts("Device Init Success\n");
}

float distanceDetect(){
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    //while (!(digitalRead(ECHO_PIN) == 1) && puts("!"));
    while (!(digitalRead(ECHO_PIN) == 1));
    gettimeofday(&timeBegin, NULL);
    //while (!(digitalRead(ECHO_PIN) == 0) && puts("@"));
    while (!(digitalRead(ECHO_PIN) == 0));
    gettimeofday(&timeEnd, NULL);

    duration = (float)(timeEnd.tv_sec * 1000000 + timeEnd.tv_usec)
               - (float)(timeBegin.tv_sec * 1000000 + timeBegin.tv_usec);

    distance = (duration / 2) * 34300 / 1000000;
//    printf("Distance: %4.2f cm\n", distance);

    return distance;
}

void lcdDisplay(vehicleControl_t *robot){

}

void servoControl(vehicleControl_t *robot){

}
