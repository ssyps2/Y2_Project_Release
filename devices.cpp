#ifdef __cplusplus
extern "C"
{
#include "stdlib.h"
#include "lcd.h"
#include "softPwm.h"
}
#endif

#include "devices.hpp"


float distance;

struct timeval timeBeginSonic, timeEndSonic;
int durationSonic;

/* Declaration of Functions */
void deviceInit(vehicleControl_t *robot){
    //lcd init
    lcdInit(2, 16, 4, LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7, 0, 0, 0, 0);
    lcdClear(robot->lcd);

    //ultrasonic init
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    digitalWrite(TRIG_PIN, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, LOW);
    delayMicroseconds(2);

    pinMode(SERVO_PIN, OUTPUT);
    softPwmCreate(SERVO_PIN, 75, 200);

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    robot->timerFlag = 0;

    puts("Device Init Success\n");
}

float distanceDetect(vehicleControl_t *robot){
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    //while (!(digitalRead(ECHO_PIN) == 1) && puts("!"));
    while (!(digitalRead(ECHO_PIN) == 1));
    gettimeofday(&timeBeginSonic, NULL);
    //while (!(digitalRead(ECHO_PIN) == 0) && puts("@"));
    while (!(digitalRead(ECHO_PIN) == 0));
    gettimeofday(&timeEndSonic, NULL);

    durationSonic = (float)(timeEndSonic.tv_sec * 1000000 + timeEndSonic.tv_usec)
               - (float)(timeBeginSonic.tv_sec * 1000000 + timeBeginSonic.tv_usec);

    distance = (durationSonic / 2) * 34300 / 1000000;
//    printf("Distance: %4.2f cm\n", distanceSonic);

    return distance;
}

void servoControl(int cmdAngle){
    for(int i = 0; i < 50; i++){
        softPwmWrite(SERVO_PIN, cmdAngle*0.15);
    }

    std::cout << "Servo: " << cmdAngle << std::endl;
}

void playAudio(vehicleControl_t *robot){
    system("play /home/pi/Music/LOSER.mp3");

    std::cout << "Play Music Done" << std::endl;
    robot->modeFlag = MATCHDONE;
}

void alarmLED(vehicleControl_t *robot){
    while (robot->timerFlag != 2){
        taskTimer(5);   //timer start, 5s
        digitalWrite(LED_RED, HIGH);
        delay(900);
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_BLUE, HIGH);
        delay(900);
        digitalWrite(LED_BLUE, LOW);
    }

    resetTimer();

    std::cout << "Alarm LED Done" << std::endl;
    robot->modeFlag = MATCHDONE;
}
