extern "C"
{
#include "lcd.h"
}
#include "main.hpp"
#include "visualScan.hpp"
#include "lineTrack.hpp"
#include "devices.hpp"
#include <thread>

static void imageMatching(const string comparePath);

vehicleControl_t robot;

struct timeval timeBegin, timeEnd;
int duration = 0;
int angle;
//std::thread webcamReadThread;

/* Start of Main */
int main() {
    std::cout << "enter main" << std::endl;
    webcamInit(&robot);
    chassisInit(&robot);
    deviceInit(&robot);

    //webcamReadThread();

    while(true){
        std::cout << "mode: " << robot.modeFlag << std::endl;
//        scanf("%d", &angle);
//        servoControl(angle);
        if (robot.modeFlag == TRACK){
            robot.lastModeFlag = EMPTY;
            midPointCapture(&robot);
            trackLine(&robot);
        }
        else if(robot.modeFlag == DETECTED){
            robot.lastModeFlag = EMPTY;
            servoControl(38);
            resizeCamera(matching_frame_width, matching_frame_height);
            robot.modeFlag = MATCH;
        }
        else if (robot.modeFlag == MATCH){
            robot.lastModeFlag = EMPTY;
            imageMatching(PlayAudio);
            imageMatching(Alarm);
        }
        else if (robot.modeFlag == MATCHDONE){
            robot.lastModeFlag = MATCHDONE;
            servoControl(75);
            resizeCamera(tracking_frame_width, tracking_frame_height);
            serialPrintf(robot.serial, "#Bafrfr 020 024 024 020");
            delay(2000);
            robot.modeFlag = TRACK;
        }

        if (robot.modeFlag == AUDIO){  //play music
            playAudio(&robot);
            lcdClear(robot.lcd);
            lcdPosition(robot.lcd, 0, 0);
            lcdPuts(robot.lcd, "Play Music");
            delay(10);
            robot.modeFlag = MATCHDONE;
        }
        else if (robot.modeFlag == LEDBLINK){
            alarmLED(&robot);
            lcdClear(robot.lcd);
            lcdPosition(robot.lcd, 0, 0);
            lcdPuts(robot.lcd, "Alarm Flash");
            delay(10);
            robot.modeFlag = MATCHDONE;
        }
        else if (robot.modeFlag == ULTRASONIC){

            lcdClear(robot.lcd);
            lcdPosition(robot.lcd, 0, 0);
            lcdPuts(robot.lcd, "Approach & Stop");
            delay(10);
            lcdPosition(robot.lcd, 0, 1);
            lcdPuts(robot.lcd, "Dist");
            delay(10);
            lcdPosition(robot.lcd, 13, 1);
            lcdPuts(robot.lcd, "cm");
            delay(10);

        }
    }
}

static void imageMatching(const string comparePath){
    float filteredMatchRatio;

    try{
        filteredMatchRatio = visualMatch(&robot, comparePath);
        std::cout << "filteredMatchRatio: " << filteredMatchRatio << std::endl;
    } catch(...){
        std::cout << "catch" << std::endl;
    }

    if (filteredMatchRatio > 75){
        if (comparePath == PlayAudio){
            robot.modeFlag = AUDIO;
        }
        else if (comparePath == Alarm){
            robot.modeFlag = LEDBLINK;
        }
        else if (comparePath == MeasureDistance){
            robot.modeFlag = ULTRASONIC;
        }
        else if (comparePath == CountShape1 || comparePath == CountShape2 || comparePath == CountShape3){
            robot.modeFlag = SHAPECOUNT;
        }
    }
}

float taskTimer(float seconds){
    if (robot.timerFlag == 0){
        gettimeofday(&timeBegin, NULL);
        robot.timerFlag = 1;
    }
    else if (robot.timerFlag == 1){
        gettimeofday(&timeEnd, NULL);
        duration = timeEnd.tv_sec - timeBegin.tv_sec;

        std::cout << "duration" << duration << std::endl;
    }

    if (duration > seconds){
        robot.timerFlag = 2;
    }
}

void resetTimer(){
    robot.timerFlag = 0;
    duration = 0;
}
