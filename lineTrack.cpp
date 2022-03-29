#ifdef __cplusplus
extern "C"
{
#include <stdlib.h>
#include "imgproc.h"
#include "wiringSerial.h"
#include "wiringPi.h"
}
#endif

#include <cmath>
#include "lineTrack.hpp"
#include "visualScan.hpp"

/* Prototype of Static Functions */
static void PIDInit(PIDTypeDef *pid);

static int PIDCalc(PIDTypeDef *pid, int pidFdb, int pidSet);


/* Declaration of Functions */
void chassisInit(vehicleControl_t *robot){
    //serial init
    wiringPiSetupGpio();
    robot->serial = serialOpen("/dev/ttyAMA0", 57600); // returns int, -1 for error

    if (robot->serial == -1){
        puts("Error in opening serial\n");
        exit(-1);
    }

    serialPrintf(robot->serial, "#ha");

    PIDInit(&robot->pid);

    puts("Chassis Init Success\n");
}

void trackLine(vehicleControl_t *robot){
    int midPosition, pidOut;
    int leftMotorSpeed, rightMotorSpeed;
    char Motor2Direction, Motor3Direction;  //left
    char Motor1Direction, Motor4Direction;  //right

    midPosition = midPointCapture(robot);
    pidOut = PIDCalc(&robot->pid, (float)midPosition * 0.1, (float)SCREEN_CENTER_PLACE * 0.1);

    leftMotorSpeed = LEFT_MOTOR_BASE_SPEED - pidOut;
    rightMotorSpeed = RIGHT_MOTOR_BASE_SPEED + pidOut;

    //direction determination
    if (leftMotorSpeed < 0){
        leftMotorSpeed = abs(leftMotorSpeed);
        Motor2Direction = 'f';
        Motor3Direction = 'r';
    }
    else if (leftMotorSpeed >= 0){
        Motor2Direction = 'r';
        Motor3Direction = 'f';
    }

    if (rightMotorSpeed < 0){
        rightMotorSpeed = abs(rightMotorSpeed);
        Motor1Direction = 'r';
        Motor4Direction = 'f';
    }
    else if (rightMotorSpeed >= 0){
        Motor1Direction = 'f';
        Motor4Direction = 'r';
    }

    //speed limitation
    if (leftMotorSpeed > MAXIMUM_SPEED){
        leftMotorSpeed = MAXIMUM_SPEED;
    }

    if (rightMotorSpeed > MAXIMUM_SPEED){
        rightMotorSpeed = MAXIMUM_SPEED;
    }

    //std::cout << "pid out:" << pidOut << std::endl;
    std::cout << "L spd: " << leftMotorSpeed << std::endl;
    std::cout << "R spd: " << rightMotorSpeed << std::endl;

    //serialPrintf(robot->serial, "#Bafrfr %03d %03d %03d %03d", rightMotorSpeed, leftMotorSpeed, leftMotorSpeed, rightMotorSpeed);
    //serialPrintf(robot->serial, "#ha");

    if (robot->modeFlag == TRACK){
        serialPrintf(robot->serial, "#Ba%c%c%c%c %03d %03d %03d %03d", Motor1Direction, Motor2Direction, Motor3Direction, Motor4Direction,
                                                                    rightMotorSpeed, leftMotorSpeed, leftMotorSpeed, rightMotorSpeed);
        //std::cout << "OK" << std::endl;
    }
    else if (robot->modeFlag == DETECTED){
        //stop the vehicle
        //serialPrintf(robot->serial, "#ha");
        //raise the camera

    }
    else if (robot->modeFlag == MATCH){
        //
    }
}

static void PIDInit(PIDTypeDef *pid){
    //initialize PID parameters
    pid->kp = 3.0;
    pid->ki = 0;
    pid->kd = 0.3;
    pid->maxIOut = 10;
    pid->maxOut = 75;

    //set error and output to be zero
    pid->errorPrevious = pid->error = 0;
    pid->out = pid->pOut = pid->iOut = pid->dOut = 0;
}

static int PIDCalc(PIDTypeDef *pid, int pidFdb, int pidSet){
    pid->errorPrevious = pid->error;
    pid->error = pidSet - pidFdb;

    std::cout << "------------------" << std::endl;
    std::cout << "err: " << pid->error << std::endl;

    pid->pOut = pid->kp * (float)pid->error;
    pid->iOut += pid->ki * (float)pid->error;
    pid->dOut = pid->kd * (float)(pid->error - pid->errorPrevious);

    //limit the max and min value of integral out
    if ((int)pid->iOut > pid->maxIOut){
        pid->iOut = pid->maxIOut;
    }
    else if ((int)pid->iOut < -pid->maxIOut){
        pid->iOut = -pid->maxIOut;
    }

    pid->out = (int)(pid->pOut + pid->iOut + pid->dOut);

    //limit the max and min value of PID out
    if (pid->out > pid->maxOut){
        pid->out = pid->maxOut;
    }
    else if (pid->out < -pid->maxOut){
        pid->out = -pid->maxOut;
    }

    return pid->out;
}
