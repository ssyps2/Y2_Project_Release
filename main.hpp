#ifndef MAIN_HPP
#define MAIN_HPP

extern "C"
{
#include "sys/time.h"
#include "wiringPi.h"
#include "wiringSerial.h"
}
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>

/* Namespace Usage */
using namespace cv;
using std::string;

template <typename Tp, typename Alloc = std::allocator<Tp>>
using Vector = std::vector<Tp, Alloc>;

const string CountShape1 = "/home/pi/Pictures/Template/CountShape1.PNG";
const string CountShape2 = "/home/pi/Pictures/Template/CountShape2.png";
const string CountShape3 = "/home/pi/Pictures/Template/CountShape3.png";
const string PlayAudio = "/home/pi/Pictures/Template/PlayAudio.png";
const string Alarm = "/home/pi/Pictures/Template/Alarm.png";
const string MeasureDistance = "/home/pi/Pictures/Template/MeasureDistance.PNG";
const string ShortCutBlue = "/home/pi/Pictures/Template/ShortCutBlue.PNG";
const string ShortCutGreen = "/home/pi/Pictures/Template/ShortCutGreen.PNG";
const string ShortCutRed = "/home/pi/Pictures/Template/ShortCutRed.PNG";
const string ShortCutYellow = "/home/pi/Pictures/Template/ShortCutYellow.png";

const int matching_frame_width = 320, matching_frame_height = 240;
const int tracking_frame_width = 480, tracking_frame_height = 272;

/* Operated Object related Structure */
typedef struct{
    int error, errorPrevious;
    float kp, ki, kd;
    float pOut, iOut, dOut;
    int out, maxOut, maxIOut;
}PIDTypeDef;

typedef enum{
    TRACK,
    DETECTED,
    MATCH,
    MATCHDONE,

    AUDIO,
    LEDBLINK,
    ULTRASONIC,
    SHAPECOUNT,

    EMPTY,
}modeTypedef;

typedef struct{
    uchar red;
    uchar green;
    uchar blue;

    uchar hue;
    uchar saturate;
    uchar value;
}colorSpaceTypedef;

typedef struct{
    int serial;
    int lcd;
    VideoCapture webcam;
    PIDTypeDef pid;
    modeTypedef modeFlag;
    modeTypedef lastModeFlag;
    colorSpaceTypedef imageColor;
    uchar timerFlag;    //0:wait, 1:counting, 2:done
    uchar resizeFlag;
    uchar colorMode;    //0:no color, 1:yellow, 2:red, 3:blue, 4:green
}vehicleControl_t;


float taskTimer(float seconds);

void resetTimer();

#endif
