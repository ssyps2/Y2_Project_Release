#ifndef MAIN_HPP
#define MAIN_HPP

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

/* Operated Object related Structure */
typedef struct{
    int error, errorPrevious;
    float kp, ki, kd;
    float pOut, iOut, dOut;
    int out, maxOut, maxIOut;
}PIDTypeDef;

typedef enum{
    TRACK = 0,
    DETECTED,
    MATCH,
    MATCHDONE,
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
    int servo;
    VideoCapture webcam;
    PIDTypeDef pid;
    modeTypedef lastModeFlag;
    modeTypedef modeFlag;
    colorSpaceTypedef imageColor;
}vehicleControl_t;


#endif
