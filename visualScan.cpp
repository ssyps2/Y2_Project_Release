#include <cstdlib>
#include "visualScan.hpp"


/* Prototype of Functions */
static void trackBarHsvDetection(vehicleControl_t *robot);
static void on_low_H_thresh_trackbar(int, void *);
static void on_high_H_thresh_trackbar(int, void *);
static void on_low_S_thresh_trackbar(int, void *);
static void on_high_S_thresh_trackbar(int, void *);
static void on_low_V_thresh_trackbar(int, void *);
static void on_high_V_thresh_trackbar(int, void *);
static void resizeCamera(vehicleControl_t *robot, int width, int height);
static void modeCheck(vehicleControl_t *robot);

int matching_frame_width = 1920, matching_frame_height = 1080;
int tracking_frame_width = 480, tracking_frame_height = 272;
string comparePath = "/home/pi/Pictures/Template/CountShape2.png";

Mat imageMatch, imageTrack;
Mat imageCompareInput, imageCompare;
Mat imageHSV, imageMask, imageErode, imageDilate;
Mat imgHSVCompare, imageMaskCompare;
Mat transformMatrix, transformImage, imageXOR;
Mat kernelErode, kernelDilate;

//double contour_area;
float compare_output;

Vector <Vector<Point>> contours, contoursCompare;
Vector <Vector<Point>> approx, approxCompare;
Point2f input_coordinate[4], output_coordinate[4];

Mat gray_image, pinkChannel, pinkMask;
int last_tx;


/* Global Variables used for HSV Detection */
const int max_value_H = 180;
const int max_value = 255;
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
const string hsv_window_name = "Object Detection";



void webcamInit(vehicleControl_t *robot){
    robot->webcam.open(0);     //open the webcam

    if (!robot->webcam.isOpened())
	{
		std::cout << "ERROR: Unable to open the camera" << std::endl;
		exit(-2);
	}

    resizeCamera(robot, tracking_frame_width, tracking_frame_height);
    //resizeCamera(matching_frame_width, matching_frame_height);

    robot->modeFlag = TRACK;
}

float visualMatch(vehicleControl_t *robot) {
    robot->webcam.read(imageMatch);

    /*char photo = (char) waitKey(30);
    if (photo == 'c') {
        imwrite("/home/pi/Pictures/record.png", imageMatch);
        puts("saved!\n");
    }*/

    imageCompareInput = imread(comparePath);
    resize(imageCompareInput, imageCompare, Size(matching_frame_width, matching_frame_height));

    //trackBarHsvDetection();

    kernelErode = getStructuringElement(MORPH_RECT, Size(8, 8));
    kernelDilate = getStructuringElement(MORPH_RECT, Size(3, 3));

    cvtColor(imageMatch, imageHSV, COLOR_BGR2HSV);
    cvtColor(imageHSV, imageHSV, COLOR_BGR2GRAY);
    threshold(imageHSV, imageHSV, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
    inRange(imageHSV, Scalar(0, 0, 0), Scalar(156, 255, 255), imageMask);
    //threshold(imageMask, imageMask, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

    cvtColor(imageCompare, imgHSVCompare, COLOR_BGR2HSV);
    cvtColor(imgHSVCompare, imgHSVCompare, COLOR_BGR2GRAY);
    threshold(imgHSVCompare, imgHSVCompare, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
    inRange(imgHSVCompare, Scalar(0, 0, 0), Scalar(143, 255, 255), imageMaskCompare);
    //threshold(imageMaskCompare, imageMaskCompare, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

    //erode-dilate algorithm
    erode(imageMask, imageErode, kernelErode);
    dilate(imageErode, imageDilate, kernelDilate);
    findContours(imageDilate, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

//    contour_area = contourArea(contours[0]);

    approx.resize(contours.size());

    for (int i = 0; i < contours.size(); i++) {
        approx[i].resize(contours[i].size());
        approxPolyDP(contours[i], approx[i], 5, true);
    }
    drawContours(imageMatch, Mat(approx[0]), -1, Scalar(0, 255, 0), 8);

    //find contours of compared image
    findContours(imageMaskCompare, contoursCompare, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    approxCompare.resize(contoursCompare.size());

    for (int i = 0; i < contoursCompare.size(); i++) {
        approxCompare[i].resize(contoursCompare[i].size());
        approxPolyDP(contoursCompare[i], approxCompare[i], 5, true);
    }
    drawContours(imageCompare, Mat(approxCompare[0]), -1, Scalar(0, 255, 0), 8);


    for (int j = 0; j < 4; j++) {
        input_coordinate[j] = approx[0][j];
        output_coordinate[j] = approxCompare[0][j];
    }

    transformMatrix = getPerspectiveTransform(input_coordinate, output_coordinate);
    warpPerspective(imageDilate, transformImage, transformMatrix, Size(matching_frame_width, matching_frame_height));

    //XOR
    bitwise_xor(transformImage, imageMaskCompare, imageXOR);

    compare_output = 100.0f * (1 - (float) countNonZero(imageXOR) / (float) (matching_frame_width * matching_frame_height));

    //modeCheck(robot);

    std::cout << compare_output << std::endl;

    //imshow("erode", imageErode);
    imshow("origin", imageMatch);
    imshow("mask", imageMask);
    imshow("transform", transformImage);
    //imshow("compareMask", imageMaskCompare);
    imshow("compare",imageCompare);
    //imshow("HSV", imageHSV);
    //imshow("XOR", imageXOR);

    waitKey(1);

    return compare_output;
}

int midPointCapture(vehicleControl_t *robot){
    float pinkRatio;

    int tx, ty;     //coordinate information
    char location[10];

    const int sample_height = tracking_frame_height * 0.5;  //sampling height of the image
    int sample_win[2]; //horizontal sampling windows, 0: previous, 1: now
    int sample_sum = 0, sample_cnt = 0;
    int edge_first, edge_last;  //edge position
    int mid_point = 0, width = 0;  //middle position and width of the black line

    uchar scanMode = 0;

    //process each frame of the video
    robot->webcam.read(imageTrack);     //read the image of one frame

    resize(imageTrack, gray_image, Size(tracking_frame_width, tracking_frame_height));
    resize(imageTrack, pinkChannel, Size(tracking_frame_width, tracking_frame_height));

    //filtering the original image
    cvtColor(imageTrack, gray_image, COLOR_BGR2HSV);
    inRange(gray_image, Scalar(0, 0, 0), Scalar(179, 255, 90), gray_image);
    if (scanMode == 0){
        threshold(gray_image, gray_image, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
    }

    cvtColor(imageTrack, pinkChannel, COLOR_BGR2HSV);
    cvtColor(pinkChannel, pinkChannel, COLOR_BGR2GRAY);
    threshold(pinkChannel, pinkChannel, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
    inRange(pinkChannel, Scalar(0, 0, 0), Scalar(156, 255, 255), pinkMask);

    pinkRatio = 100.0f * (float) countNonZero(pinkMask) / (float) (tracking_frame_width * tracking_frame_height);

    //std::cout << pinkRatio << std::endl;

    /*if (pinkRatio >= 8 && pinkRatio <= 25){
        robot->modeFlag = DETECTED;
    }
    else {
        robot->modeFlag = TRACK;
    }*/

    if (scanMode == 0){
        //initialize the sampling windows
        sample_win[1] = gray_image.at<uchar>(Point(0, sample_height));

        //reset the edge position of the frame to be scanned
        edge_first = edge_last = 0;

        //find the center position of the tracking line for one frame
        for (int pos = 0; pos < tracking_frame_width; pos++) {
            sample_win[0] = sample_win[1];
            sample_win[1] = gray_image.at<uchar>(Point(pos, sample_height));    //update the sampling window

            if (sample_win[1] - sample_win[0] < -240) {
                edge_first = pos;
            } else if (sample_win[1] - sample_win[0] > 240) {
                edge_last = pos;
                if (edge_last - edge_first > 15) {
                    break;
                }
            } else if (pos == tracking_frame_width-1 && gray_image.at<uchar>(Point(tracking_frame_width-1, sample_height)) == 0) {
                edge_last = tracking_frame_width-1;
            } else if (pos == 0 && gray_image.at<uchar>(Point(0, sample_height)) == 0) {
                edge_first = 0;
            }
        }

        //mid-point location and line width calculation
        mid_point = (edge_first + edge_last) / 2;
        width = edge_last - edge_first;
    }
    else if (scanMode == 1){
        for (int pos = 0; pos < tracking_frame_width; pos++) {
            sample_win[2] = gray_image.at<uchar>(Point(pos, sample_height));
            if (sample_win[2] > 240){
                sample_sum += pos;
                sample_cnt++;
            }
            if (sample_cnt < 5){
                mid_point = 0;
                width = 0;
            }
            else {
                mid_point = (int)(sample_sum / sample_cnt);
                width = sample_cnt;
            }
        }
    }

    std::cout << "width: " << width << std::endl;

    if (width >= 60 && gray_image.at<uchar>(Point(tracking_frame_width-1, sample_height)) == 0){
        last_tx = tx;
        if (scanMode == 0){
            tx = edge_last;
        }
        else if (scanMode == 1){
            tx = (int)(mid_point + width/2);
        }
    }
    else if (width >= 60 && gray_image.at<uchar>(Point(0, sample_height)) == 0){
        last_tx = tx;
        if (scanMode == 0){
            tx = edge_first;
        }
        else if (scanMode == 1){
            tx = (int)(mid_point - width/2);
        }
    }
    else if (width > 15 && width < 60){
        last_tx = tx;
        tx = mid_point;
    }
    else if (width == 0){
        tx = last_tx;
    }

    ty = sample_height;

    //put point
    circle(imageTrack, Point(tx, ty), 2, Scalar(0, 0, 255), 3);

    //put location
    sprintf(location, "(%d, %d)", tx, ty);
    putText(imageTrack, location, Point(tx-50, sample_height-15),
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);

    //modeCheck(robot);

    //display the processed video
    imshow("Processed Video", imageTrack);
    imshow("Binary", gray_image);
    //imshow("pink channel", pinkMask);
    waitKey(1);

    return tx;
}

static void modeCheck(vehicleControl_t *robot){
    if(robot->lastModeFlag == DETECTED && robot->modeFlag == MATCH){
        resizeCamera(robot, matching_frame_width, matching_frame_height);
    }
    else if (robot->lastModeFlag == MATCHDONE && robot->modeFlag == TRACK){
        resizeCamera(robot, tracking_frame_width, tracking_frame_height);
    }
}

static void resizeCamera(vehicleControl_t *robot, int width, int height){
    robot->webcam.set(CAP_PROP_FRAME_WIDTH, width);
	robot->webcam.set(CAP_PROP_FRAME_HEIGHT, height);
}

static void trackBarHsvDetection(vehicleControl_t *robot){
    Mat frame, frame_HSV, frame_threshold;
    namedWindow(hsv_window_name);

    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", hsv_window_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", hsv_window_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", hsv_window_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", hsv_window_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", hsv_window_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", hsv_window_name, &high_V, max_value, on_high_V_thresh_trackbar);

    while (true) {
        robot->webcam >> frame;
        //frame = imread("/home/pi/Pictures/Template/CountShape2.png");
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        // Show the frames
        imshow(hsv_window_name, frame_threshold);
        char key = (char) waitKey(30);
        if (key == 'q' || key == 27) {
            break;
        }
    }
}


/* Callback Functions of HSV Detection */
static void on_low_H_thresh_trackbar(int, void *) {
    low_H = min(high_H - 1, low_H);
    setTrackbarPos("Low H", hsv_window_name, low_H);
}

static void on_high_H_thresh_trackbar(int, void *) {
    high_H = max(high_H, low_H + 1);
    setTrackbarPos("High H", hsv_window_name, high_H);
}

static void on_low_S_thresh_trackbar(int, void *) {
    low_S = min(high_S - 1, low_S);
    setTrackbarPos("Low S", hsv_window_name, low_S);
}

static void on_high_S_thresh_trackbar(int, void *) {
    high_S = max(high_S, low_S + 1);
    setTrackbarPos("High S", hsv_window_name, high_S);
}

static void on_low_V_thresh_trackbar(int, void *) {
    low_V = min(high_V - 1, low_V);
    setTrackbarPos("Low V", hsv_window_name, low_V);
}

static void on_high_V_thresh_trackbar(int, void *) {
    high_V = max(high_V, low_V + 1);
    setTrackbarPos("High V", hsv_window_name, high_V);
}