#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>

/* Namespace Usage */
using namespace std;
using namespace cv;

/* Prototype of Functions */
void TrackBar_HSV_Detection(const string& file_path);
static void on_low_H_thresh_trackbar(int, void *);
static void on_high_H_thresh_trackbar(int, void *);
static void on_low_S_thresh_trackbar(int, void *);
static void on_high_S_thresh_trackbar(int, void *);
static void on_low_V_thresh_trackbar(int, void *);
static void on_high_V_thresh_trackbar(int, void *);

/* Global Variables of HSV Detection */
const int max_value_H = 180;
const int max_value = 255;
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
string hsv_window_name = "Object Detection";

/* Start of Main */
int main() {
    string image_path = "/home/pengyuan/Pictures/contour3.png";
    string compare_path = "/home/pengyuan/Pictures/contour4.png";

//    TrackBar_HSV_Detection(image_path);

    Mat img = imread(image_path);
    Mat imgCmp_Input = imread(compare_path);
    Mat imgCmp;
    resize(imgCmp_Input, imgCmp, Size(img.cols, img.rows));
    Mat imgHSV, mask, imgEro, imgDil;
    Mat imgHSV_Cmp, mask_Cmp;
    Mat matTransform, imgTransform;
    Mat img_XOR;

    double contour_area, approx_area;
    float compare_output;

    vector<vector<Point>> contours, contours_Cmp;
    vector<vector<Point>> approx, approx_Cmp;
    Point2f input_coordinate[4], output_coordinate[4];

    Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(3,3));
    Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(3,3));

    cvtColor(img, imgHSV, COLOR_BGR2HSV);
    inRange(imgHSV, Scalar(132, 50, 58), Scalar(172, 255, 255), mask);

    cvtColor(imgCmp, imgHSV_Cmp, COLOR_BGR2HSV);
    inRange(imgHSV_Cmp, Scalar(132, 50, 58), Scalar(172, 255, 255), mask_Cmp);

    //erode-dilate algorithm
    erode(mask, imgEro, kernel_erode);
    dilate(imgEro, imgDil, kernel_dilate);
    findContours(imgDil, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//    contour_area = contourArea(contours[0], false);

    approx.resize(contours.size());

    for (int i = 0; i<contours.size(); i++)
    {
        approx[i].resize(contours[i].size());
        approxPolyDP(contours[i], approx[i], 5, true);
    }
    drawContours(img, Mat(approx[0]), -1, Scalar(0,255,0), 8);

    //find contours of compared image
    findContours(mask_Cmp, contours_Cmp, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    approx_Cmp.resize(contours_Cmp.size());

    for (int i = 0; i<contours_Cmp.size(); i++)
    {
        approx_Cmp[i].resize(contours_Cmp[i].size());
        approxPolyDP(contours_Cmp[i], approx_Cmp[i], 5, true);
    }
    drawContours(imgCmp, Mat(approx_Cmp[0]), -1, Scalar(0,255,0), 8);

    for (int j = 0; j < 4; j++){
        input_coordinate[j] = approx[0][j];
        output_coordinate[j] = approx_Cmp[0][j];
    }

    matTransform = getPerspectiveTransform(input_coordinate, output_coordinate);
    warpPerspective(imgDil, imgTransform, matTransform, Size(imgDil.cols,imgDil.rows));

    //XOR
    bitwise_xor(imgTransform, mask_Cmp, img_XOR);

    compare_output = 100.0f * (1 - (float)countNonZero(img_XOR) / (float)(img_XOR.cols * img_XOR.rows));

    cout << compare_output << endl;

//    imshow("what", imgTransform);
//    imshow("compare", mask_Cmp);
    imshow("XOR", img_XOR);
    waitKey(0);
}

/* Declaration of Functions */
void TrackBar_HSV_Detection(const string& file_path){
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
        frame = imread(file_path);
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        // Show the frames
        imshow(hsv_window_name, frame_threshold);
        char key = (char) waitKey(30);
        if (key == 'q' || key == 27)
        {
            break;
        }
    }
}

/* Callback Functions of HSV Detection */
static void on_low_H_thresh_trackbar(int, void *){
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", hsv_window_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *){
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", hsv_window_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *){
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", hsv_window_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *){
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", hsv_window_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *){
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", hsv_window_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *){
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", hsv_window_name, high_V);
}