#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

/* Namespace Usage */
using namespace cv;
using std::string;

template <typename Tp, typename Alloc = std::allocator<Tp>>
using Vector = std::vector<Tp, Alloc>;

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
    string image_path = "/home/pengyuan/Pictures/record.png";
    string comparePath = "/home/pengyuan/Pictures/Templates/CountShape2.png";

    int matching_frame_width = 480, matching_frame_height = 360;

    Mat imageMatch(matching_frame_width, matching_frame_height, CV_8UC3);
    Mat imageCompareInput, imageCompare;
    Mat imageHSV, imageMask, imageErode, imageDilate;
    Mat imgHSVCompare, imageMaskCompare;
    Mat transformMatrix, transformImage, imageXOR;
    Mat kernelErode, kernelDilate;

    double contour_area;
    float compare_output;

    Vector <Vector<Point>> contours, contoursCompare;
    Vector <Vector<Point>> approx, approxCompare;
    Point2f input_coordinate[4], output_coordinate[4];

    //TrackBar_HSV_Detection(image_path);
    //TrackBar_HSV_Detection(comparePath);

    imageCompareInput = imread(comparePath);
    resize(imageCompareInput, imageCompare, Size(matching_frame_width, matching_frame_height));

    kernelErode = getStructuringElement(MORPH_RECT, Size(6, 6));
    kernelDilate = getStructuringElement(MORPH_RECT, Size(3, 3));

    cvtColor(imageMatch, imageHSV, COLOR_BGR2HSV);
    inRange(imageHSV, Scalar(157, 157, 114), Scalar(180, 255, 255), imageMask);

    cvtColor(imageCompare, imgHSVCompare, COLOR_BGR2HSV);
    inRange(imgHSVCompare, Scalar(64, 0, 0), Scalar(180, 255, 255), imageMaskCompare);

    //erode-dilate algorithm
    erode(imageMask, imageErode, kernelErode);
    dilate(imageErode, imageDilate, kernelDilate);
    findContours(imageDilate, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    //contour_area = contourArea(contours[0]);

    approx.resize(contours.size());

    for (int i = 0; i<contours.size(); i++)
    {
        approx[i].resize(contours[i].size());
        approxPolyDP(contours[i], approx[i], 5, true);
    }
    drawContours(imageMatch, Mat(approx[0]), -1, Scalar(0,255,0), 8);

    //find contours of compared image
    findContours(imageMaskCompare, contoursCompare, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    approxCompare.resize(contoursCompare.size());

    for (int i = 0; i<contoursCompare.size(); i++)
    {
        approxCompare[i].resize(contoursCompare[i].size());
        approxPolyDP(contoursCompare[i], approxCompare[i], 5, true);
    }
    drawContours(imageCompare, Mat(approxCompare[0]), -1, Scalar(0,255,0), 8);

    for (int j = 0; j < 4; j++){
        input_coordinate[j] = approx[0][j];
        output_coordinate[j] = approxCompare[0][j];
    }

    transformMatrix = getPerspectiveTransform(input_coordinate, output_coordinate);
    warpPerspective(imageDilate, transformImage, transformMatrix, Size(imageDilate.cols,imageDilate.rows));

    bitwise_xor(transformImage, imageMaskCompare, imageXOR);

    compare_output = 100.0f * (1 - (float)countNonZero(imageXOR) / (float)(imageXOR.cols * imageXOR.rows));

    std::cout << compare_output << std::endl;

    imshow("origin", imageMatch);
//    imshow("what", transformImage);
//    imshow("compare", imageMaskCompare);
//    imshow("XOR", imageXOR);
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