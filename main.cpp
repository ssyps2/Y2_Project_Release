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
Mat transformPerspective(std::vector<Point> boundingContour, Mat frame, int x_size, int y_size);
Point findContourCentre(std::vector<cv::Point> contour);
int getMaxAreaContourId(Vector<Vector<cv::Point>> contours);

/* Global Variables of HSV Detection */
const int max_value_H = 180;
const int max_value = 255;
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
string hsv_window_name = "Object Detection";


int main() {
    string image_path = "/home/pengyuan/Pictures/record.jpg";
    string comparePath = "/home/pengyuan/Pictures/Templates/CountShape1.PNG";

    int matching_frame_width = 480, matching_frame_height = 272;

    Mat imageMatch = imread(image_path);
//    cv::resize(imageMatch, imageMatch, cv::Size(matching_frame_width, matching_frame_height));

    Mat imageCompareInput, imageCompare;
    Mat imageHSV, imageMask, imageErode, imageDilate;
    Mat imgHSVCompare, imageMaskCompare;
    Mat transformMatrix, transformImage, imageXOR;
    Mat kernelErode, kernelDilate, kernelDilate2;

    float compare_output;

    Vector <Vector<Point>> contours, contoursCompare;
    Point2f input_coordinate[4], output_coordinate[4];

//    TrackBar_HSV_Detection(image_path);
    //TrackBar_HSV_Detection(comparePath);

    imageCompareInput = imread(comparePath);
    resize(imageCompareInput, imageCompare, Size(matching_frame_width, matching_frame_height));

    kernelErode = getStructuringElement(MORPH_RECT, Size(11, 11));
    kernelDilate = getStructuringElement(MORPH_RECT, Size(6, 6));
    kernelDilate2 = getStructuringElement(MORPH_RECT, Size(7, 7));

    cvtColor(imageMatch, imageHSV, COLOR_BGR2HSV);
//    imshow("HSV", imageHSV);
//    imwrite("/home/pengyuan/Pictures/hsv.jpg",imageHSV);
    inRange(imageHSV, Scalar(157, 157, 114), Scalar(180, 255, 255), imageMask);
//    imshow("mask", imageMask);
//    imwrite("/home/pengyuan/Pictures/mask.jpg",imageMask);

    cvtColor(imageCompare, imgHSVCompare, COLOR_BGR2HSV);
    inRange(imgHSVCompare, Scalar(64, 0, 0), Scalar(180, 255, 255), imageMaskCompare);

    //erode-dilate algorithm
    dilate(imageMask, imageMask, kernelDilate);
    erode(imageMask, imageErode, kernelErode);
//    imshow("erode", imageErode);
//    imwrite("/home/pengyuan/Pictures/erode.jpg",imageErode);

    dilate(imageErode, imageDilate, kernelDilate2);
    imshow("processed", imageDilate);
//    imwrite("/home/pengyuan/Pictures/processed.jpg",imageDilate);

    /* ------------------------------------------------------------------------------------------------- */
    findContours(imageDilate, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
//    drawContours(imageMatch, contours, -1, Scalar(0,255,0), 2);
//    imshow("tree", imageMatch);
//    imwrite("/home/pengyuan/Pictures/tree.jpg",imageMatch);
    int contourID = getMaxAreaContourId(contours);
    Vector<Point> boundingContour = contours[contourID];

    Vector<Point> approx;
    int tri_cnt = 0, rect_cnt = 0, cir_cnt = 0;
    char tri_str[15], rect_str[15], cir_str[15];

    for(int i= 0; i < contours.size(); i++)
    {
        if (i == contourID || contourArea(contours.at(i)) < 120 || contourArea(contours.at(i)) > 50000) continue;

        approxPolyDP(contours[i], approx, 8, true);

        if (approx.size() == 3) tri_cnt++;
        else if (approx.size() == 4) rect_cnt++;
        else if (approx.size() > 4) cir_cnt++;

        std::cout << "Area of contour " << i << ": " << contourArea(contours.at(i)) << std::endl;
        drawContours(imageMatch, Mat(approx), -1, Scalar(0,255,0), 6);
    }

    std::cout << tri_cnt << "," << rect_cnt << "," << cir_cnt << std::endl;

//    drawContours(imageMatch, Mat(approx_cir), -1, Scalar(0,255,0), 8);
    sprintf(tri_str, "Triangle:   %d", tri_cnt);
    sprintf(rect_str, "Rectangle: %d", rect_cnt);
    sprintf(cir_str, "Circle:     %d", cir_cnt);
    putText(imageMatch, tri_str, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
    putText(imageMatch, rect_str, Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
    putText(imageMatch, cir_str, Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
    putText(imageMatch, "Epsilon:   8", Point(10, 150), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
    imshow("approx", imageMatch);
    imwrite("/home/pengyuan/Pictures/cnt_shape_120_e8.jpg",imageMatch);
//    imshow("before", imageMatch);
//    imwrite("/home/pengyuan/Pictures/before.jpg",imageMatch);

//    Point point1{0,0}, point2{0,0}, point3{0,0}, point4{0,0};
//    int point1_cnt = 0, point2_cnt = 0, point3_cnt = 0, point4_cnt = 0;
//    Point contourCentre = findContourCentre(boundingContour);
////    circle(imageMatch, contourCentre, 6, Scalar(0,255,255),6);
////    imshow("circle", imageMatch);
////    imwrite("/home/pengyuan/Pictures/circle.jpg",imageMatch);
//
//    for (int i = 0; i<approx.size(); i++){
//        if(approx[i].x < contourCentre.x){
//            if(approx[i].y < contourCentre.y){
//                point1.x += approx[i].x;
//                point1.y += approx[i].y;
//                point1_cnt++;
//            }
//            else{
//                point4.x += approx[i].x;
//                point4.y += approx[i].y;
//                point4_cnt++;
//            }
//        }
//        else if (approx[i].x >= contourCentre.x){
//            if(approx[i].y < contourCentre.y){
//                point2.x += approx[i].x;
//                point2.y += approx[i].y;
//                point2_cnt++;
//            }
//            else{
//                point3.x += approx[i].x;
//                point3.y += approx[i].y;
//                point3_cnt++;
//            }
//        }
//    }
//
//    point1.x /= point1_cnt; point1.y /= point1_cnt;
//    point2.x /= point2_cnt; point2.y /= point2_cnt;
//    point3.x /= point3_cnt; point3.y /= point3_cnt;
//    point4.x /= point4_cnt; point4.y /= point4_cnt;
//
//    Vector<Point> inputCorner = {point1, point2, point3, point4};
//
//    drawContours(imageMatch, Mat(inputCorner), -1, Scalar(0,255,0), 8);
//
//    findContours(imageMaskCompare, contoursCompare, RETR_TREE, CHAIN_APPROX_SIMPLE);
//    int contourCompareID = getMaxAreaContourId(contoursCompare);
//    Vector<Point> boundingContourCompare = contoursCompare[contourCompareID];
//
//    Vector<Point> approxCompare;
//    approxPolyDP(boundingContourCompare, approxCompare, 5, true);
//    drawContours(imageCompare, Mat(approxCompare), -1, Scalar(0,255,0), 8);
//
//    for (int j = 0; j < 4; j++){
//        output_coordinate[j] = approxCompare[j];
//    }
//
////    transformMatrix = getPerspectiveTransform(input_coordinate, output_coordinate);
////    warpPerspective(imageDilate, transformImage, transformMatrix, Size(imageDilate.cols,imageDilate.rows));
//
//    transformImage = transformPerspective(inputCorner, imageDilate, matching_frame_width, matching_frame_height);
//
//    bitwise_xor(transformImage, imageMaskCompare, imageXOR);
//
//    compare_output = 100.0f * (1 - (float)countNonZero(imageXOR) / (float)(imageXOR.cols * imageXOR.rows));

//    std::cout << compare_output << std::endl;

//    imshow("after", imageMatch);
//    imwrite("/home/pengyuan/Pictures/after.jpg",imageMatch);
//    imshow("maskCompare", imageMaskCompare);
//    imwrite("/home/pengyuan/Pictures/compare.jpg",imageMaskCompare);
//    imshow("transform", transformImage);
//    imwrite("/home/pengyuan/Pictures/transform.jpg",transformImage);
//    imshow("XOR", imageXOR);
//    imwrite("/home/pengyuan/Pictures/xor.jpg",imageXOR);
    waitKey(0);
}

int getMaxAreaContourId(Vector<Vector<cv::Point>> contours){
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        }
    }
    return maxAreaContourId;
}

Mat transformPerspective(std::vector<Point> boundingContour, Mat frame, int x_size, int y_size)
{
    if(boundingContour.size() != 4)
    {
        // Error the contour has too many points. Only 4 are allowed
        Mat emptyMat;
        return emptyMat;
    }

    Mat symbol(y_size,x_size,CV_8UC1, Scalar(0));

    cv::Point2f symbolCorners[4], boundingCorners[4];      // Create (and populate) variables containing the corner locations for the transform
    symbolCorners[0] = Point2f(0,0);
    symbolCorners[1] = Point2f(symbol.cols - 1,0);
    symbolCorners[2] = Point2f(symbol.cols - 1,symbol.rows - 1);
    symbolCorners[3] = Point2f(0,symbol.rows - 1);

    Point contourCentre = findContourCentre(boundingContour);   // To populate the contour corners we need to check the order of the points

    int point1, point2, point3, point4;

    if(boundingContour[0].x > contourCentre.x)
    {
        if(boundingContour[0].y > contourCentre.y)
            point3 = 0;
        else
            point2 = 0;
    }
    else
    {
        if(boundingContour[0].y > contourCentre.y)
            point4 = 0;
        else
            point1 = 0;
    }

    if(boundingContour[1].x > contourCentre.x)
    {
        if(boundingContour[1].y > contourCentre.y)
            point3 = 1;
        else
            point2 = 1;
    }
    else
    {
        if(boundingContour[1].y > contourCentre.y)
            point4 = 1;
        else
            point1 = 1;
    }

    if(boundingContour[2].x > contourCentre.x)
    {
        if(boundingContour[2].y > contourCentre.y)
            point3 = 2;
        else
            point2 = 2;
    }
    else
    {
        if(boundingContour[2].y > contourCentre.y)
            point4 = 2;
        else
            point1 = 2;
    }

    if(boundingContour[3].x > contourCentre.x)
    {
        if(boundingContour[3].y > contourCentre.y)
            point3 = 3;
        else
            point2 = 3;
    }
    else
    {
        if(boundingContour[3].y > contourCentre.y)
            point4 = 3;
        else
            point1 = 3;
    }

    if(point1 + point2 + point3 + point4 != 6)
    {
        //Unable to reconstruct rectangle
        Mat emptyMat;
        return emptyMat;
    }

/*
    boundingCorners[0] = boundingContour[point1];
    boundingCorners[1] = boundingContour[point2];
    boundingCorners[2] = boundingContour[point3];
    boundingCorners[3] = boundingContour[point4];
*/
    int dia_proc = 1; //with dilate pre-process
    int dia_size = 0;

    if (dia_proc)
        dia_size = 4;

    //Bonding the contour with dilate pre-process
    boundingCorners[0].x = boundingContour[point1].x+dia_size;
    boundingCorners[0].y = boundingContour[point1].y+dia_size;
    boundingCorners[1].x = boundingContour[point2].x-dia_size;
    boundingCorners[1].y = boundingContour[point2].y+dia_size;
    boundingCorners[2].x = boundingContour[point3].x-dia_size;
    boundingCorners[2].y = boundingContour[point3].y-dia_size;
    boundingCorners[3].x = boundingContour[point4].x+dia_size;
    boundingCorners[3].y = boundingContour[point4].y-dia_size;

    Mat transformMatrix = cv::getPerspectiveTransform(boundingCorners, symbolCorners); // Calculate the required transform operation
    Mat transformedSymbol(272,480,CV_8UC1,Scalar(0));
    cv::warpPerspective(frame, transformedSymbol, transformMatrix, cv::Size(symbol.cols,symbol.rows));  // Perform the transformation

    return transformedSymbol;
}

Point findContourCentre(std::vector<cv::Point> contour)
{
    Moments foundRegion;    // Variables to store the region moment and the centre point
    Point centre;
    foundRegion = moments(contour, false);      // Calculate the moment for the contour
    centre.x = (foundRegion.m10/foundRegion.m00);  //Calculate the X and Y positions
    centre.y = (foundRegion.m01/foundRegion.m00);

    return centre;
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