#include "visualScan.hpp"


/* Prototype of Functions */
static void trackBarHsvDetection(vehicleControl_t *robot);
static void on_low_H_thresh_trackbar(int, void *);
static void on_high_H_thresh_trackbar(int, void *);
static void on_low_S_thresh_trackbar(int, void *);
static void on_high_S_thresh_trackbar(int, void *);
static void on_low_V_thresh_trackbar(int, void *);
static void on_high_V_thresh_trackbar(int, void *);
Mat transformPerspective(std::vector<Point> boundingContour, Mat frame, int x_size, int y_size);
Point findContourCentre(std::vector<cv::Point> contour);
int getMaxAreaContourId(Vector<Vector<cv::Point>> contours);
static void RGBCallback(int event, int x, int y, int, void *param);
static void HSVCallback(int event, int x, int y, int, void *param);
static int shortcutTrack(Mat inputImage);

extern vehicleControl_t robot;

Mat imageTrack;

const int sample_height = tracking_frame_height * 0.7;

int tx, ty, last_tx;


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

    resizeCamera(tracking_frame_width, tracking_frame_height);
    //resizeCamera(matching_frame_width, matching_frame_height);

    robot->modeFlag = TRACK;

    std::cout << "Webcam Init Success" << std::endl;
}

int midPointCapture(vehicleControl_t *robot){
    Mat hsvImage, binaryImage, pinkChannel, pinkMask, colorChannel;

    float pinkRatio;

    int sample_win; //horizontal sampling windows, 0: previous, 1: now
    int sample_sum = 0, sample_cnt = 0;
    int mid_point = 0, width = 0;  //middle position and width of the black line

    int colorWidth = 0;
    robot->colorMode = 0;    //0:no color, 1:yellow, 2:red, 3:blue, 4:green

    //process each frame of the video
    robot->webcam.read(imageTrack);     //read the image of one frame

    resize(imageTrack, hsvImage, Size(tracking_frame_width, tracking_frame_height));
    resize(imageTrack, binaryImage, Size(tracking_frame_width, tracking_frame_height));
    resize(imageTrack, pinkChannel, Size(tracking_frame_width, tracking_frame_height));

    //filtering the original image
    cvtColor(imageTrack, hsvImage, COLOR_BGR2HSV);
    inRange(hsvImage, Scalar(0, 0, 0), Scalar(179, 255, 70), binaryImage);

    cvtColor(imageTrack, pinkChannel, COLOR_BGR2HSV);
    inRange(pinkChannel, Scalar(143, 132, 0), Scalar(180, 255, 255), pinkMask);

    pinkRatio = 100.0f * (float) countNonZero(pinkMask) / (float) (tracking_frame_width * tracking_frame_height);

    std::cout << "pink ratio: " << pinkRatio << std::endl;

//    if (robot->lastModeFlag == MATCHDONE && robot->modeFlag == TRACK){
//        taskTimer(3);
//    }

    if (pinkRatio >= 2 && pinkRatio <= 20){
        //delay(900);
        robot->modeFlag = DETECTED;
        std::cout << "detected" << std::endl;
        serialPrintf(robot->serial, "#Bafrfr 020 020 020 020");
        delay(300);
    }
    else {
        std::cout << "not detected" << std::endl;
    }

    if (robot->colorMode == 1) inRange(hsvImage, Scalar(17, 42, 46), Scalar(46, 255, 255), colorChannel);  //yellow
    if (robot->colorMode == 2) inRange(hsvImage, Scalar(117, 42, 43), Scalar(180, 255, 255), colorChannel);  //red
    if (robot->colorMode == 3) inRange(hsvImage, Scalar(90, 63, 46), Scalar(125, 255, 255), colorChannel);  //blue
    if (robot->colorMode == 4) inRange(hsvImage, Scalar(40, 168, 50), Scalar(85, 255, 255), colorChannel); //green

    if (robot->colorMode != 0) colorWidth = shortcutTrack(colorChannel);

    for (int pos = 0; pos < tracking_frame_width; pos++) {
        if (colorWidth >= COLORWIDTH){
            sample_win = colorChannel.at<uchar>(Point(pos, sample_height));
        }
        else {
            sample_win = binaryImage.at<uchar>(Point(pos, sample_height));
        }

        if (sample_win > 240){
            sample_sum += pos;
            sample_cnt++;
        }

        if (sample_cnt == 0){
            mid_point = 0;
            width = 0;
        }
        else {
            mid_point = (int)(sample_sum / sample_cnt);
            width = sample_cnt;
        }
    }

    //std::cout << "width: " << width << std::endl;

    if (width >= 60 && binaryImage.at<uchar>(Point(tracking_frame_width-1, sample_height)) > 240){
        last_tx = tx;
        tx = (int)(mid_point + width/2);
    }
    else if (width >= 60 && binaryImage.at<uchar>(Point(0, sample_height)) > 240){
        last_tx = tx;
        tx = (int)(mid_point - width/2);
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

    //modeCheck(robot);

    //setMouseCallback("Processed Video", RGBCallback, &imageTrack);

    std::cout << "Processed" << std::endl;
    //imshow("Processed Video", imageTrack);
    //imshow("Binary", hsvImage);
    //imshow("pink channel", pinkMask);
    //waitKey(1);

    return tx;
}

static int shortcutTrack(Mat inputImage){
    int sample_win[2];
    int edge_first = 0, edge_last = 0;  //edge position
    int mid_point = 0, width = 0;

    threshold(inputImage, inputImage, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

    //initialize the sampling windows
    sample_win[1] = inputImage.at<uchar>(Point(0, sample_height));

    //find the center position of the tracking line for one frame
    for (int pos = 0; pos < tracking_frame_width; pos++) {
        sample_win[0] = sample_win[1];
        sample_win[1] = inputImage.at<uchar>(Point(pos, sample_height));    //update the sampling window

        if (sample_win[1] - sample_win[0] < -240) {
            edge_first = pos;
        } else if (sample_win[1] - sample_win[0] > 240) {
            edge_last = pos;
            if (edge_last - edge_first > 15) {
                break;
            }
        } else if (pos == tracking_frame_width-1 && inputImage.at<uchar>(Point(tracking_frame_width-1, sample_height)) == 0) {
            edge_last = tracking_frame_width-1;
        } else if (pos == 0 && inputImage.at<uchar>(Point(0, sample_height)) == 0) {
            edge_first = 0;
        }
    }

    mid_point = (edge_first + edge_last) / 2;
    width = edge_last - edge_first;

    return width;
}

float visualMatch(vehicleControl_t *robot, const string comparePath){
    Mat imageMatch, imageCompare;
    Mat imageHSV, imageMask, imageErode, imageDilate;
    Mat imgHSVCompare, imageMaskCompare;
    Mat transformMatrix, transformImage, imageXOR;

    Mat yellowChannel, redChannel, blueChannel, greenChannel;
    float yellowRatio, redRatio, blueRatio, greenRatio;

    robot->webcam.read(imageMatch);

    Mat imageCompareInput = imread(comparePath);
    resize(imageCompareInput, imageCompare, Size(matching_frame_width, matching_frame_height));

    /*char keyRecord = (char) waitKey(1);
    if (keyRecord == 'c') {
        imwrite("/home/pi/Pictures/record.png", imageMatch);
        std::cout << "Record success" << std::endl;
    }*/

    //trackBarHsvDetection(robot);

    Mat kernelErode = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat kernelDilate = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat kernelDilate2 = getStructuringElement(MORPH_RECT, Size(4, 4));

    cvtColor(imageMatch, imageHSV, COLOR_BGR2HSV);
    inRange(imageHSV, Scalar(157, 116, 102), Scalar(180, 255, 255), imageMask);

    cvtColor(imageCompare, imgHSVCompare, COLOR_BGR2HSV);
    inRange(imgHSVCompare, Scalar(64, 0, 0), Scalar(180, 255, 255), imageMaskCompare);

    Vector <Vector<Point>> colorContours;

    inRange(imageHSV, Scalar(17, 42, 46), Scalar(46, 255, 255), yellowChannel);  //yellow
    inRange(imageHSV, Scalar(117, 42, 43), Scalar(180, 255, 255), redChannel);  //red
    inRange(imageHSV, Scalar(90, 63, 46), Scalar(125, 255, 255), blueChannel);  //blue
    inRange(imageHSV, Scalar(40, 168, 50), Scalar(85, 255, 255), greenChannel); //green

    yellowRatio = 100.0f * (float) countNonZero(yellowRatio) / (float) (matching_frame_width * matching_frame_height);
    redRatio = 100.0f * (float) countNonZero(redRatio) / (float) (matching_frame_width * matching_frame_height);
    blueRatio = 100.0f * (float) countNonZero(blueRatio) / (float) (matching_frame_width * matching_frame_height);
    greenRatio = 100.0f * (float) countNonZero(greenRatio) / (float) (matching_frame_width * matching_frame_height);



//    if (yellowRatio > )

    //erode-dilate algorithm
    dilate(imageMask, imageMask, kernelDilate);
    erode(imageMask, imageErode, kernelErode);
    dilate(imageErode, imageDilate, kernelDilate2);

    Vector <Vector<Point>> contours, contoursCompare;
    Vector <Vec4i> hierarchy;

    findContours(imageDilate, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    int contourID = getMaxAreaContourId(contours);
    Vector<Point> boundingContour = contours[contourID];

    Vector<Point> approx;
    approxPolyDP(boundingContour, approx, 5, true);
    Point point1{0,0}, point2{0,0}, point3{0,0}, point4{0,0};
    int point1_cnt = 0, point2_cnt = 0, point3_cnt = 0, point4_cnt = 0;
    Point contourCentre = findContourCentre(boundingContour);

    for (int i = 0; i<approx.size(); i++){
        if(approx[i].x < contourCentre.x){
            if(approx[i].y < contourCentre.y){
                point1.x += approx[i].x;
                point1.y += approx[i].y;
                point1_cnt++;
            }
            else{
                point4.x += approx[i].x;
                point4.y += approx[i].y;
                point4_cnt++;
            }
        }
        else if (approx[i].x >= contourCentre.x){
            if(approx[i].y < contourCentre.y){
                point2.x += approx[i].x;
                point2.y += approx[i].y;
                point2_cnt++;
            }
            else{
                point3.x += approx[i].x;
                point3.y += approx[i].y;
                point3_cnt++;
            }
        }
    }

    if (point1_cnt == 0 || point2_cnt == 0 || point3_cnt == 0 || point4_cnt == 0){
        throw std::exception();
    }
    point1.x /= point1_cnt; point1.y /= point1_cnt;
    point2.x /= point2_cnt; point2.y /= point2_cnt;
    point3.x /= point3_cnt; point3.y /= point3_cnt;
    point4.x /= point4_cnt; point4.y /= point4_cnt;

    /*std::cout << "P1 " << point1.x << "," << point1.y << std::endl;
    std::cout << "P2 " << point2.x << "," << point2.y << std::endl;
    std::cout << "P3 " << point3.x << "," << point3.y << std::endl;
    std::cout << "P4 " << point4.x << "," << point4.y << std::endl;*/

    Vector<Point> inputCorner = {point1, point2, point3, point4};

    drawContours(imageMatch, Mat(inputCorner), -1, Scalar(0,255,0), 8);
//    drawContours(imageMatch, Mat(boundingContour), -1, Scalar(0,255,0), 3);

    findContours(imageMaskCompare, contoursCompare, RETR_TREE, CHAIN_APPROX_SIMPLE);
    int contourCompareID = getMaxAreaContourId(contoursCompare);
    Vector<Point> boundingContourCompare = contoursCompare[contourCompareID];

    Vector<Point> approxCompare;
    approxPolyDP(boundingContourCompare, approxCompare, 5, true);
    drawContours(imageCompare, Mat(approxCompare), -1, Scalar(0,255,0), 8);

    Point2f input_coordinate[4], output_coordinate[4];

    for (int j = 0; j < 4; j++){
        output_coordinate[j] = approxCompare[j];
    }

//    transformMatrix = getPerspectiveTransform(input_coordinate, output_coordinate);
//    warpPerspective(imageDilate, transformImage, transformMatrix, Size(imageDilate.cols,imageDilate.rows));

    transformImage = transformPerspective(inputCorner, imageDilate, matching_frame_width, matching_frame_height);

    bitwise_xor(transformImage, imageMaskCompare, imageXOR);

    float compare_output = 100.0f * (1 - (float) countNonZero(imageXOR) / (float) (imageXOR.cols * imageXOR.rows));

    //setMouseCallback("origin", RGBCallback, &imageMatch);
    //setMouseCallback("origin", HSVCallback, &imageHSV);

    std::cout << "compare output: " << compare_output << std::endl;

    //imshow("origin", imageMatch);
    imshow("mask", imageMask);
    imshow("transform", transformImage);
    //imshow("compareMask", imageMaskCompare);
    //imshow("XOR", imageXOR);

    waitKey(1);

    return compare_output;
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
    Mat transformedSymbol(240,320,CV_8UC1,Scalar(0));
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

static void RGBCallback(int event, int x, int y, int, void *param){
    Mat *imgRGB = (Mat *)param;
    int areaDownX, areaDownY, areaUpX, areaUpY;

    if (event == EVENT_LBUTTONDOWN){
        robot.imageColor.blue = imgRGB->at<Vec3b>(Point(x, y))[0];
        robot.imageColor.green = imgRGB->at<Vec3b>(Point(x, y))[1];
        robot.imageColor.red = imgRGB->at<Vec3b>(Point(x, y))[2];

        std::cout << "R: " << (int)robot.imageColor.red << " G: " << (int)robot.imageColor.green
                                            << " B: " << (int)robot.imageColor.blue << std::endl;
    }
    else if (event == EVENT_RBUTTONDOWN){
        areaDownX = x;
        areaDownY = y;

        std::cout << "Right Button Down" << std::endl;
    }
    else if (event == EVENT_RBUTTONUP){
        areaUpX = x;
        areaUpY = y;



        std::cout << "Right Button Up" << std::endl;
    }
}

static void HSVCallback(int event, int x, int y, int, void *param){
    Mat *imgHSV = (Mat *)param;
    int areaDownX, areaDownY, areaUpX, areaUpY;

    if (event == EVENT_LBUTTONDOWN){
        robot.imageColor.hue = imgHSV->at<Vec3b>(Point(x, y))[0];
        robot.imageColor.saturate = imgHSV->at<Vec3b>(Point(x, y))[1];
        robot.imageColor.value = imgHSV->at<Vec3b>(Point(x, y))[2];

        std::cout << "H: " << (int)robot.imageColor.hue << " S: " << (int)robot.imageColor.saturate
                                            << " V: " << (int)robot.imageColor.value << std::endl;
    }
    else if (event == EVENT_RBUTTONDOWN){
        areaDownX = x;
        areaDownY = y;

        std::cout << "Right Button Down" << std::endl;
    }
    else if (event == EVENT_RBUTTONUP){
        areaUpX = x;
        areaUpY = y;



        std::cout << "Right Button Up" << std::endl;
    }
}

void resizeCamera(int width, int height){
    robot.webcam.set(CAP_PROP_FRAME_WIDTH, width);
	robot.webcam.set(CAP_PROP_FRAME_HEIGHT, height);
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

        //cvtColor(frame, frame_threshold, COLOR_BGR2GRAY);
        //threshold(frame_threshold, frame_threshold, low_S, high_S, THRESH_BINARY);

        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);

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
