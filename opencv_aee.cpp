#include "opencv_aee.hpp"


Point templateMatch(Mat frame, Mat templ, int method, double threshold)
{
    Mat result;
    result.create(frame.rows - templ.rows + 1, frame.cols - templ.cols + 1, CV_32FC1);

    double minVal, maxVal;
    Point minLoc, maxLoc, matchLoc;

    cv::matchTemplate(frame, templ, result, method);

    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
    //printf("%f - ", maxVal);
	//printf("%f - ", minVal);

    if((maxVal >= threshold) || ((minVal <= threshold) && (method == TM_SQDIFF || method == TM_SQDIFF_NORMED)))
    {
        cv::normalize(result, result, 0, 1, NORM_MINMAX, -1);
        cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

        if(method  == TM_SQDIFF || method == TM_SQDIFF_NORMED)
            matchLoc = minLoc;
        else
            matchLoc = maxLoc;

        matchLoc.x += templ.cols/2;
        matchLoc.y += templ.rows/2;
    }
    else
    {
		// No valid match found
        matchLoc.x = -1;
        matchLoc.y = -1;
    }

    return matchLoc;
}

Point featureMatch(Mat frame, Mat object, int minHessian, float scalingFactor, int goodMatchLimit)
{
    // Configure the variables the extract the keypoints and descriptors using SURF
    Ptr<SURF> detector = SURF::create(minHessian);

    std::vector<KeyPoint> kpObject, kpFrame;
    Mat descObject, descFrame;

    detector->detectAndCompute(object, Mat(), kpObject, descObject);
    detector->detectAndCompute(frame, Mat(), kpFrame, descFrame);

	if(descFrame.empty() || descObject.empty())
	{
		// Unable to find features in one (or both) of the images
		Point error(-4,-4);
		return error;
	}

    // Match the descriptors using FLANN
    FlannBasedMatcher matcher;
    std::vector<DMatch> matches;
    matcher.match(descObject, descFrame, matches);

    if(matches.size() == 0)
    {
        // No match found
        Point error(-1,-1);
        return error;
    }

    // Find the min and max distance values for the matches
    double min_dist = DBL_MAX, max_dist = -DBL_MAX;     // Initialise the variables to their corresponding min and max type values
    for(int i = 0; i < descObject.rows; i++)
    {
        double dist = matches[i].distance;

        if(dist < min_dist)
            min_dist = dist;
        if(dist > max_dist)
            max_dist = dist;
    }

    // Create a subset of "good" matches based on the distance values
    std::vector<DMatch> goodMatches;
    for(int i = 0; i < descObject.rows; i++)
    {
        if(matches[i].distance < scalingFactor*min_dist)        // A distance scale the distance to limit "good" matches
            goodMatches.push_back(matches[i]);
    }

    if(goodMatches.size() < goodMatchLimit)
    {
        // No (or not enough) good matches found
        Point error(-2,-2);
        return error;
    }

    // Separately store the "good" match keypoints
    std::vector<Point2f> gkpObject, gkpFrame;
    for(int i = 0; i < goodMatches.size(); i++)
    {
        gkpObject.push_back(kpObject[goodMatches[i].queryIdx].pt);
        gkpFrame.push_back(kpFrame[goodMatches[i].trainIdx].pt);
    }

    Mat H = findHomography(gkpObject, gkpFrame, RANSAC);    // Find the transform between the two planes formed by the keypoints
	if(H.empty()) // should this be isempty()?
	{
		// Unable to find homography, could return a match (or calc the centre of the good matches)
		Point error(-3,-3);
		return error;
	}
    // Create an array containing the corners of the object image
    std::vector<Point2f> corObject(4);
    corObject[0] = cvPoint(0,0);
    corObject[1] = cvPoint(object.cols,0);
    corObject[2] = cvPoint(object.cols,object.rows);
    corObject[3] = cvPoint(0,object.rows);

    // Create an array containing the corners of the object in the frame and use the transformation to populate
    std::vector<Point2f> corFrame(4);
    perspectiveTransform(corObject, corFrame, H);

    // Optionally draw the matched features and a box around the found object
    if(FEATURE_MATCH_DRAW)
    {
        Mat imgMatches;
        cv::drawMatches(object, kpObject, frame, kpFrame, goodMatches, imgMatches, Scalar::all(-1), Scalar::all(-1), std::vector<char>());

        line( imgMatches, corFrame[0] + Point2f( object.cols, 0), corFrame[1] + Point2f( object.cols, 0), Scalar(0, 255, 0), 4 );
        line( imgMatches, corFrame[1] + Point2f( object.cols, 0), corFrame[2] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
        line( imgMatches, corFrame[2] + Point2f( object.cols, 0), corFrame[3] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
        line( imgMatches, corFrame[3] + Point2f( object.cols, 0), corFrame[0] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );

        namedWindow("FEATURE_MATCH");
        imshow("FEATURE_MATCH", imgMatches);
        waitKey(0);
    }

    // Calculate the centre point of the box
    float minx = FLT_MAX, maxx = -FLT_MAX, miny = FLT_MAX, maxy = -FLT_MAX;
    for(int i = 0; i < corFrame.size(); i++)
    {
        if(corFrame[i].x < minx)
            minx = corFrame[i].x;
        if(corFrame[i].x > maxx)
            maxx = corFrame[i].x;

        if(corFrame[i].y < miny)
            miny = corFrame[i].y;
        if(corFrame[i].y > maxy)
            maxy = corFrame[i].y;
    }
    Point centre((int)(minx + (maxx-minx)/2), (int)(miny + (maxy - miny)/2));

    return centre;
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


Mat readImage(const char* imageLoc)
{
	Mat image = imread(imageLoc, IMREAD_COLOR);

	if(!image.data)
	{
		std::cout<< "ERROR: Unable to read image data" << std::endl;
		errorTrap();
	}

	return image;
}

void errorTrap(void)
{
    while(1);
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

    //Bonding the contour with dialate pre-process
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

float compareImages(Mat cameraImage, Mat librarySymbol)
{
    float matchPercent = 100 - (100/((float)librarySymbol.cols*(float)librarySymbol.rows) * (2*(float)countNonZero(librarySymbol^cameraImage))); // An incorrect pixel has double weighting
    return matchPercent;
}
