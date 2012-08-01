// The "Square Detector" program.
// It loads several images sequentially and tries to find squares in
// each image

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include <ros/ros.h>
#include <osuar_vision/windowCoordinates.h>


#define FIND_WINDOW         0
#define FIND_SECURITY_LIGHT 1
#define FIND_FLASH_DRIVE    2


using namespace cv;


// SQUARES

// Thresholds for Canny edge detector.
int cannyThres1 = 100;
int cannyThres2 = 500;

// Hierarchy levels for findContours()
int hierarchyLevels = 0;

// Threshold for maximum cosine between angles (x100).
int maxCosineThres = 20;

// Threshold for ratio of shortest side / longest side (x100).
int sideRatioThres = 75;

// Maximum square area.
int maxSquareArea = 41000;

// Find colors of any hue...
int hueLow  = 100;
int hueHigh = 160;

// ...of low saturation...
int satLow  = 10;
int satHigh = 80;

// ...ranging down to gray, but not completely dark. That is to say, white.
int valLow  = 60;
int valHigh = 255;

// Blur size
int blurSize = 10;

// Hough transform thresholds
int accThres   = 60;
int minLineLen = 5;
int maxLineGap = 70;

// Instantiate a Mat in which to store each video frame.
Mat origFrame;
Mat resizedFrame;   // Scaled-down from origFrame by factor of 2.
Mat hsvFrame;   // Converted to HSV space from resizedFrame.
Mat bwFrame;   // Black/white image after thresholding hsvFrame.
Mat dilatedFrame;
Mat grayFrame;
Mat cannyFrame;

vector<vector<Point> > squares;
vector<vector<Point> > contours;
vector<Vec4i> houghLines;
vector<Vec4i> windowLines;
vector<Vec4i> hierarchy;

ros::Publisher visPub;
osuar_vision::windowCoordinates winCoords;

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static void findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();
    contours.clear();

    Mat pyr, timg;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());

    // find contours and store them all as a list
    findContours(cannyFrame, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    vector<Point> approx;

    // test each contour
    for( size_t i = 0; i < contours.size(); i++ )
    {
        // approximate contour with accuracy proportional
        // to the contour perimeter
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if( approx.size() == 4 &&
            fabs(contourArea(Mat(approx))) > 1000 &&
            fabs(contourArea(Mat(approx))) < maxSquareArea &&
            isContourConvex(Mat(approx)) )
        {
            double maxCosine = 0;
            double minSideLen = 100000;
            double maxSideLen = 0;
            double sideRatio = 0;

            for( int j = 2; j < 5; j++ )
            {
                // find the maximum cosine of the angle between joint edges
                double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                maxCosine = MAX(maxCosine, cosine);

                // Find the maximum difference in length of adjacent
                // sides
                double sideLen = sqrt(pow((approx[j%4].x - approx[(j+1)%4].x), 2) + pow((approx[j%4].y - approx[(j+1)%4].y), 2));
                minSideLen = MIN(minSideLen, sideLen);
                maxSideLen = MAX(maxSideLen, sideLen);
            }

            sideRatio = minSideLen / maxSideLen;

            // if cosines of all angles are small
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
            if( maxCosine < ((double) maxCosineThres)/100 && sideRatio >= (double) sideRatioThres/100 )
                squares.push_back(approx);
        }
    }
}


// the function draws all the squares in the image
static void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, CV_AA);

        //std::cout << "x: " << squares[i][0].x << "  y: " << squares[i][0].y << "\n";

        // Only publish coordinates for one of the squares. Coordinates are
        // shifted over by half the camera resolution (which itself is scaled
        // down by a factor of two!) on each axis. The y coordinate is inverted
        // so up is positive.
        if (i == 0) {
            putText(image, "0", squares[0][0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0));
            putText(image, "1", squares[0][1], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0));
            putText(image, "2", squares[0][2], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0));
            putText(image, "3", squares[0][3], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0));
            winCoords.x =   (squares[0][0].x + squares[0][2].x)/2 - 180;
            winCoords.y = -((squares[0][0].y + squares[0][2].y)/2 - 120);
            winCoords.size = fabs(squares[0][0].x - squares[0][2].x);
            visPub.publish(winCoords);
        }
    }
}


static void findBlobs (const Mat& image) {
    // Run Canny on bwFrame and save to cannyFrame.
    Canny(bwFrame, cannyFrame, cannyThres1, cannyThres2, 5);
    dilate(cannyFrame, cannyFrame, Mat(), Point(-1,-1), 3);

    // Find contours.
    findContours(cannyFrame, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    if (!contours.empty() && !hierarchy.empty()) {                                   
    int i, b, g, r;
        for (i=0; i>=0; i=hierarchy[i][0]) {
            b = rand()&255;
            g = rand()&255;
            r = rand()&255;
            drawContours(resizedFrame, contours, i, Scalar(b,g,r), CV_FILLED, 8, hierarchy);
        }
    }
}




int main(int argc, char** argv) {
    ros::init(argc, argv, "vision");
    ros::NodeHandle nh;
    visPub = nh.advertise<osuar_vision::windowCoordinates>("window_coordinates", 1);

    // Instantiate VideoCapture object. See here for details:
    // http://opencv.willowgarage.com/documentation/cpp/reading_and_writing_images_and_video.html
    VideoCapture cap(1);

    // Configure video. Our camera NEEDS the frame width to be set to 720
    // pixels.
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 720);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    //cap.set(CV_CAP_PROP_FPS, 20);

    cvNamedWindow("control panel", 1);
    cvMoveWindow("control panel", 450, 20);

    cvNamedWindow("origImage", 1);
    cvMoveWindow("origImage", 20, 20);

    cvNamedWindow("bwImage", 1);
    cvMoveWindow("bwImage", 20, 270);

    cvNamedWindow("cannyImage", 1);
    cvMoveWindow("cannyImage", 20, 520);

    cvCreateTrackbar("cannyThres1",     "control panel", &cannyThres1, 2000, NULL);
    cvCreateTrackbar("cannyThres2",     "control panel", &cannyThres2, 2000, NULL);
    cvCreateTrackbar("hierarchyLevels", "control panel", &hierarchyLevels, 6, NULL);
    cvCreateTrackbar("maxCosineThres (x100)", "control panel", &maxCosineThres, 100, NULL);
    cvCreateTrackbar("sideRatioThres (x100)", "control panel", &sideRatioThres, 100, NULL);
    cvCreateTrackbar("maxSquareArea", "control panel", &maxSquareArea, 100000, NULL);
    cvCreateTrackbar("hueLow",     "control panel", &hueLow,  179, NULL);
    cvCreateTrackbar("hueHigh",    "control panel", &hueHigh, 179, NULL);
    cvCreateTrackbar("satLow",     "control panel", &satLow,  255, NULL);
    cvCreateTrackbar("satHigh",    "control panel", &satHigh, 255, NULL);
    cvCreateTrackbar("valLow",     "control panel", &valLow,  255, NULL);
    cvCreateTrackbar("valHigh",    "control panel", &valHigh, 255, NULL);
    cvCreateTrackbar("accThres",   "control panel", &accThres,   100, NULL);
    cvCreateTrackbar("minLineLen", "control panel", &minLineLen, 150, NULL);
    cvCreateTrackbar("maxLineGap", "control panel", &maxLineGap, 150, NULL);

    while (true) {
        // Capture image.
        cap >> origFrame;

        // Resize the image to increase processing rate. See here for details:
        // http://opencv.willowgarage.com/documentation/cpp/image_filtering.html
        pyrDown(origFrame, resizedFrame, Size(origFrame.cols/2, origFrame.rows/2));

        // Convert the frame to HSV and save to hsvFrame.
        cvtColor(resizedFrame, hsvFrame, CV_BGR2HSV);

        // Threshold hsvFrame for color of maze s and save to bwFrame.
        inRange(hsvFrame, Scalar(hueLow,  satLow,  valLow),
                          Scalar(hueHigh, satHigh, valHigh), bwFrame);

        // Convert resizedFrame to grayscale and save to grayFrame.
        cvtColor(resizedFrame, grayFrame, CV_BGR2GRAY);

        // Run Canny on grayFrame and save to cannyFrame.
        Canny(bwFrame, cannyFrame, cannyThres1, cannyThres2, 5);
        dilate(cannyFrame, cannyFrame, Mat(), Point(-1,-1));

        // Run probabilistic Hough Transform on cannyFrame and save results to
        // houghLines.
        HoughLinesP(cannyFrame, houghLines,
                1,                  // rho
                CV_PI/180,          // theta
                MAX(accThres, 1),   // Accumulator threshold (must be > 0)
                minLineLen,         // Minimum line length
                maxLineGap          // Maximum line gap
                );

        // Save (relatively) horizontal and vertical lines to windowLines.
        for (size_t i=0; i<houghLines.size(); i++) {
            if (fabs((houghLines[i][1]-houghLines[i][3]) / (fabs(houghLines[i][0]-houghLines[i][2]))+1) > 10 ||
                    fabs((houghLines[i][0]-houghLines[i][2]) / (fabs(houghLines[i][1]-houghLines[i][3]))+1) > 10) {
                windowLines.push_back(houghLines[i]);
            }
        }

        // Draw windowLines on the original image.
        for (size_t i=0; i<windowLines.size(); i++) {
            line(resizedFrame, Point(windowLines[i][0], windowLines[i][1]), Point(windowLines[i][2], windowLines[i][3]), Scalar(0,0,255), 1, 8 );
            //line(bwFrame, Point(windowLines[i][0], windowLines[i][1]), Point(windowLines[i][2], windowLines[i][3]), Scalar(255,255,255), 1, 8 );
        }

        // Clear vectors.
        houghLines.clear();
        windowLines.clear();

        // Find squares in cannyFrame and draw them on resizedFrame.
        findSquares(cannyFrame, squares);
        drawSquares(resizedFrame, squares);

        // Show the image, with the squares overlaid.
        imshow("origImage", resizedFrame);
        imshow("bwImage", bwFrame);
        imshow("cannyImage", cannyFrame);

        // Wait 5 milliseconds for a keypress.
        int c = waitKey(5);
        // Exit if the spacebar is pressed. NOTE: killing the program with
        // Ctrl+C sometimes stops OpenCV at a bad place and effects a kernel
        // panic! If you really like Ctrl+C, do so at your own risk!
        if ((char) c == 32) {
            return 0;
        }
    }

    return 0;
}
