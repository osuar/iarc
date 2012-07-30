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

using namespace cv;

int thresh1 = 100;
int thresh2 = 500, N = 11;

// Threshold for maximum cosine between angles (x100).
int maxCosineThresh = 20;

// Threshold for ratio of shortest side / longest side (x100).
int sideRatioThresh = 75;

// Maximum square area.
int maxSquareArea = 41000;

// Find colors of any hue...
int wallHueLow  = 0;
int wallHueHigh = 179;

// ...of low saturation...
int wallSatLow  = 0;
int wallSatHigh = 50;

// ...ranging down to gray, but not completely dark. That is to say, white.
int wallValLow  = 90;
int wallValHigh = 255;

// Hough transform thresholds
int minLineLen = 5;
int maxLineGap = 10;

// Hierarchy levels for findContours()
int hierarchyLevels = 0;

// Instantiate a Mat in which to store each video frame.
Mat origFrame;
Mat resizedFrame;   // Scaled-down from origFrame by factor of 2.
Mat hsvFrame;   // Converted to HSV space from resizedFrame.
Mat bwFrame;   // Black/white image after thresholding hsvFrame.
Mat grayFrame;
Mat cannyFrame;

vector<vector<Point> > squares;
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

    Mat pyr, timg;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    //for( int c = 0; c < 3; c++ )
    //{
        //int ch[] = {c, 0};
        //mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        //for( int l = 0; l < N; l++ )
        //{
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            //if( l == 0 )
            //{
            //    // apply Canny. Take the upper threshold from slider
            //    // and set the lower to 0 (which forces edges merging)
            //    Canny(image, cannyFrame, 0, thresh2, 5);
            //    // dilate canny output to remove potential
            //    // holes between edge segments
            //    dilate(cannyFrame, cannyFrame, Mat(), Point(-1,-1));
            //}
            //else
            //{
            //    // apply threshold if l!=0:
            //    //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
            //    cannyFrame = image >= (l+1)*255/N;
            //}

            // find contours and store them all as a list
            findContours(cannyFrame, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            // Draw the contours.
            drawContours(resizedFrame, contours, -1, Scalar(255,255,0), 1, CV_AA);

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

                    //std::cout << minSideLen << "  " << maxSideLen << "\n";

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < ((double) maxCosineThresh)/100 && sideRatio >= (double) sideRatioThresh/100 )
                        squares.push_back(approx);
                }
            }
        //}
    //}
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

    cvCreateTrackbar("thresh1",   "control panel", &thresh1,      2000, NULL);
    cvCreateTrackbar("thresh2",   "control panel", &thresh2,      2000, NULL);
    cvCreateTrackbar("hierarchyLevels", "control panel", &hierarchyLevels, 6, NULL);
    cvCreateTrackbar("maxCosineThresh (x100)", "control panel", &maxCosineThresh, 100, NULL);
    cvCreateTrackbar("sideRatioThresh (x100)", "control panel", &sideRatioThresh, 100, NULL);
    cvCreateTrackbar("maxSquareArea", "control panel", &maxSquareArea, 100000, NULL);
    cvCreateTrackbar("wallHueLow",  "control panel", &wallHueLow,  179, NULL);
    cvCreateTrackbar("wallHueHigh", "control panel", &wallHueHigh, 179, NULL);
    cvCreateTrackbar("wallSatLow",  "control panel", &wallSatLow,  255, NULL);
    cvCreateTrackbar("wallSatHigh", "control panel", &wallSatHigh, 255, NULL);
    cvCreateTrackbar("wallValLow",  "control panel", &wallValLow,  255, NULL);
    cvCreateTrackbar("wallValHigh", "control panel", &wallValHigh, 255, NULL);
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

        // Threshold hsvFrame for color of maze walls and save to bwFrame.
        inRange(hsvFrame, Scalar(wallHueLow,  wallSatLow,  wallValLow),
                          Scalar(wallHueHigh, wallSatHigh, wallValHigh), bwFrame);

        // Convert resizedFrame to grayscale and save to grayFrame.
        cvtColor(resizedFrame, grayFrame, CV_BGR2GRAY);

        // Run Canny on grayFrame and save to cannyFrame.
        Canny(bwFrame, cannyFrame, thresh1, thresh2, 5);
        dilate(cannyFrame, cannyFrame, Mat(), Point(-1,-1));

        // Run probabilistic Hough Transform on cannyFrame and save results to
        // houghLines.
        HoughLinesP(cannyFrame, houghLines,
                1,              // rho
                CV_PI/180,      // theta
                80,             // Accumulator threshold
                minLineLen,     // Minimum line length
                maxLineGap      // Maximum line gap
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
