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

using namespace cv;

int hueLow  = 0;
int hueHigh = 179;
int satLow  = 0;
int satHigh = 50;
int valLow  = 90;
int valHigh = 255;


// Instantiate a Mat in which to store each video frame.
Mat origFrame;
Mat resizedFrame;   // Scaled-down from origFrame by factor of 2.
Mat hsvFrame;   // Converted to HSV space from resizedFrame.
Mat bwFrame;   // Black/white image after thresholding hsvFrame.

ros::Publisher blobPub;


int main(int argc, char** argv) {
    ros::init(argc, argv, "blobber");
    ros::NodeHandle nh;

    // Instantiate VideoCapture object. See here for details:
    // http://opencv.willowgarage.com/documentation/cpp/reading_and_writing_images_and_video.html
    VideoCapture cap(0);

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

    cvCreateTrackbar("hueLow",  "control panel", &hueLow,  179, NULL);
    cvCreateTrackbar("hueHigh", "control panel", &hueHigh, 179, NULL);
    cvCreateTrackbar("satLow",  "control panel", &satLow,  255, NULL);
    cvCreateTrackbar("satHigh", "control panel", &satHigh, 255, NULL);
    cvCreateTrackbar("valLow",  "control panel", &valLow,  255, NULL);
    cvCreateTrackbar("valHigh", "control panel", &valHigh, 255, NULL);

    while (true) {
        // Capture image.
        cap >> origFrame;

        // Resize the image to increase processing rate. See here for details:
        // http://opencv.willowgarage.com/documentation/cpp/image_filtering.html
        pyrDown(origFrame, resizedFrame, Size(origFrame.cols/2, origFrame.rows/2));

        // Convert the frame to HSV and save to hsvFrame.
        cvtColor(resizedFrame, hsvFrame, CV_BGR2HSV);

        // Threshold hsvFrame for color of maze walls and save to bwFrame.
        inRange(hsvFrame, Scalar(hueLow,  satLow,  valLow),
                          Scalar(hueHigh, satHigh, valHigh), bwFrame);

        // Show the image, with the squares overlaid.
        imshow("origImage", resizedFrame);
        imshow("bwImage", bwFrame);

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
