// AppliedRoboticsCV.cpp : Defines the entry point for the console application.
#include <osuar_vision/blob_detection.h>
#define ORIGIONALIMAGE 0
#define HUEIMAGE 1
#define SATIMAGE 2
#define BRIGHTIMAGE 3
#define ANDIMAGE 4
#define BLOBIMAGE 5
#define BLOBROI 6
#define RED 1
#define BLUE 0


int main(int, char**){
	CvCapture *capture = 0;
	capture = cvCaptureFromCAM( 0 );
	IplImage  *frame = 0;
	IplImage *blobFrameOne = 0;
	IplImage *blobFrameTwo = 0;
	char * windowName = (char *) "blobDetection";
	int hueValue = 0, saturationValue = 0, brightnessValue = 0, select = 0, minSize = 0, maxSize = 0;
	CvRect * rect = 0;
	cvNamedWindow(windowName);
	cvNamedWindow("Control Panel");
	cvCreateTrackbar( "     Hue     ", "Control Panel" , &hueValue       , 255  , NULL);
	cvCreateTrackbar( "Saturation"   , "Control Panel" , &saturationValue, 255  , NULL);
	cvCreateTrackbar( "Brightness"   , "Control Panel" , &brightnessValue, 255  , NULL);
	cvCreateTrackbar( "Min Size  "   , "Control Panel" , &minSize        , 100000, NULL);
	cvCreateTrackbar( "Max Size  "   , "Control Panel" , &maxSize        , 100000, NULL);
	cvCreateTrackbar( "Image Sel ", "Control Panel" , &select    , 6    , NULL);
	int i=0;
	for(i=0; i<4; i++) //first few frames from webcam are garbage
		frame = cvQueryFrame( capture );
	while(true){
		blobFrameOne = blobDetection(frame, hueValue, saturationValue, brightnessValue, rect, minSize, maxSize, select);
		renameWindow(select, windowName);
		cvShowImage(windowName, blobFrameOne);
		if(blobFrameTwo != 0) cvReleaseImage(&blobFrameTwo);
		if(rect != 0) delete[] rect;
		rect = 0;
		cvWaitKey(1);
		frame = cvQueryFrame( capture );
		blobFrameTwo = blobDetection(frame, hueValue, saturationValue, brightnessValue, rect, minSize, maxSize, select);
		renameWindow(select, windowName);
		cvShowImage(windowName, blobFrameTwo);
		cvReleaseImage(&blobFrameOne);
		cvWaitKey(1);
        }
	cvReleaseImage(&frame);
	cvReleaseCapture(&capture);
	return 0;
}

