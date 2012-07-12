#ifndef BLOB_DETECTION_H
#define BLOB_DETECTION_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvblobs/BlobResult.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
using namespace cv;
using namespace std;

#define CVX_RED    CV_RGB(0xff,0x00,0x00)
#define CVX_GREEN    CV_RGB(0x00,0xff,0x00)
#define CVX_BLUE    CV_RGB(0x00,0x00,0xff)
#define ORIGIONALIMAGE 0
#define HUEIMAGE 1
#define SATIMAGE 2
#define BRIGHTIMAGE 3
#define ANDIMAGE 4
#define BLOBIMAGE 5
#define BLOBROI 6
#define RED 1
#define BLUE 0

void renameWindow(int select, char *& previousWindowName){
	switch(select){
		case 0:
			if(!strcmp("blobDetection - Origional Image", previousWindowName)) return;
			cvNamedWindow("blobDetection - Origional Image");
			cvDestroyWindow(previousWindowName);
			//free(previousWindowName);
			previousWindowName = (char *)"blobDetection - Origional Image";
			return;
		case 1:
			if(!strcmp("blobDetection - Hue", previousWindowName)) return;
			cvNamedWindow("blobDetection - Hue");
			cvDestroyWindow(previousWindowName);
			previousWindowName = (char *)"blobDetection - Hue";
			break;
		case 2:
			if(!strcmp("blobDetection - Saturation", previousWindowName)) return;
			cvNamedWindow("blobDetection - Saturation");
			cvDestroyWindow(previousWindowName);
			previousWindowName = (char *)"blobDetection - Saturation";
			break;
		case 3:
			if(!strcmp("blobDetection - Brightness", previousWindowName)) return;
			cvNamedWindow("blobDetection - Brightness");
			cvDestroyWindow(previousWindowName);
			previousWindowName = (char *)"blobDetection - Brightness";
			break;
		case 4:
			if(!strcmp("blobDetection - cvAnd", previousWindowName)) return;
			cvNamedWindow("blobDetection - cvAnd");
			cvDestroyWindow(previousWindowName);
			previousWindowName = (char *)"blobDetection - cvAnd";
			break;
		case 5:
			if(!strcmp("blobDetection - Blobs", previousWindowName)) return;
			cvNamedWindow("blobDetection - Blobs");
			cvDestroyWindow(previousWindowName);
			previousWindowName = (char *)"blobDetection - Blobs";
			break;
		case 6:
			if(!strcmp("blobDetection - ROIs Overlayed", previousWindowName)) return;
			cvNamedWindow("blobDetection - ROIs Overlayed");
			cvDestroyWindow(previousWindowName);
			previousWindowName = (char *)"blobDetection - ROIs Overlayed";
			break;


	}


}

//Display An image on the screen
void showImage(cv::Mat image){
	//read an image
	//cv::Mat image = cv::imread(myImage);
	if(image.size().height >1000){
		cv::Mat simage = cv::Mat(image.rows / 4, image.cols / 3, CV_8UC3);
		cv::resize(image, simage,simage.size());
		cv::namedWindow("My Image");
		cv::imshow("My Image", simage);
		cv::waitKey();
	}else{
		cv::namedWindow("My Image");
		cv::imshow("My Image", image);
		cv::waitKey();
	}
}

void showImage(cv::Mat &image, char * windowName){
	//read an image
	//cv::Mat image = cv::imread(myImage);
	if(image.size().height >1000){
		cv::Mat simage = cv::Mat(image.rows / 4, image.cols / 4, CV_8UC3);
		cv::resize(image, simage,simage.size());
		cv::namedWindow(windowName);
		cv::imshow(windowName, simage);
		cv::waitKey();
	}else{
		cv::namedWindow(windowName);
		cv::imshow(windowName, image);
		cv::waitKey();
	}
}



cv::Mat iplToMat(IplImage* img){
	cv::Mat result(img);
	return result;
}


void callbackButton2(int state, void* userdata){}

void showImage(cv::Mat image, int timeDisplayed, char* windowName){
	if(image.size().height >1000){
		cv::Mat simage = cv::Mat(image.rows / 4, image.cols / 3, CV_8UC3);
		cv::resize(image, simage,simage.size());
		cv::namedWindow(windowName);
		cv::imshow(windowName, simage);
		cv::waitKey(timeDisplayed);
	}else if(image.size().height >600){
		cv::Mat simage = cv::Mat(image.rows / 2, image.cols / 2, CV_8UC3);
		cv::resize(image, simage,simage.size());
		cv::namedWindow(windowName);
		cv::imshow(windowName, simage);
		cv::waitKey(timeDisplayed);
	}else{
		cv::namedWindow(windowName);
		cv::imshow(windowName, image);
		cv::waitKey(timeDisplayed);
	}
}

IplImage * blobDetection(const IplImage *image, double hueThresh, double satThresh, double brightThresh,CvRect *&rect, int minArea, int maxArea, int select){

	IplImage *imageBGR = cvCreateImage( cvGetSize(image), 8, 3);
	cvCopy(image, imageBGR);
	if(select == 0) return imageBGR;
	if (!imageBGR) {
		cout << "In blobDetection: ERROR: Image is NULL " << endl;
		cvWaitKey(2000);
		return NULL;	// Quit
	}

	// Create some GUI windows for output display.

	// Convert the image to HSV colors.
	IplImage* imageHSV = cvCreateImage( cvGetSize(imageBGR), 8, 3);	// Full HSV color image.
	cvCvtColor(imageBGR, imageHSV, CV_BGR2HSV);				// Convert from a BGR to an HSV image.

	// Get the separate HSV color components of the color input image.
	IplImage* planeH = cvCreateImage( cvGetSize(imageBGR), 8, 1);	// Hue component.
	IplImage* planeS = cvCreateImage( cvGetSize(imageBGR), 8, 1);	// Saturation component.
	IplImage* planeV = cvCreateImage( cvGetSize(imageBGR), 8, 1);	// Brightness component.
	cvCvtPixToPlane(imageBGR,planeH,planeS,planeV, 0);
	cvMerge(planeH,0,planeV,0,imageHSV);
	cvCvtPixToPlane(imageHSV, planeH, planeS, planeV, 0);	// Extract the 3 color components.


	// Detect which pixels in each of the H, S and V channels are probably skin pixels.
	// Assume that skin has a Hue between 0 to 18 (out of 180), and Saturation above 50, and Brightness above 80.
	if(hueThresh) cvThreshold(planeH, planeH, hueThresh, UCHAR_MAX, CV_THRESH_BINARY_INV);
	if(select == HUEIMAGE){
		cvReleaseImage( &imageBGR );
		cvReleaseImage( &imageHSV );
		cvReleaseImage( &planeS);
		cvReleaseImage( &planeV);
		return planeH;
	}


	if(satThresh) cvThreshold(planeS, planeS, satThresh, UCHAR_MAX, CV_THRESH_BINARY);
	if(select == SATIMAGE){
		cvReleaseImage( &imageBGR );
		cvReleaseImage( &imageHSV );
		cvReleaseImage( &planeH );
		cvReleaseImage( &planeV);
		return planeS;
	}

	if(brightThresh) cvThreshold(planeV, planeV, brightThresh, UCHAR_MAX, CV_THRESH_BINARY);
	if(select == BRIGHTIMAGE){
		cvReleaseImage( &imageBGR );
		cvReleaseImage( &imageHSV );
		cvReleaseImage( &planeH );
		cvReleaseImage( &planeS );
		return planeV;
	}


	// Combine all 3 thresholded color components, so that an output pixel will only
	// be white if the H, S and V pixels were also white.
	IplImage* blobPixels = cvCreateImage( cvGetSize(imageBGR), 8, 1);	// Greyscale output image.
	cvAnd(planeH, planeS, blobPixels);				// imageSkin = H {BITWISE_AND} S.
	cvAnd(blobPixels, planeV, blobPixels);	// imageSkin = H {BITWISE_AND} S {BITWISE_AND} V.
	if(select == ANDIMAGE){
		cvReleaseImage( &imageBGR );
		cvReleaseImage( &imageHSV );
		cvReleaseImage( &planeH );
		cvReleaseImage( &planeS );
		cvReleaseImage( &planeV );
		return blobPixels;
	}


	// Find blobs in the image.
	CBlobResult blobs;
	blobs = CBlobResult(blobPixels, NULL, 0);	// Use a black background color.

	// Ignore the blobs whose area is less than minArea.
	blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, minArea);
	blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, maxArea);
	// Show the blobs within the right Area.
	IplImage* imageBlobs = cvCreateImage( cvGetSize(imageBGR), 8, 1);	// Greyscale output image.
	IplImage * blobROI = cvCloneImage(imageBGR);
        rect = new CvRect[blobs.GetNumBlobs()];
	for (int i = 0; i < blobs.GetNumBlobs(); i++) {
		CBlob *currentBlob = blobs.GetBlob(i);
		currentBlob->FillBlob(imageBlobs, CV_RGB(255,255,255));	// Draw the large blobs as white.
		rect[i] = currentBlob->GetBoundingBox(); 
		cvRectangle(blobROI,cvPoint( rect[i].x, rect[i].y ), cvPoint(rect[i].x+rect[i].width, rect[i].y + rect[i].height),cvScalar(0,255,0),3);
		//showImage(iplToMat(blobROI),1000,"sec");
	}
	// Wait until the user hits a key on the GUI window.
	//cvWaitKey(0);	// Note that if you don't use cvWaitKey(), OpenCV will never display anything!
	if(select == BLOBIMAGE){
		// Free all the resources.
		cvReleaseImage( &imageBGR );
		cvReleaseImage( &imageHSV );
		cvReleaseImage( &planeH );
		cvReleaseImage( &planeS );
		cvReleaseImage( &planeV );
		cvReleaseImage( &blobPixels );
		cvReleaseImage( &blobROI);
		return imageBlobs;
	}

	cvReleaseImage( &imageBGR );
	cvReleaseImage( &imageHSV );
	cvReleaseImage( &planeH );
	cvReleaseImage( &planeS );
	cvReleaseImage( &planeV );
	cvReleaseImage( &blobPixels );
	cvReleaseImage( &imageBlobs);
	return blobROI;

	//cvReleaseImage( &imageSkinBlobs );

}

#endif // BLOB_DETECTION_H

