#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>

/*
  Do not use any smoothing algorithms when detecting edges
*/

//Takes avg pixel color value (3x3 grid)
CvScalar avgpixelcolor(IplImage*image,CvPoint seed,int gridSize){
    CvScalar pixColor;
    cvSetImageROI(image, cvRect(seed.x,seed.y,gridSize,gridSize));  //Restrict Color Avg to ROI

    IplImage*b=cvCreateImage(cvGetSize(image),IPL_DEPTH_8U,1); //Channel Holders
    IplImage*g=cvCreateImage(cvGetSize(image),IPL_DEPTH_8U,1);
    IplImage*r=cvCreateImage(cvGetSize(image),IPL_DEPTH_8U,1);

    cvSplit(image,b,g,r,NULL);  //Split Color Spaces and Average
    pixColor.val[0]=cvMean(b);
    pixColor.val[1]=cvMean(g);
    pixColor.val[2]=cvMean(r);

    cvResetImageROI(image);
    cvReleaseImage(&b);
    cvReleaseImage(&g);
    cvReleaseImage(&r);
    return pixColor;
}

int main( int argc, char** argv ) {

    if (argc>=2){
        //Load image
        IplImage*src = cvLoadImage( argv[1]);

        //Create Windows and Position
        cvNamedWindow("Input",0);
        cvResizeWindow("Input",500,350);
        cvMoveWindow("Input", 0, 0);

        cvNamedWindow("Output",0);
        cvResizeWindow("Output",500,350);
        cvMoveWindow("Output", 0, 600);

        cvNamedWindow( "Hough", 0 );
        cvResizeWindow("Hough",500,350);
        cvMoveWindow("Hough",700,0);

        cvNamedWindow( "FloodFill", 0 );
        cvResizeWindow("FloodFill",500,350);
        cvMoveWindow("FloodFill",700,600);

        //Display Original Image
        cvShowImage( "Input", src );
        IplImage*srcCopy = cvCloneImage(src);

        //Flood Fill
        CvPoint seedPoint= cvPoint((srcCopy->width)/2,(srcCopy->height)/2);
        CvScalar pixColor=avgpixelcolor(srcCopy,seedPoint,5); //Takes avg pixel color value (5x5 grid)
        cvLine( srcCopy, cvPoint(seedPoint.x,srcCopy->height*.9), cvPoint(seedPoint.x,srcCopy->height*.1), pixColor, 3, 8 );
        cvFloodFill(srcCopy,seedPoint,cvScalarAll(255),cvScalarAll(50),cvScalarAll(50),NULL,8|CV_FLOODFILL_FIXED_RANGE, NULL);
        cvShowImage("FloodFill",srcCopy);

        //Convert to Grayscale
        IplImage*dst = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1 );
        cvCvtColor( srcCopy, dst , CV_BGR2GRAY );

        //Display Flood Fill Results
        cvCircle(srcCopy,seedPoint,5,cvScalarAll(0),3,8,0);

        //Threshold
        IplImage*thresh = cvCreateImage(cvGetSize(dst), IPL_DEPTH_8U,1);
        cvThreshold(dst,thresh,
                    230,        //Set Manually during Initialization
                    255,        //Max Pixel Intensity Value (Do not Change)
                    CV_THRESH_TOZERO
                    );

        //Canny Edge Detection
        cvCanny( thresh, dst,
                 0,    //Low Threshold
                 255,   //High Threshold
                 3
                 );

        //Storage for Hough Line Endpoints
        CvMemStorage* storage = cvCreateMemStorage(0);

        //Hough
        CvSeq* lines = cvHoughLines2( dst,storage,CV_HOUGH_PROBABILISTIC,
                                      1,               //rho
                                      1*CV_PI/180,     //theta
                                      150,             //Accumulator threshold
                                      500,             //Min Line Length
                                      200              //Min Colinear Separation
                                      );

        //Draw Vertical Lines on src image
        for(int i = 0; i < lines->total; i++ )
        {
            CvPoint* Point = (CvPoint*)cvGetSeqElem(lines,i);
            cvLine( src, Point[0], Point[1], CV_RGB(0,220,20), 3, 8 );

            //Reject Horizontal lines
            float slope=(Point[0].y-Point[1].y)/(Point[0].x-Point[1].x);

        }

        //Create a Trapazodal Mask

        //Detect Horizontal Lines

        //Isolate 4 points

        //Display Image
        cvShowImage( "Output", src);

        //For Calibration Purposes "what the Hough transform sees"
        cvShowImage( "Hough", dst );

        //Wait for User 10sec
        cvWaitKey(10000);

        //Deallocate Memory
        cvReleaseImage( &src );
        cvReleaseImage( &dst );
        cvReleaseImage( &thresh );
        cvDestroyWindow( "Input" );
        cvDestroyWindow( "Output" );
        cvDestroyWindow("Hough");

    }
    else{
        printf("Hough Transform Code Requires \n");
        return 0;
    }
}
