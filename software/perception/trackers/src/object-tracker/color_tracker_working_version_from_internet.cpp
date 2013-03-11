// TrackColour.cpp : Defines the entry point for the console application.
//
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;

IplImage* GetThresholdedImage(IplImage* img){
  // Convert the image into an HSV image
  IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
  cvCvtColor(img, imgHSV, CV_BGR2HSV);

  IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);

  // Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
  //cvInRangeS(imgHSV, cvScalar(112, 100, 100), cvScalar(124, 255, 255), imgThreshed);

  // Yellow:
  //cvInRangeS(imgHSV, cvScalar(0, 100, 100), cvScalar(30, 255, 255), imgThreshed);
  // Orange (tropicana:
  //cvInRangeS(imgHSV, cvScalar(10, 50, 50), cvScalar(15, 255, 255), imgThreshed);
  // red bowl:
  //cvInRangeS(imgHSV, cvScalar(0, 50, 50), cvScalar(8, 255, 255), imgThreshed);
  // green (top of lemon juice):
  cvInRangeS(imgHSV, cvScalar(55, 50, 50), cvScalar(65, 255, 255), imgThreshed);

  cvReleaseImage(&imgHSV);

  return imgThreshed;
}

int main()
{
  // Initialize capturing live feed from the camera
  CvCapture* capture = 0;
  capture = cvCaptureFromCAM(0);        

  // Couldn't get a device? Throw an error and quit
  if(!capture){
    printf("Could not initialize capturing...\n");
    return -1;
  }

  // The two windows we'll be using
  cvNamedWindow("video");
  cvNamedWindow("thresh");

  // This image holds the "scribble" data...
  // the tracked positions of the ball
  IplImage* imgScribble = NULL;

  // An infinite loop
  while(true){
    // Will hold a frame captured from the camera
    IplImage* frame = 0;
    frame = cvQueryFrame(capture);

    cvFlip(frame, NULL, 1);

    // If we couldn't grab a frame... quit
    if(!frame)
      break;

    // If this is the first frame, we need to initialize it
    if(imgScribble == NULL){
      imgScribble = cvCreateImage(cvGetSize(frame), 8, 3);
    }

    // Holds the yellow thresholded image (yellow = white, rest = black)
    IplImage* imgColorThresh = GetThresholdedImage(frame);

    // Calculate the moments to estimate the position of the ball
    CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
    cvMoments(imgColorThresh, moments, 1);

    // The actual moment values
    double moment10 = cvGetSpatialMoment(moments, 1, 0);
    double moment01 = cvGetSpatialMoment(moments, 0, 1);
    double area = cvGetCentralMoment(moments, 0, 0);

    // Holding the last and current ball positions
    static int posX = 0;
    static int posY = 0;

    int lastX = posX;
    int lastY = posY;

    posX = moment10/area;
    posY = moment01/area;

    // Print it out for debugging purposes
    cout << area << "\n";
    printf("position (%d,%d)\n", posX, posY);

    // We want to draw a line only if its a valid position
    if (area> 30){
      if(lastX>0 && lastY>0 && posX>0 && posY>0){
        // Draw a yellow line from the previous point to the current point
        cvLine(imgScribble, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,255,255), 5);
      }
    }

    // Add the scribbling image and the frame... and we get a combination of the two
    cvAdd(frame, imgScribble, frame);
    cvShowImage("thresh", imgColorThresh);
    cvShowImage("video", frame);

    // Wait for a keypress
    int c = cvWaitKey(10);
    if(c!=-1){
      // If pressed, break out of the loop
      break;
    }

    // Release the thresholded image... we need no memory leaks.. please
    cvReleaseImage(&imgColorThresh);

    delete moments;
  }

  // We're done using the camera. Other applications can now use it
  cvReleaseCapture(&capture);
  return 0;
}
