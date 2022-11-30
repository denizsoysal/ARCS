/*

#include<opencv2/opencv.hpp>//OpenCV header to use VideoCapture class//
#include<iostream>

using namespace std;
using namespace cv;
int main() {
   Mat myImage;//Declaring a matrix to load the frames//
   namedWindow("Video Player");//Declaring the video to show the video//
   VideoCapture cap(0);//Declaring an object to capture stream of frames from default camera//


   if (!cap.isOpened()){ //This section prompt an error message if no video stream is found//
      cout << "No video stream detected" << endl;
      system("pause");
      return-1;
   }


   while (true){ //Taking an everlasting loop to show the video//
      cap >> myImage;

      if (myImage.empty()){ //Breaking the loop if no video frame is detected//
         break;
      }
      
      imshow("Video Player", myImage);//Showing the video//
      char c = (char)waitKey(25);//Allowing 25 milliseconds frame processing time and initiating break condition//
      if (c == 27){ //If 'Esc' is entered break the loop//
         break;
      }
   }


   cap.release();//Releasing the buffer memory//
   return 0;
}

*/



/////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;
int main(int, char**)
{
    Mat frame;
    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //--- GRAB AND WRITE LOOP
    cout << "Start grabbing" << endl
        << "Press any key to terminate" << endl;
    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
        imshow("Live", frame);
        if (waitKey(5) >= 0)
            break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}