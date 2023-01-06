//using the otsu detector like in https://learnopencv.com/otsu-thresholding-with-opencv/, we detect the whiteboard

/*we make the assumptions that 
- the whiteboard is on top of the table in the workspace of the robot 
- the whiteboard has edges in very distinc colour than the rest of the image (the whiteboard used has metallic ones)
- Thus the otsu detector finds 2 main color group: white and metallic ones 
*/

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/core/types.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

const char* window_name = "Region Growing";


int main( int argc, char** argv )

{

Mat img, img_gray,img_thresh,img_thresh_dilated,img_thresh_dilated_eroded;
img = imread("../BB.png",IMREAD_COLOR);
cvtColor( img, img_gray, COLOR_BGR2GRAY);
cv::threshold(img_gray, img_thresh, 0, 255, THRESH_BINARY | THRESH_OTSU);
namedWindow( window_name, WINDOW_NORMAL );
cv::resizeWindow(window_name, 300, 300);
erode(img_thresh, img_thresh_dilated,getStructuringElement(MORPH_RECT,Size(55,55)));
dilate(img_thresh_dilated, img_thresh_dilated_eroded,getStructuringElement(MORPH_RECT,Size(55,55)));

imshow(window_name,img_thresh_dilated_eroded);


waitKey();


}










