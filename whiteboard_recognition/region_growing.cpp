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
double x_center, y_center, right_size, left_size, up_size, down_size, base_size, step, corr_1, corr_2;
double corr_threshold = 0.9;
bool left_flag =  true;
bool right_flag = true;
bool up_flag = true;
bool down_flag = true;
Mat src, src_hsv, base, hist_base, ext_1, ext_2, hist_ext_1, hist_ext_2, show;

int main( int argc, char** argv )
{
    //define the matrix we will need
    cv::Mat src, image, diff , gray,squared_diff, image_blurred, absolute_diff, closed_image, closed_image_blurred;
    
    //read image
    image = imread("../322915141_1210091463221811_7344717193191901619_n.jpg",IMREAD_COLOR);
    
    //read image
    src = imread("test.jpg",IMREAD_COLOR);

    //substract one image from   another
    absdiff(image, src, diff);

    Mat planes[3];
    split(diff,planes);  // planes[2] is the red channel

    // //threshold on saturation level in hsv space 
    // //https://stackoverflow.com/questions/17185151/how-to-obtain-a-single-channel-value-image-from-hsv-image-in-opencv-2-1
    // Mat hsv_image;
    // cvtColor(diff, hsv_image, COLOR_BGR2HSV);

    // std::vector<cv::Mat> hsv_channels;
    // cv::split(hsv_image, hsv_channels);
    // cv::Mat h_image = hsv_channels[0];
    // cv::Mat s_image = hsv_channels[1];
    // cv::Mat v_image = hsv_channels[2];


    namedWindow("final closed image", WINDOW_NORMAL);
      cv::resizeWindow("final closed image", 300, 300);
      imshow("final closed image",diff);
      waitKey();




    waitKey();
    return 0;
}