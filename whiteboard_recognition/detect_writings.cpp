// #include <opencv2/core.hpp>
// #include <opencv2/videoio.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include "opencv2/imgproc.hpp"
// #include <iostream>
// #include <stdio.h>

// using namespace cv;
// using namespace std;

// //https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html


// int main( int argc, char** argv )
// {
//   cv::Mat image_rgb, image_hsv, thresholded_hsv, dst, cdst, cdstP;;
//   image_rgb = imread("../323102321_1972663339608618_688167511960360177_n.jpg",IMREAD_COLOR);
//   // cvtColor(image_rgb, image_hsv, COLOR_BGR2HSV);

//   // // inRange(image_hsv, Scalar(0, 0, 160), Scalar(255, 255, 255), thresholded_hsv);
//   // std::vector<cv::Mat> hsv_channels;
//   // cv::split(image_hsv, hsv_channels);
//   // cv::Mat h_image = hsv_channels[0];
//   // cv::Mat s_image = hsv_channels[1];
//   // cv::Mat v_image = hsv_channels[2];

//   // namedWindow("final closed image", WINDOW_NORMAL);
//   // imshow("final closed image",h_image);
//   // cv::resizeWindow("final closed image", 300, 300);
//   // cv::waitKey(0);



  

//       // Edge detection
//     Canny(image_rgb, dst, 50, 200, 3);
//     // Copy edges to the images that will display the results in BGR
//     cvtColor(dst, cdst, COLOR_GRAY2BGR);
//     cdstP = cdst.clone();
//     // // Standard Hough Line Transform
//     // vector<Vec2f> lines; // will hold the results of the detection
//     // HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 ); // runs the actual detection
//     // // Draw the lines
//     // for( size_t i = 0; i < lines.size(); i++ )
//     // {
//     //     float rho = lines[i][0], theta = lines[i][1];
//     //     Point pt1, pt2;
//     //     double a = cos(theta), b = sin(theta);
//     //     double x0 = a*rho, y0 = b*rho;
//     //     pt1.x = cvRound(x0 + 1000*(-b));
//     //     pt1.y = cvRound(y0 + 1000*(a));
//     //     pt2.x = cvRound(x0 - 1000*(-b));
//     //     pt2.y = cvRound(y0 - 1000*(a));
//     //     line( cdst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
//     // }
//     // Probabilistic Line Transform
//     vector<Vec4i> linesP; // will hold the results of the detection
//     HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
//     // Draw the lines
//     for( size_t i = 0; i < linesP.size(); i++ )
//     {
//         Vec4i l = linesP[i];
//         line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
//     }
//     // Show results
//       namedWindow("Detected Lines (in red) - Probabilistic Line Transform", WINDOW_NORMAL);
//     cv::resizeWindow("Detected Lines (in red) - Probabilistic Line Transform", 300, 300);
//     imshow("Detected Lines (in red) - Probabilistic Line Transform", dst);
//     // Wait and Exit
//     waitKey();
//     return 0;

// }





// CONTOUR DETECTIOM: 

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;
Mat src_gray;
int thresh = 100;
RNG rng(12345);
void thresh_callback(int, void* );
int main( int argc, char** argv )
{
    Mat src = imread("../323102321_1972663339608618_688167511960360177_n.jpg",IMREAD_COLOR);
    cvtColor( src, src_gray, COLOR_BGR2GRAY );
    blur( src_gray, src_gray, Size(3,3) );
    const char* source_window = "Source";
    namedWindow( source_window );
    imshow( source_window, src );
    const int max_thresh = 255;
    createTrackbar( "Canny thresh:", source_window, &thresh, max_thresh, thresh_callback );
    thresh_callback( 0, 0 );
    waitKey();
    return 0;
}
void thresh_callback(int, void* )
{
    Mat canny_output;
    Canny( src_gray, canny_output, thresh, thresh*2 );
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
    }
    imshow( "Contours", drawing );
}




