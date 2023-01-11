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

//try this shit out:
//https://stackoverflow.com/questions/71446868/about-opencv-approxpolydp-function-program-cant-find-rectangle-that-i-want-if

// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/imgproc.hpp"
// #include <iostream>
// using namespace cv;
// using namespace std;
// Mat src_gray;
// int thresh = 100;
// RNG rng(12345);
// void thresh_callback(int, void* );
// int main( int argc, char** argv )
// {
//     Mat src = imread("../aaa.png",IMREAD_COLOR);
//     cvtColor( src, src_gray, COLOR_BGR2GRAY );
//     blur( src_gray, src_gray, Size(3,3) );
//     const char* source_window = "Source";
//     namedWindow( source_window );
//     imshow( source_window, src );
//     const int max_thresh = 255;
//     createTrackbar( "Canny thresh:", source_window, &thresh, max_thresh, thresh_callback );
//     thresh_callback( 0, 0 );
//     waitKey();
//     return 0;
// }
// void thresh_callback(int, void* )
// {
//     Mat canny_output;
//     Canny( src_gray, canny_output, thresh, thresh*2 );
//     vector<vector<Point> > contours;
//     vector<vector<Point> > approx_contour;
//     vector<Vec4i> hierarchy;
//     findContours( canny_output, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
//     Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
//     // // select last contour find 
//     // int i = contours.size()-1;
//     //draz all contour:
//      for( size_t i = 0; i< contours.size(); i++ ){
//         Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
//         approxPolyDP(contours[i],contours[i],100,1);
//         drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
//         std::cout << contours[i].size();
//         // putText(drawing,std::to_string(contours[i].size()),center,1,3,Scalar(255,0,0),2);

//     }

//     for(vector<Point>p:contours)
//         for(Point k:p)
//             circle(drawing,k,5,Scalar(255,255,255),FILLED);
            

//     namedWindow("final closed image", WINDOW_NORMAL);
//     imshow("final closed image",drawing);
//     cv::resizeWindow("final closed image", 300, 300);
//     cv::waitKey(0);

// }



// CONTOUR DETECTIOM: 

//https://stackoverflow.com/questions/13495207/opencv-c-sorting-contours-by-their-contourarea
// comparison function object
bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i < j );
}



//https://stackoverflow.com/questions/71446868/about-opencv-approxpolydp-function-program-cant-find-rectangle-that-i-want-if

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;

int thresh = 100;

int main(){





// I used cvtColor, inRange , bitwise_and, GaussianBlur, Canny functions before.
vector<vector<Point>> contours; // to store contours
vector<Vec4i> hierarchy;

int mode = RETR_CCOMP;
//int module = CHAIN_APPROX_NONE;
//int module = CHAIN_APPROX_SIMPLE;
//int module = CHAIN_APPROX_TC89_L1;
int module = CHAIN_APPROX_TC89_KCOS;

Mat canny_output,src_gray;
Mat src = imread("../aaa.png",IMREAD_COLOR);
cvtColor(src,src_gray,COLOR_BGR2GRAY    );
blur( src_gray, src_gray, Size(3,3) );
Canny( src_gray, canny_output, thresh, thresh*2 );


findContours(canny_output, contours, hierarchy, mode, module);

sort(contours.begin(), contours.end(), compareContourAreas);
drawContours(image, contours, -1, Scalar(255, 0, 0), 1);
imshow("ContourImage", image);

vector<Point> contours_approx;

for (int i = 0; i < 5; i++)
{
    double length = arcLength(contours[i], true);
    approxPolyDP(contours[i], contours_approx, 0.1 * length, true);

    if (contours_approx.size() == 4)
    {
        break;
    }
    contours_approx.clear();
}

for (Point i : contours_approx)
{
    cout << i << endl; // to check coordinate of vertex
}

drawContours(contour2, vector<vector<Point>>(1, contours_approx), -1, Scalar(0, 0, 255), 1);

// to find vertex
circle(contour2, Point(x0, y0), 5, Scalar(255, 0, 0), -1, -1, 0); // blue
circle(contour2, Point(x1, y1), 5, Scalar(0, 255, 0), -1, -1, 0); // green
circle(contour2, Point(x2, y2), 5, Scalar(0, 0, 255), -1, -1, 0); // red
circle(contour2, Point(x3, y3), 5, Scalar(255, 255, 255), -1, -1, 0); // white
imshow("contour2", contour2);



}
