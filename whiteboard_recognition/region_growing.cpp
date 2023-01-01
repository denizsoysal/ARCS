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
    namedWindow( window_name, WINDOW_AUTOSIZE );

    src = imread("../closed_whiteboard.jpg",IMREAD_COLOR);
    if( src.empty() ){
        return EXIT_FAILURE;
    }
    cvtColor( src, src_hsv, COLOR_BGR2HSV );



    // initialize variables
    x_center = src_hsv.size().width/2.0;
    y_center = src_hsv.size().height/2.0;
    right_size = 0.0;
    left_size = 0.0;
    up_size = 0.0;
    down_size = 0.0;
    base_size = 10.0;
    step = 10.0;

    base = src_hsv(Range(y_center - base_size, y_center + base_size), Range(x_center - base_size, x_center + base_size));



    // calculate base histogram
    int h_bins = 180, s_bins = 256, v_bins = 256;
    int histSize[] = { h_bins, s_bins, v_bins};

    float h_ranges[] = { 0, 90 };
    float s_ranges[] = { 0, 128 };
    float v_ranges[] = { 0, 128 };
    const float* ranges[] = { h_ranges, s_ranges , v_ranges};

    int channels[] = { 0, 1, 2};
    calcHist( &base, 1, channels, Mat(), hist_base, 3, histSize, ranges, true, false );
    normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );



    // Loop over extension to all cardinal directions
    for(int i = 0; i < 1000; i++){
        if(up_flag){
            //find extension +1 and +2
            ext_1 = src_hsv(Range(y_center + base_size + up_size, y_center + base_size + up_size + step), 
                        Range(x_center - base_size - left_size, x_center + base_size + right_size));

            ext_2 = src_hsv(Range(y_center + base_size + up_size + step, y_center + base_size + up_size + 2.0*step), 
                        Range(x_center - base_size - left_size, x_center + base_size + right_size));

            // calculate histograms of extensions
            calcHist( &ext_1, 1, channels, Mat(), hist_ext_1, 3, histSize, ranges, true, false );
            normalize( hist_ext_1, hist_ext_1, 0, 1, NORM_MINMAX, -1, Mat() );

            calcHist( &ext_2, 1, channels, Mat(), hist_ext_2, 3, histSize, ranges, true, false );
            normalize( hist_ext_2, hist_ext_2, 0, 1, NORM_MINMAX, -1, Mat() );

            // compare histograms
            corr_1 = compareHist( hist_base, hist_ext_1, HISTCMP_CORREL );
            corr_2 = compareHist( hist_base, hist_ext_2, HISTCMP_CORREL );
            printf("Up: %f, %f \n",corr_1, corr_2);
            
            // if correlation +1 or correlation +2 greater than the threshold
            if(corr_1 > corr_threshold || corr_2 > corr_threshold){
                // update size
                up_size += step;

                // update base
                base = src_hsv(Range(y_center - base_size - down_size, y_center + base_size + up_size),
                           Range(x_center - base_size - left_size, x_center + base_size + right_size));

                // update base histogram
                calcHist( &base, 1, channels, Mat(), hist_base, 3, histSize, ranges, true, false );
                normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );

                // draw extended base
                show = src.clone();
                rectangle(show, Point(x_center - base_size - left_size, y_center + base_size + up_size), 
                                Point(x_center + base_size + right_size, y_center - base_size - down_size), Scalar(0,0,0), 1);
                imshow(window_name,show);
                waitKey(100);
            }
            else{
                up_flag = false;
            }
        }

        if(right_flag){
            //find extension +1 and +2
            ext_1 = src_hsv(Range(y_center - base_size - down_size, y_center + base_size + up_size), 
                        Range(x_center + base_size + right_size, x_center + base_size + right_size + step));

            ext_2 = src_hsv(Range(y_center - base_size - down_size, y_center + base_size + up_size), 
                        Range(x_center + base_size + right_size + step, x_center + base_size + right_size + 2.0*step));

            // calculate histograms of extensions
            calcHist( &ext_1, 1, channels, Mat(), hist_ext_1, 3, histSize, ranges, true, false );
            normalize( hist_ext_1, hist_ext_1, 0, 1, NORM_MINMAX, -1, Mat() );

            calcHist( &ext_2, 1, channels, Mat(), hist_ext_2, 3, histSize, ranges, true, false );
            normalize( hist_ext_2, hist_ext_2, 0, 1, NORM_MINMAX, -1, Mat() );

            // compare histograms
            corr_1 = compareHist( hist_base, hist_ext_1, HISTCMP_CORREL );
            corr_2 = compareHist( hist_base, hist_ext_2, HISTCMP_CORREL );
            printf("Right: %f, %f \n",corr_1, corr_2);
            
            // if correlation +1 or correlation +2 greater than the threshold
            if(corr_1 > corr_threshold || corr_2 > corr_threshold){
                // update size
                right_size += step;

                // update base
                base = src_hsv(Range(y_center - base_size - down_size, y_center + base_size + up_size),
                           Range(x_center - base_size - left_size, x_center + base_size + right_size));

                // update base histogram
                calcHist( &base, 1, channels, Mat(), hist_base, 3, histSize, ranges, true, false );
                normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );

                // draw extended base
                show = src.clone();
                rectangle(show, Point(x_center - base_size - left_size, y_center + base_size + up_size), 
                                Point(x_center + base_size + right_size, y_center - base_size - down_size), Scalar(0,0,0), 1);
                imshow(window_name,show);
                waitKey(100);
            }
            else{
                right_flag = false;
            }
        }

        if(down_flag){
            //find extension +1 and +2
            ext_1 = src_hsv(Range(y_center - base_size - down_size - step, y_center - base_size - down_size), 
                        Range(x_center - base_size - left_size, x_center + base_size + right_size));

            ext_2 = src_hsv(Range(y_center - base_size - down_size - 2.0*step, y_center - base_size - down_size - step), 
                        Range(x_center - base_size - left_size, x_center + base_size + right_size));

            // calculate histograms of extensions
            calcHist( &ext_1, 1, channels, Mat(), hist_ext_1, 3, histSize, ranges, true, false );
            normalize( hist_ext_1, hist_ext_1, 0, 1, NORM_MINMAX, -1, Mat() );

            calcHist( &ext_2, 1, channels, Mat(), hist_ext_2, 3, histSize, ranges, true, false );
            normalize( hist_ext_2, hist_ext_2, 0, 1, NORM_MINMAX, -1, Mat() );

            // compare histograms
            corr_1 = compareHist( hist_base, hist_ext_1, HISTCMP_CORREL );
            corr_2 = compareHist( hist_base, hist_ext_2, HISTCMP_CORREL );
            printf("Down: %f, %f \n",corr_1, corr_2);
            
            // if correlation +1 or correlation +2 greater than threshold
            if(corr_1 > corr_threshold || corr_2 > corr_threshold){
                // update size
                down_size += step;

                // update base
                base = src_hsv(Range(y_center - base_size - down_size, y_center + base_size + up_size),
                           Range(x_center - base_size - left_size, x_center + base_size + right_size));

                // update base histogram
                calcHist( &base, 1, channels, Mat(), hist_base, 3, histSize, ranges, true, false );
                normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );

                // draw extended base
                show = src.clone();
                rectangle(show, Point(x_center - base_size - left_size, y_center + base_size + up_size), 
                                Point(x_center + base_size + right_size, y_center - base_size - down_size), Scalar(0,0,0), 1);
                imshow(window_name,show);
                waitKey(100);
            }
            else{
                down_flag = false;
            }
        }

        if(left_flag){
            //find extension +1 and +2
            ext_1 = src_hsv(Range(y_center - base_size - down_size, y_center + base_size + up_size), 
                        Range(x_center - base_size - left_size - step, x_center - base_size - left_size));

            ext_2 = src_hsv(Range(y_center - base_size - down_size, y_center + base_size + up_size), 
                        Range(x_center - base_size - left_size - 2.0*step, x_center - base_size - left_size - step));

            // calculate histograms of extensions
            calcHist( &ext_1, 1, channels, Mat(), hist_ext_1, 3, histSize, ranges, true, false );
            normalize( hist_ext_1, hist_ext_1, 0, 1, NORM_MINMAX, -1, Mat() );

            calcHist( &ext_2, 1, channels, Mat(), hist_ext_2, 3, histSize, ranges, true, false );
            normalize( hist_ext_2, hist_ext_2, 0, 1, NORM_MINMAX, -1, Mat() );

            // compare histograms
            corr_1 = compareHist( hist_base, hist_ext_1, HISTCMP_CORREL );
            corr_2 = compareHist( hist_base, hist_ext_2, HISTCMP_CORREL );
            printf("Left: %f, %f \n",corr_1, corr_2);
            
            // if correlation +1 or correlation +2 greater than the threshold
            if(corr_1 > corr_threshold || corr_2 > corr_threshold){
                // update size
                left_size += step;

                // update base
                base = src_hsv(Range(y_center - base_size - down_size, y_center + base_size + up_size),
                           Range(x_center - base_size - left_size, x_center + base_size + right_size));

                // update base histogram
                calcHist( &base, 1, channels, Mat(), hist_base, 3, histSize, ranges, true, false );
                normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );

                // draw extended base
                show = src.clone();
                rectangle(show, Point(x_center - base_size - left_size, y_center + base_size + up_size), 
                                Point(x_center + base_size + right_size, y_center - base_size - down_size), Scalar(0,0,0), 1);
                imshow(window_name,show);
                waitKey(100);
            }
            else{
                left_flag = false;
            }
        }
    }

    waitKey(0);
    return 0;
}