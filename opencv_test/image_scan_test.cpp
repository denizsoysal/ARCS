#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;
/*
int main(int, char**)
{
    Mat I = imread("/home/lejoint/Documents/ARCS_Deniz/data/ARCS_dataset_whiteboard/test_image1.pgm",IMREAD_GRAYSCALE);
    Mat J;

    
    int divideWith = 10;
    uchar table[256];
    for (int i = 0; i < 256; ++i)
       table[i] = (uchar)(divideWith * (i/divideWith));

    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    
    for( int i = 0; i < 256; ++i) {
        p[i] = table[i];
    }

    LUT(I, lookUpTable, J);

    imwrite("reduced_image.png",J);
  


    J = close(I,)
    return 0;
}
 */ 


Mat src, dst_morph, detected_edges, dst;
int morph_elem = 0;
int morph_size = 0;
int morph_operator = 0;
int const max_operator = 4;
int const max_elem = 2;
int const max_kernel_size = 40;
const char* window_name = "Test closing and edge detecting";

int lowThreshold = 0;
const int max_lowThreshold = 100;
const int ratiox = 3;
const int kernel_size = 3;


void Morphology_Operations( int, void* );
static void CannyThreshold(int, void*);

int main( int argc, char** argv )
{
  //src = imread("/home/lejoint/Documents/ARCS_Deniz/data/ARCS_dataset_whiteboard/test_image1.pgm",IMREAD_GRAYSCALE);
  src = imread("/home/deniz/Documents/phd/arcs/repository/ARCS/whiteboard.jpeg",IMREAD_GRAYSCALE);
  namedWindow( window_name, WINDOW_AUTOSIZE ); // Create window

  createTrackbar("Operator:\n 0: Opening - 1: Closing  \n 2: Gradient - 3: Top Hat \n 4: Black Hat", window_name, &morph_operator, max_operator, Morphology_Operations );
  createTrackbar( "Kernel size:\n 2n +1", window_name, &morph_size, max_kernel_size,Morphology_Operations );
  createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

  Morphology_Operations( 0, 0 );
  CannyThreshold(0, 0);

  waitKey(0);
  return 0;
}





void Morphology_Operations( int, void* )
{
  int operation = morph_operator + 2;// Since MORPH_X : 2,3,4,5 and 6
  Mat element = getStructuringElement( MORPH_RECT, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
  morphologyEx( src, dst_morph, operation, element );
  imshow( window_name, dst_morph );
}


static void CannyThreshold(int, void*)
{
    blur( dst_morph, detected_edges, Size(3,3) );
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratiox, kernel_size );
    imshow( window_name, detected_edges );
}