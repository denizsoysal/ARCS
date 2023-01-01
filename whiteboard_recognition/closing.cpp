#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

void Morphology_Operations( int, void* );





Mat src, dst_morph, closed_whiteboard;
int morph_size = 0;
int morph_operator = 0;
int const max_operator = 4;
int const max_kernel_size_morph = 40;

const char* window_name = "Closing";



int main( int argc, char** argv )
{
  src = imread("../whiteboard.jpg",IMREAD_COLOR);
  namedWindow( window_name, WINDOW_AUTOSIZE );

  createTrackbar("Operator:\n 0: Opening - 1: Closing  \n 2: Gradient - 3: Top Hat \n 4: Black Hat", window_name, &morph_operator, max_operator, Morphology_Operations );
  createTrackbar( "Kernel size:\n 2n +1", window_name, &morph_size, max_kernel_size_morph, Morphology_Operations );
  Morphology_Operations( 0, 0 );

  waitKey(0);

  // int operation_save = 1 + 2;// Since MORPH_X : 2,3,4,5 and 6
  // Mat element_save = getStructuringElement( MORPH_RECT, Size( 2*1 + 1, 2*1+1 ), Point( 1, 1 ) );
  // morphologyEx( src, closed_whiteboard, operation_save, element_save );

  // imwrite("/Downloads/closed_whiteboard.jpg",closed_whiteboard);
  return 0;
}


void Morphology_Operations( int, void* )
{
  int operation = morph_operator + 2;// Since MORPH_X : 2,3,4,5 and 6
  Mat element = getStructuringElement( MORPH_RECT, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
  morphologyEx( src, dst_morph, operation, element );
  imshow( window_name, dst_morph );
  imwrite("closed_whiteboard_v2.jpg",dst_morph);
  
}