/*

Title: Algorithm that performs closing until we reach a plateau in our homogenity metric

Explanation:
- We want to perform closing on the image to be able to obtain a smooth surface for the 
whiteboard (remove writing etc)
- What we do is performing closing 
- Then blurring the closed image, compute the sum of absolute difference between 
the closed image and the blurred closed image --> this is our non-homogenity metric
- Continue (increase the kernel of the closing operation)until the (non-homogenity metric at 
time t+1)/(non-homogenity metric at time t)>threshold, with threshold being e.g. 0.7
(this will mean that the non-homogenity does not increase that much)

- Note that the non-homogenity metric decreases as the algorithm continue (because the 
homogenity increases)

- Note that as a first preprocessing step, we apply median blur to be able to remove the "salt and 
pepper noise". We consider writing to be this kind of noise because they are high frequency
and low size in space compared to other object in the image

Assumptions:
- We make the hypothesis that we are close to the whiteboard, 
and that it occupies most of the image 
- We make the hypothesis as the writing on the whiteboard are 
smaller in size in the image frame compared to other object 
on the image. So when doing closing they will disappear before the objects  



P.S. : DOING IT WITH MEDIAN BLURRING INSTEAD OF CLOSING SEAMS BETTER
P.S.2 : AS A HOMOGENEOUS METRIC, FFT CAN ALSO BE CONSIDERED

*/



#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include <cmath>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{

  //define the matrix we will need
  cv::Mat src, image, sum, squared_diff, image_blurred, absolute_diff, closed_image, closed_image_blurred;
  
  //read image
  image = imread("../322915141_1210091463221811_7344717193191901619_n.jpg",IMREAD_COLOR);

  //apply median blur 
  // medianBlur(src,image,33);

  //Blur the image with 3x3 Gaussian kernel
  GaussianBlur(image, image_blurred, Size(11, 11), 0);

  //compute pixel wise absolute diff between image and blurred one 
  absdiff(image, image_blurred, absolute_diff); 	

  //squared diff
  squared_diff = absolute_diff.mul(absolute_diff);

  //compute sum 
  double sum_absolute_diff_initial = cv::sum(cv::sum(squared_diff))[0];
  std::cout << "initial loss: " << sum_absolute_diff_initial << "\n";

  //create an array of 2 elements: previous loss, current loss
  double loss_array [2];
  //previous loss at the beginning: initial loss
  loss_array[0] = sum_absolute_diff_initial;

  //loop for different kernel size (i) until we reach a plateau (minimum) 
  //in term of loss (non-homogenity)
  for(int i=0; i<=50; i++){

    //closing operation 
    // morphologyEx(image, closed_image, MORPH_CLOSE,getStructuringElement(MORPH_RECT,Size(2*i+1,2*i+1)));
    medianBlur(image,closed_image,2*i+1);
    //Blur the closed image with 3x3 Gaussian kernel
    GaussianBlur(closed_image, closed_image_blurred, Size(11, 11), 0);

    //compute pixel wise absolute diff between image and blurred one 
    absdiff(closed_image, closed_image_blurred, absolute_diff); 	

     //squared diff
    squared_diff = absolute_diff.mul(absolute_diff);

    //compute sum 
    double sum_absolute_diff = cv::sum(cv::sum(squared_diff))[0];
    std::cout << "loss of the closed image: " <<sum_absolute_diff << "\n";

    //add this loss as current loss
    loss_array[1] = sum_absolute_diff;

    //compute ratio between current loss and previous loss
    double ratio = loss_array[1] / loss_array[0];
    std::cout << "loss improvement is: " << 1/ratio << "\n";

    //if this ratio is higher than 0.8, this means that we don't improve a lot
    //this threshold is user-defined
      if(i>0 and ratio > 0.96){

      std::cout << "Loss plateau reached !" << "\n";
      std::cout<< "Final 'i' is:" << i << "\n" << "Final kernel size is then:" << 2*i+1 << "\n";


      imwrite("test.jpg", closed_image); // A JPG FILE IS BEING SAVED


      namedWindow("final closed image", WINDOW_NORMAL);
      cv::resizeWindow("final closed image", 300, 300);
      imshow("final closed image",closed_image);
      waitKey();

      break;

    }

    //if ratio threshold not reached, add this loss as previous loss
    loss_array[0] = sum_absolute_diff;

  }

  return 0;

}

