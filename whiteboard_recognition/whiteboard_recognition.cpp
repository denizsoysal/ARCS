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
#include "opencv2/features2d.hpp"
#include <cmath>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

cv::Mat adaptive_median_filtering(std::string path_to_img)
{
  //define the matrix we will need
  cv::Mat src, image, sum, squared_diff, image_blurred, absolute_diff, closed_image, closed_image_blurred;
  //read image
  image = imread(path_to_img,IMREAD_COLOR);
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
      if(i>0 and ratio > 0.98){
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
  return closed_image;
}

cv::Mat segment_otsu(cv::Mat image_to_segment)
{
  const char* window_name = "Region Growing";

  Mat img_gray,img_thresh,img_thresh_dilated,img_thresh_dilated_eroded;
  cvtColor( image_to_segment, img_gray, COLOR_BGR2GRAY);
  cv::threshold(img_gray, img_thresh, 0, 255, THRESH_BINARY | THRESH_OTSU);
  namedWindow( window_name, WINDOW_NORMAL );
  cv::resizeWindow(window_name, 300, 300);
  erode(img_thresh, img_thresh_dilated,getStructuringElement(MORPH_RECT,Size(55,55)));
  dilate(img_thresh_dilated, img_thresh_dilated_eroded,getStructuringElement(MORPH_RECT,Size(55,55)));

  imshow(window_name,img_thresh_dilated_eroded);


  waitKey();

  return img_thresh_dilated_eroded;
}

cv::Mat detect_contour(cv::Mat segmented_image)
{
    const char* window_name = "Region Growing";
    namedWindow( window_name, WINDOW_NORMAL );
  cv::resizeWindow(window_name, 300, 300);


    Mat src_gray, sobelx, sobely;
    RNG rng(12345);
    cvtColor( segmented_image, src_gray, COLOR_BGR2GRAY );
    blur( src_gray, src_gray, Size(3,3) );
    const char* source_window = "Source";

    //canny
    Mat canny_output;
    Canny( src_gray, canny_output, 30 , 60 );

    //sobel
    Sobel(src_gray, sobelx, CV_64F, 1, 0, 3);
    Sobel(src_gray, sobely, CV_64F, 0, 1, 3);
    Mat abs_grad_x;
    Mat abs_grad_y; 
    convertScaleAbs(sobelx,abs_grad_x);
    convertScaleAbs(sobely, abs_grad_y);

    Mat grad;
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0,grad);

    Mat grad_blurred;
    GaussianBlur(grad, grad_blurred, Size(11, 11), 0);


    imshow(window_name,grad_blurred);
    waitKey();


    //we find the contour in a tree hierarchy
    vector<vector<Point> > contours;
    vector<vector<Point> > approx_contour;
    vector<Vec4i> hierarchy;
    findContours( grad_blurred, contours, hierarchy, cv::RETR_EXTERNAL, CHAIN_APPROX_NONE );
    Mat drawing = Mat::zeros( grad_blurred.size(), CV_8UC3 );



    //draz all contour:
     for( size_t i = 0; i< contours.size(); i++ ){
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        approxPolyDP(contours[i],contours[i],100,1);
        drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
        std::cout << contours[i].size();
        // putText(drawing,std::to_string(contours[i].size()),center,1,3,Scalar(255,0,0),2);

    }

    //blob detection
    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 100;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.1;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.87;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;

    // Storage for blobs
    std::vector<KeyPoint> keypoints;

#if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2

    // Set up detector with params
    SimpleBlobDetector detector(params);

    // Detect blobs
    detector.detect(drawing, keypoints);
#else 

    // Set up detector with params
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    // Detect blobs
    detector->detect(drawing, keypoints);
#endif 

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
    // the size of the circle corresponds to the size of blob

    Mat im_with_keypoints;
    drawKeypoints(drawing, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);




    // //we consider the whiteboard being in the inner contour
    // //so we take the last contour in our contour hierarchy
    // int i = contours.size()-1;
    // Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
    // approxPolyDP(contours[i],contours[i],100,1);
    // drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
    // std::cout << contours[i].size();

    //now apply blob detection to find the biggest black region in the image 
    //based on this:
    //https://learnopencv.com/blob-detection-using-opencv-python-c/

    namedWindow("final closed image", WINDOW_NORMAL);
    imshow("final closed image",im_with_keypoints);
    cv::resizeWindow("final closed image", 300, 300);
    cv::waitKey(0);
    waitKey();
    return drawing;

}


double* histogram_change_detect(cv::Mat input_to_detect)
{
  //this function detect big changes in the histogram
  //we apply a mask to the image each time to only consider a subset of the image
  //we compute the histogram
  //we move the mask 
  //we look at the difference 

  //this function return the address of the double array storing the coordinate of the
  //detected mask

  const char* window_name = "Region Growing";
  namedWindow( window_name, WINDOW_NORMAL );
  cv::resizeWindow(window_name, 300, 300);

  //first define the masked input 
  Mat masked, mask, second_mask, second_masked, diff, squared, sum_squared;
  mask = cv::Mat::zeros(input_to_detect.size(), CV_8U); // all 0
  second_mask = cv::Mat::zeros(input_to_detect.size(), CV_8U); // all 0
  //we start at the middle of the image (input_to_detect.size().width)/2, (input_to_detect.size().height)/2
  //and draw a rectange of the size of the image divided by 30
  int i;
  int size = 40;
  //loop for different kernel size (i) until we reach a plateau (minimum) 
  //in term of loss (non-homogenity)

  //create an array of 2 elements: previous diff, current diff
  double diff_array [2];
  
  for(i=0; i<=20; i++){


    cout << "the value of i is:" << i << "\n";
    mask(Rect(((input_to_detect.size().width)/2)+i*(input_to_detect.size().width)/size, ((input_to_detect.size().height)/2)+i*(input_to_detect.size().height)/size, (input_to_detect.size().width)/size, (input_to_detect.size().height)/size)) = 255;
    second_mask(Rect(((input_to_detect.size().width)/2)+(i+1)*(input_to_detect.size().width)/size, ((input_to_detect.size().height)/2)+(i+1)*(input_to_detect.size().height)/size, (input_to_detect.size().width)/size, (input_to_detect.size().height)/size)) = 255;
    //now we apply the mask on the image
    input_to_detect.copyTo(masked, mask);
    //now we apply the mask on the image
    input_to_detect.copyTo(second_masked, second_mask);
    imshow(window_name, second_masked);
    waitKey();

    
    // instead of pixel wise as here, do it with histogram comparison:
    // https://docs.opencv.org/3.4/d8/dc8/tutorial_histogram_comparison.html

    //first go to HSV spac
    Mat hsv_masked, hsv_second_masked;
    cvtColor( masked, hsv_masked, COLOR_BGR2HSV );
    cvtColor( second_masked, hsv_second_masked, COLOR_BGR2HSV );

    int h_bins = 50, s_bins = 60;
    int histSize[] = { h_bins, s_bins };
    // hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };
    const float* ranges[] = { h_ranges, s_ranges };
    // Use the 0-th and 1-st channels
    int channels[] = { 0, 1 };
    Mat hist_masked, hist_second_masked;

    calcHist( &hsv_masked, 1, channels, Mat(), hist_masked, 2, histSize, ranges, true, false );
    normalize( hist_masked, hist_masked, 0, 1, NORM_MINMAX, -1, Mat() );
    calcHist( &hsv_second_masked, 1, channels, Mat(), hist_second_masked, 2, histSize, ranges, true, false );
    normalize( hist_second_masked, hist_second_masked, 0, 1, NORM_MINMAX, -1, Mat() );

    float diff = compareHist( hist_second_masked, hist_masked, 1);
    cout << "value of diff is" <<  diff << "\n";

    //store intial diff
    if(i==0){
      diff_array[1] = diff;
    }

    //add this loss as current loss
    diff_array[2] = diff;
    std::cout << diff_array[1] << " and: " << diff_array[2] << "\n";
    //compute ratio between current loss and previous loss
    float ratio = diff_array[2] / diff_array[1];
    std::cout << "loss is: " << ratio << "\n";
    diff_array[1] = diff;
    //if this ratio is higher than 0.8, this means that we don't improve a lot
    //this threshold is user-defined
    if(i>3 and ratio > 1.3){
      std::cout << "Loss plateau reached !" << "\n";
      imshow(window_name,second_masked);
      waitKey();
      break;

    }
    //if ratio threshold not reached, add this loss as previous loss
    
    



  }


  //we declare the array as static
  static double coord_array [4];
  coord_array[0] = ((input_to_detect.size().width)/2)+(i+1)*(input_to_detect.size().width)/size;
  coord_array[1] = ((input_to_detect.size().height)/2)+(i+1)*(input_to_detect.size().height)/size;

  return coord_array;
}


int main( int argc, char** argv )
{ 
    const char* window_name = "Region Growing";
    namedWindow( window_name, WINDOW_NORMAL );
    cv::resizeWindow(window_name, 300, 300);


    cv::Mat closed_image = adaptive_median_filtering("../322915141_1210091463221811_7344717193191901619_n.jpg");
    //get address of coordinate:
    double* detected;
    detected =  histogram_change_detect(closed_image);

    Point center(detected[0] , detected[1]);//Declaring the center point
    int radius = 10; //Declaring the radius
    Scalar line_Color(0, 0, 0);//Color of the circle
    int thickness = 2;//thickens of the line
    circle(closed_image, center,radius, line_Color, thickness);//Using circle()function to draw the line//
    imshow(window_name, closed_image);//Showing the circle//
    waitKey();
    std::cout << detected[0] << "  " << detected[1] << "  "  ;
    // cv::Mat image_after_otsu =  segment_otsu(closed_image);
    // imwrite("../allonsons.jpg",image_after_otsu);
    // cv::Mat detected_contour = detect_contour(closed_image);
}

