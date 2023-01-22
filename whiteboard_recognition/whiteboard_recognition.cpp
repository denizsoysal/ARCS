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


double* histogram_change_detect(cv::Mat input_to_detect, int direction)
{
    // Get the size of the image
    int rows = input_to_detect.rows;
    int cols = input_to_detect.cols;

    // Create the output image
    Mat image;

    // Resize the image
    resize(input_to_detect, image, Size(cols/2,rows/2),0,0,INTER_LINEAR);

    // Convert to grayscale
    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);

    // Set initial mask position to center of image
    int maskX = gray.cols / 2;
    int maskY = gray.rows / 2;
    int maskWidth = 25;
    int maskHeight = 25;

    // Set threshold for histogram difference
    float threshold = 0.80;
    float range[] = { 0, 255 };
    const float* histRange = { range };

    // Initialize histograms
    Mat currentHist, nextHist;
    int channels[] = { 0 };

    // Set step size for moving mask
    int step = maskWidth;
    int i = 0;
    int nextX,nextY;

    //create struc to store point and corresponding value (to look at the max later)
    struct Point_with_diff {
      int x, y;
      float diff;
      Point_with_diff(int a, int b, float difference) { this->x = a; this->y = b; this->diff=difference;}
    };



    // Move mask in different directions
    while (true) {

        // Get current masked image
        Rect roi(maskX, maskY, maskWidth, maskHeight);
        if (roi.x < 0) roi.x = 0;
        if (roi.y < 0) roi.y = 0;
        if (roi.x + roi.width > gray.cols) roi.width = gray.cols - roi.x;
        if (roi.y + roi.height > gray.rows) roi.height = gray.rows - roi.y;
        Mat currentMask(gray, roi);

        // Calculate histogram of current masked image
        calcHist( &currentMask, 1, channels, Mat(), currentHist, 1, &maskWidth, &histRange);
        if(direction == 0){
          //go up right 
            nextX = maskX;
            nextY = maskY;
            nextX += step;
            nextY += step;
        }

        else if (direction== 1) {
          //go up left
            nextX = maskX;
            nextY = maskY;
            nextX -= step;
            nextY += step;
        }


        else if (direction== 2) {
          //go down right
            nextX = maskX;
            nextY = maskY;
            nextX += step;
            nextY -= step;
        }

        else if(direction == 3){
          //go down left
            nextX = maskX;
            nextY = maskY;
            nextX -= step;
            nextY -= step;
        }




        if(nextY > gray.rows or nextX > gray.cols ){
            cout<<"Whiteboard not detected"<<endl;
            break;
        }


        // Get next masked image
        Rect roiNext(nextX, nextY, maskWidth, maskHeight);
        if (roiNext.x < 0) roiNext.x = 0;
        if (roiNext.y < 0) roiNext.y = 0;
        if (roiNext.x + roiNext.width > gray.cols) roiNext.width = gray.cols - roiNext.x;
        if (roiNext.y + roiNext.height > gray.rows) roiNext.height = gray.rows - roiNext.y;
        Mat nextMask(gray, roiNext);
        imshow("next",nextMask);
        waitKey();
        // Calculate histogram of next masked image
        calcHist( &nextMask, 1, channels, Mat(), nextHist, 1, &maskWidth, &histRange,HISTCMP_INTERSECT);
        // Compare histograms
        double histDiff = compareHist(currentHist, nextHist, HISTCMP_BHATTACHARYYA);

        cout << histDiff << "\n";
           std::cout.flush();

        // If histogram difference is above threshold, consider it an edge
        if (histDiff > threshold and histDiff < 2*threshold and i>1) {
            // Draw line on image
            line(image, Point(maskX, maskY), Point(nextX, nextY), Scalar(0, 255, 0), 2);
            break;
        }
        

        // Update mask position
        maskX = nextX;
        maskY = nextY;
        i = i+1;
    }

    // Show image with lines
    namedWindow("Edges", WINDOW_NORMAL);
    cv::resizeWindow("Edges", 300, 300);
    imshow("Edges", image);
    waitKey();

    
    //we declare the array as static
    static double coord_array [4];
    coord_array[0] = (maskX, maskY);
    coord_array[1] = (nextX, nextY);

    std::cout << coord_array[0] << "\n";
    std::cout << coord_array[1];

    return coord_array;

}


int main( int argc, char** argv )
{ 
    const char* window_name = "Region Growing";
    namedWindow( window_name, WINDOW_NORMAL );
    cv::resizeWindow(window_name, 300, 300);


    cv::Mat closed_image = adaptive_median_filtering("../323531059_3414989708821240_2898831769865535112_n.jpg");
    // //get address of coordinate:
    // Mat immm = imread("../323527929_737669997789369_8093382492363965159_n.jpg");
    // //read image
    // Mat closed_image;
    // closed_image.convertTo(immm,  CV_32F, 1.0/255);

    histogram_change_detect(closed_image,0);

    // Point center(detected[0] , detected[1]);//Declaring the center point
    // int radius = 30; //Declaring the radius
    // Scalar line_Color(0, 0, 0);//Color of the circle
    // int thickness = 5;//thickens of the line
    // circle(closed_image, center,radius, line_Color, thickness);//Using circle()function to draw the line//
    // imshow(window_name, closed_image);//Showing the circle//
    // waitKey();
    // std::cout << detected[0] << "  " << detected[1] << "  "  ;
    // cv::Mat image_after_otsu =  segment_otsu(closed_image);
    // imwrite("../allonsons.jpg",image_after_otsu);
    // cv::Mat detected_contour = detect_contour(closed_image);
}

