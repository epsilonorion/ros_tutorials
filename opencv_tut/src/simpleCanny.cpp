#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>
 
/*here is a simple program which demonstrates the use of ros and opencv to do image manipulations on video streams given out as image topics from the monocular vision
of robots,here the device used is a ardrone(quad-rotor).*/
 
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
 
static const char WINDOW[] = "Image window";
 
class simplecanny
{
  ros::NodeHandle nh_;
  ros::NodeHandle n;
 ros::Publisher pub ;
  image_transport::ImageTransport it_;    
  image_transport::Subscriber image_sub_; //image subscriber 
  image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)
  std_msgs::String msg;
public:
 simplecanny()
    : it_(nh_)
  {
 
     image_sub_ = it_.subscribe("/camera/image_raw", 1, &simplecanny::imageCb, this);
     image_pub_= it_.advertise("/camera/image_processed",1);
 
 
  }
 
  ~simplecanny()
  {
    cv::destroyWindow(WINDOW);
  }
 
  void imageCb(const sensor_msgs::ImageConstPtr& original_image)
  {
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
 
//     sensor_msgs::cv_bridge bridge;//we need this object bridge for facilitating conversion from ros-img to opencv
//	IplImage* img = bridge.imgMsgToCv(msg,"bgr8");  //image being converted from ros to opencv using cv_bridge
      Mat out1;
      Mat gray_out;
      Mat canny_out;
      Mat gray_out1;
      Mat img1;
//      IplImage* out1 = cvCreateImage( cv_ptr->image.size(), IPL_DEPTH_8U, 3 );   //make sure to feed the image(img) data to the parameters necessary for canny edge output 
//      IplImage* gray_out = cvCreateImage( cv_ptr->image.size(), IPL_DEPTH_8U, 1 ); 
//      IplImage* canny_out = cvCreateImage( cv_ptr->image.size(), IPL_DEPTH_8U, 1 );
//      IplImage* gray_out1=cvCreateImage( cv_ptr->image.size(), IPL_DEPTH_8U, 3 );
//      IplImage* img1 = cvCreateImage( cv_ptr->image.  size(), IPL_DEPTH_8U, 3 ); 
      cv::cvtColor(cv_ptr->image, gray_out, CV_BGR2GRAY);
      cv::GaussianBlur(gray_out, gray_out, Size(3, 3), 0, 0);    // Replaces cv::boxFilter(gray_out, gray_out, -1, cv::Size(3,3)); or cvSmooth(gray_out, gray_out, CV_GAUSSIAN, 9,9);
      cv::Canny(gray_out, canny_out, 50, 125, 3);
      cv::cvtColor(canny_out, gray_out1, CV_GRAY2BGR);
      cv::imshow( "CAMERA FEED", cv_ptr->image);
      cv::imshow( "GRAY CAMERA", gray_out);
      cv::imshow( "CANNY CAMERA", canny_out);
      cv::imshow( "CANNY EDGE DETECTION",gray_out1);
      cvWaitKey(2);   
 
}
};
 
 
 
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_canny");
  simplecanny ic;
  ros::spin();
 
  return 0;
}
