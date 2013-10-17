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
 
class imagetracking
{
    ros::NodeHandle nh_;
    ros::NodeHandle n;
    ros::Publisher pub ;
    image_transport::ImageTransport it_;    
    image_transport::Subscriber image_sub_; //image subscriber 
    image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)
    std_msgs::String msg;

public:
    imagetracking()
    : it_(nh_)
    {
        image_sub_ = it_.subscribe("/camera/image_raw", 1, &imagetracking::imageCb, this);
        image_pub_= it_.advertise("/camera/image_processed",1);

        namedWindow(WINDOW);
    }

    ~imagetracking()
    {
        destroyWindow(WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& original_image)
    {
        Mat input = imread("/home/josh/Downloads/squares.png");
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
            ROS_ERROR(    "tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
            return;
        }

        Mat imgHSV, imgThreshold;

        /// Convert it to HSV
        cvtColor( cv_ptr->image, imgHSV, CV_BGR2HSV );
        /// Threshold the image to preset color
        inRange( imgHSV, Scalar(148, 119, 0), Scalar(168, 139, 255), imgThreshold);

        vector<vector<Point> > contours;
        findContours(imgThreshold.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        Mat dst = Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());
        drawContours(dst, contours, -1, Scalar::all(255), CV_FILLED);

        dst &= cv_ptr->image;
        /// Show your results
        imshow( WINDOW, cv_ptr->image);
        imshow( "THRESHOLD IMAGE", imgThreshold);
        imshow( "PROCESSED IMAGE", dst);
        waitKey(3);   
 
    }
};
 
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "find_circle");
    imagetracking ic;
    ros::spin();

    return 0;
}
