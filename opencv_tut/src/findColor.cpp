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

    int rh, rl, gh, gl, bh, bl;

public:
    imagetracking()
    : it_(nh_)
    {
        image_sub_ = it_.subscribe("/camera/image_raw", 1, &imagetracking::imageCb, this);
        image_pub_= it_.advertise("/camera/image_processed",1);

        namedWindow(WINDOW);
        
        rh = 255;
        rl = 100;
        gh = 255;
        gl = 0;
        bh = 70;
        bl = 0;

        createTrackbar("rh", WINDOW, &rh, 255);
        createTrackbar("rl", WINDOW, &rl, 255);
        createTrackbar("gh", WINDOW, &gh, 255);
        createTrackbar("gl", WINDOW, &gl, 255);
        createTrackbar("bh", WINDOW, &bh, 255);
        createTrackbar("bl", WINDOW, &bl, 255);
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
            ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
            return;
        }

        Mat redOnly, blueOnly, greenOnly;
        Mat imgHSV, imgThreshold;
        Mat imgColorThresh;

        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

        /// Convert it to HSV
        cvtColor( cv_ptr->image, imgHSV, CV_BGR2HSV );
        /// Threshold the image to preset color
        inRange( imgHSV, Scalar(bl, gl, rl), Scalar(bh, gh, rh), imgThreshold);
        bitwise_not(imgThreshold, imgThreshold);
        erode(imgThreshold, imgThreshold, Mat());
        dilate(imgThreshold, imgThreshold, element);

        inRange( cv_ptr->image, Scalar(0, 0, 0), Scalar(0, 0, 255), redOnly);
        inRange( cv_ptr->image, Scalar(0, 0, 0), Scalar(255, 0, 0), blueOnly);
        inRange( cv_ptr->image, Scalar(0, 0, 0), Scalar(0, 255, 0), greenOnly);

        inRange( cv_ptr->image, Scalar(bl, gl, rl), Scalar(bh, gh, rh), imgColorThresh);

        /// Show your results
        imshow( WINDOW, cv_ptr->image);
        imshow( "HSV IMAGE", imgHSV);
        imshow( "THRESHOLD IMAGE", imgThreshold);
        imshow( "RGB THRESHOLD IMAGE", imgColorThresh);
        imshow( "RED ONLY", redOnly);
        imshow( "BLUE ONLY", blueOnly);
        imshow( "GREEN ONLY", greenOnly);
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
