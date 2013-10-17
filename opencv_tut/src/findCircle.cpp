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
 
class findcircle
{
    ros::NodeHandle nh_;
    ros::NodeHandle n;
    ros::Publisher pub ;
    image_transport::ImageTransport it_;    
    image_transport::Subscriber image_sub_; //image subscriber 
    image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)
    std_msgs::String msg;

public:
    findcircle()
    : it_(nh_)
    {
        image_sub_ = it_.subscribe("/camera/image_raw", 1, &findcircle::imageCb, this);
        image_pub_= it_.advertise("/camera/image_processed",1);

        namedWindow(WINDOW);
    }

    ~findcircle()
    {
        destroyWindow(WINDOW);
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
 
        Mat src_gray;

        /// Convert it to gray
        cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );

        /// Reduce the noise so we avoid false circle detection
        GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );

        vector<Vec3f> circles;

        /// Apply the Hough Transform to find the circles
        HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 2, 20, 100, 155, 20, 300);              // NEEDS TO BE TUNED

        ROS_INFO("Number of circles = %d", circles.size());

        /// Draw the circles detected
        for( size_t i = 0; i < circles.size(); i++ )
        {
            ROS_INFO("DRAWING CIRCLE");
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }

        /// Show your results
        imshow( WINDOW, cv_ptr->image );

        waitKey(3);   
 
    }
};
 
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "find_circle");
    findcircle ic;
    ros::spin();

    return 0;
}
