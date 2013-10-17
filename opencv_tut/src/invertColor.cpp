//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
 
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
 
//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
 
//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
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
 
    int height = cv_ptr->image.rows;
    int width = cv_ptr->image.cols;
    int step = cv_ptr->image.step1();
    int channel = cv_ptr->image.channels();
    int depth = cv_ptr->image.depth();

//    for(int i=0; i<cv_ptr->image.rows; i++)     // Height
//    {
//        //Go through all the columns
//        for(int j=0; j<cv_ptr->image.cols; j++) // Width
//        {
//            //Go through all the channels (b, g, r)
//            for(int k=0; k<cv_ptr->image.channels(); k++)
//            {
//                //Invert the image by subtracting image data from 255               
//                cv_ptr->image.data[i*step+j*channel+k]=255-cv_ptr->image.data[i*step+j*channel+k];
//            }
//        }
//    }

    cv::bitwise_not(cv_ptr->image, cv_ptr->image);
//    cv_ptr->image = 255 - cv_ptr->image;
     
    //Display the image using OpenCV
    cv::imshow(WINDOW, cv_ptr->image);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    pub.publish(cv_ptr->toImageMsg());
}
 
/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
    ROS_INFO("opencv_Tutorials Starting.");
    ros::init(argc, argv, "image_processor");

    ros::NodeHandle nh;

    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);
    //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
    
    //OpenCV HighGUI call to destroy a display window on shut-down.
    cv::destroyWindow(WINDOW);

    pub = it.advertise("camera/image_processed", 1);

    ros::spin();
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}
