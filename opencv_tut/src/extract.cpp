#include <opencv/cv.h>

using namespace cv;

extract::extract(const Mat &img) {
	split(img, channelsRGB);

	cv::Mat imgHSV; cvtColor(img, imgHSV, CV_BGR2HSV);
	split(imgHSV, channelsHSV);

	cv::Mat imgLAB; cvtColor(img, imgLAB, CV_RGB2Lab);
	split(imgLAB, channelsLAB);
}
