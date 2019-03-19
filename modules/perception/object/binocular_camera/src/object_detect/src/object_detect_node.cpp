#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <dirent.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

extern "C" void init_disparity_method(const uint8_t _p1, const uint8_t _p2);
extern "C" cv::Mat compute_disparity_method(cv::Mat left, cv::Mat right, float *elapsed_time_ms, const char* directory, const char* fname);

bool debug = true;
// SGM
struct dirent *ep;
float elapsed_time_ms;
const char* directory = "void";

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	try {
		cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat left_img = image.colRange(0, image.cols/2);
        cv::Mat right_img = image.colRange(image.cols/2, image.cols);
        cv::Mat left_img_gray, right_img_gray;
        cv::cvtColor(left_img, left_img_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_img, right_img_gray, cv::COLOR_BGR2GRAY);
        cv::Mat disparity_img = compute_disparity_method(left_img_gray, right_img_gray, &elapsed_time_ms, directory, ep->d_name);
        if (debug) {
            cv::Mat disparity_show = cv::Mat::zeros(disparity_img.size(), disparity_img.type());
            cv::applyColorMap(disparity_img, disparity_show, cv::COLORMAP_JET);
            cv::imshow("disparity image", disparity_show);
            cv::waitKey(1);
        }
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_detect");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	init_disparity_method(6, 96);
	image_transport::Subscriber sub = it.subscribe("/mini_binocular/image", 1, imageCallback);
	ros::spin();
	cv::destroyAllWindows();
	return 0;
}