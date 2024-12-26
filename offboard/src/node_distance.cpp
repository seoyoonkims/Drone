#include <ros/ros.h>
#include <ros/console.h>
#include "image_converter.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32MultiArray.h>

// 480 * 640
int centerXPoints[3] = {60, 240, 420};
int centerYPoints[3] = {80, 320, 560};
int SquarePoints[3] = {-10, 0, 10};

ros::Publisher distance_pub;

void callback_Distance(const sensor_msgs::ImageConstPtr& image);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_node");

    ros::NodeHandle nh;
    distance_pub = nh.advertise <std_msgs::Float32MultiArray> ("distance", 10);

    ros::Subscriber img_sub = nh.subscribe ("/camera/depth/image_raw", 10, callback_Distance);
    ros::Rate loop_rate(15);

    ros::spin();
    return 0;
}

void callback_Distance(const sensor_msgs::ImageConstPtr& image)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch(cv_bridge::Exception& e)
    {
       ROS_ERROR("cv_bridge exception\n");
       return;
    }

    cv::Mat depth_image = cv_ptr->image;
    
    std_msgs::Float32MultiArray average_distance;
	std::vector<float> distance_info;

    average_distance.layout.dim.push_back(std_msgs::MultiArrayDimension());
	average_distance.layout.dim.push_back(std_msgs::MultiArrayDimension());
	average_distance.layout.dim[0].size = 0;
	average_distance.layout.dim[1].size = 9;
	average_distance.layout.dim[0].stride = 3;
	average_distance.layout.dim[1].stride = 3;
	average_distance.layout.dim[0].label = "rows";
	average_distance.layout.dim[1].label = "columns";

    

    uint16_t depth_value;

    for(auto X : centerXPoints) {
        for (auto Y : centerYPoints) {
            float distance = 0, total_distance = 0;
            for (auto x : SquarePoints) {
                for (auto y : SquarePoints) {
                    depth_value = depth_image.at<uint16_t> (X + x, Y + y);

                    distance = static_cast<float>(depth_value) * 0.001;   // mm to m

                    if (distance < 0)
                    {
                        ROS_ERROR("depth/distance exception\n");
                        return;
                    }
                    total_distance += distance;

                }
            }
            distance_info.push_back(total_distance / 9);
        }
    }
    average_distance.data.insert(average_distance.data.end(), distance_info.begin(), distance_info.end());
    distance_pub.publish(average_distance);
    return;    

}