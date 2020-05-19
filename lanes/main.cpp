#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>


// DONT PUT LEADING SLASH FOR TOPICS!!
const char *topic_right = "right/image_rect_color";
const char *topic_left = "left/image_rect_color";

void callback(const sensor_msgs::ImageConstPtr &right, const sensor_msgs::ImageConstPtr &left) {
    ROS_INFO(right->encoding.c_str(), left->encoding.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lane");
    ros::NodeHandle nh;

    ROS_INFO("Started");

    image_transport::ImageTransport imageTransport(nh);
    image_transport::SubscriberFilter right(imageTransport, topic_right, 4),
            left(imageTransport, topic_left, 4);
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> > filter(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>(5), right, left);


    ROS_INFO("Made message_filter");

    const message_filters::Connection conn = filter.registerCallback(callback);

    ROS_INFO("Added callback");

    ros::spin();
}