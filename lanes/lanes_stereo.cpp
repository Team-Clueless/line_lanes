//  This is a small module that runs a callback with the latest images from both cameras.
// Currently does nothing.

#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// Bunch of objects that are useful in the callback
struct Helpers {
    sensor_msgs::CameraInfo::ConstPtr camera_info;

    image_transport::Publisher pub_right, pub_left;

    float height;
};

const char *topic_right = "right/image_rect_color";
const char *topic_left = "left/image_rect_color";
const char *topic_camera_info = "left/camera_info";

const char *pub_topic_right = "right/image_masked";
const char *pub_topic_left = "left/image_masked";


void callback(const sensor_msgs::ImageConstPtr &msg_right, const sensor_msgs::ImageConstPtr &msg_left,
              Helpers &helper) {
    try {
        cv_bridge::CvImagePtr cv_img_left = cv_bridge::toCvCopy(msg_left, "bgr8"), cv_img_right = cv_bridge::toCvCopy(
                msg_right, "bgr8");

    } catch (std::exception &e) {
        ROS_ERROR("Callback failed: %s", e.what());
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lane");
    ros::NodeHandle nh;

    ROS_INFO("Started");

    // Create both the subscribers:
    image_transport::ImageTransport imageTransport(nh);
    image_transport::SubscriberFilter right(imageTransport, topic_right, 4),
            left(imageTransport, topic_left, 4);


    // Create the sync thing
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>>
            filter(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>(5),
                   right, left);


    Helpers helper{
            ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic_camera_info)
            imageTransport.advertise(pub_topic_right, 1),
            imageTransport.advertise(pub_topic_left, 1),
    };


    // Get the height from the ground.
    {
        tf::TransformListener listener;
        while (true) {
            try {
                tf::StampedTransform transform;
                listener.lookupTransform("/ground", helper.model_left.tfFrame().c_str(), ros::Time(0), transform);
                helper.height = transform.getOrigin().z();
                ROS_INFO("Height of camera from ground: %f3", helper.height);
                break;
            } catch (const std::exception &e) {
                ROS_WARN("Transform not found, sleeping.");
                ros::Duration(1).sleep();
            }
        }
    }

    // Adds the callback
    const message_filters::Connection conn = filter.registerCallback(
            boost::bind(&callback, _1, _2, boost::ref(helper)));

    ROS_INFO("Spinning...");

    ros::spin();
}