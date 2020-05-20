//  NOT READY

#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv/cv.h>

#include <dynamic_reconfigure/server.h>
#include <igvc_bot/LanesConfig.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

struct Helpers {
    ros::Publisher pub_point_cloud;
    image_transport::Publisher pub_right, pub_left;

    // Params:
    cv::Scalar white_lower, white_upper;
    uint8_t erosion_size, erosion_iter;
    cv::Mat erosion_element;

    image_geometry::PinholeCameraModel model_left; // TODO change to stereo
    float height;
};

const char *topic_right = "right/image_rect_color";
const char *topic_left = "left/image_rect_color";
const char *topic_left_camera = "left/camera_info";

const char *pub_topic_right = "right/image_masked";
const char *pub_topic_left = "left/image_masked";


const char *points_topic = "points2";


void callback(const sensor_msgs::ImageConstPtr &msg_right, const sensor_msgs::ImageConstPtr &msg_left,
              Helpers &helper) {

    try {
        cv::Mat hsv_left, mask_left, masked_left, mask_left_eroded;

        cv_bridge::CvImagePtr cv_img_left = cv_bridge::toCvCopy(msg_left, "bgr8");

        cv::cvtColor(cv_img_left->image, hsv_left, cv::COLOR_BGR2HSV);

        cv::GaussianBlur(hsv_left, hsv_left, cv::Size(7, 7), 0, 0);

        cv::inRange(hsv_left, helper.white_lower, helper.white_upper, mask_left);
        cv::floodFill(mask_left, cv::Point(mask_left.cols / 2, 2), cv::Scalar(0));
        cv::erode(mask_left, mask_left_eroded, helper.erosion_element, cv::Point(-1, -1), helper.erosion_iter);

        cv_img_left->image.copyTo(masked_left, mask_left_eroded);

        // TODO: Remove more circular blobs. (the lane is an extremely elliptical blob)

        helper.pub_left.publish(
                cv_bridge::CvImage(cv_img_left->header, cv_img_left->encoding, masked_left).toImageMsg());

        std::vector<cv::Point> points;
        cv::findNonZero(mask_left_eroded, points);

        sensor_msgs::PointCloud2Ptr point_cloud = boost::make_shared<sensor_msgs::PointCloud2>();
        point_cloud->header.frame_id = helper.model_left.tfFrame().c_str();
        point_cloud->header.stamp = ros::Time::now();
        point_cloud->height = 1;
        point_cloud->width = points.size();
        point_cloud->is_bigendian = false;
        point_cloud->is_dense = false;
        sensor_msgs::PointCloud2Modifier pc_mod(*point_cloud);
        pc_mod.setPointCloud2FieldsByString(1, "xyz");
        sensor_msgs::PointCloud2Iterator<float> x(*point_cloud, "x"), y(*point_cloud, "y"), z(*point_cloud, "z");


        // TODO : Deal with the robot being tilted !!!
        for (const auto &point : points) {
            cv::Point3d ray = helper.model_left.projectPixelTo3dRay(point);
            cv::Point3d correct_ray(ray.z, -ray.x, -ray.y);

            correct_ray *= -helper.height / correct_ray.z;
            *x = correct_ray.x;
            *y = correct_ray.y;
            *z = correct_ray.z;


            ++x;
            ++y;
            ++z;
        }

        helper.pub_point_cloud.publish(point_cloud);


    } catch (std::exception &e) {
        ROS_ERROR("Callback failed: %s", e.what());
    }
}


void dynamic_reconfigure_callback(const igvc_bot::LanesConfig &config, const uint32_t &level, Helpers &helpers) {
    ROS_INFO("Reconfiguring...");
    if (level & 1u << 0u) {
        ROS_INFO("Reconfiguring lower level.");
        helpers.white_lower = cv::Scalar(config.h_lower, config.s_lower, config.v_lower);
    }

    if (level & 1u << 1u) {
        ROS_INFO("Reconfiguring upper level.");
        helpers.white_upper = cv::Scalar(config.h_upper, config.s_upper, config.v_upper);
    }

    if (level & 1u << 2u) {
        ROS_INFO("Reconfiguring erosion kernel.");
        helpers.erosion_size = config.erosion_size;
        helpers.erosion_iter = config.erosion_iter;
        helpers.erosion_element = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE,
                                                            cv::Size(2 * helpers.erosion_size + 1,
                                                                     2 * helpers.erosion_size + 1));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lane");
    ros::NodeHandle nh;

    ROS_INFO("Started");

    image_transport::ImageTransport imageTransport(nh);
    image_transport::SubscriberFilter right(imageTransport, topic_right, 4),
            left(imageTransport, topic_left, 4);

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>>
            filter(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>(5),
                   right, left);

    sensor_msgs::CameraInfo::ConstPtr camera_info_left = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
            topic_left_camera);

    Helpers helper{
            nh.advertise<sensor_msgs::PointCloud2>(points_topic, 5),
            imageTransport.advertise(pub_topic_right, 1),
            imageTransport.advertise(pub_topic_left, 1),

            // Params:
            (0, 0, 0), // Initialize from dynamic directly?
            (180, 40, 255),
            2,
            1,
            cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(2 * 3 + 1, 2 * 3 + 1))
    };

    helper.model_left.fromCameraInfo(camera_info_left);

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

    dynamic_reconfigure::Server<igvc_bot::LanesConfig> server;
    dynamic_reconfigure::Server<igvc_bot::LanesConfig>::CallbackType dynamic_reconfigure_callback_function = boost::bind(
            &dynamic_reconfigure_callback, _1, _2, boost::ref(helper));
    server.setCallback(dynamic_reconfigure_callback_function);


    const message_filters::Connection conn = filter.registerCallback(
            boost::bind(&callback, _1, _2, boost::ref(helper)));

    ROS_INFO("Spinning...");

    ros::spin();
}