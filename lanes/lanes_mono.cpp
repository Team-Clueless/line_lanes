#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <igvc_bot/LanesConfig.h>

#include <vector>


// Objects that the callback needs. Initialized in main().
struct Helpers {
    ros::Publisher pub_point_cloud;
    image_transport::Publisher pub_masked;

    cv::Scalar white_lower, white_upper; // HSV range for color white.

    // For cv::erode
    uint8_t erosion_size, erosion_iter;
    cv::Mat erosion_element;

    // for cv::blur
    uint8_t blur_size;

    bool publish_masked;

    // For projecting the image onto the ground.
    image_geometry::PinholeCameraModel cameraModel;
    float height;
};


const char *topic_image = "image_rect_color";
const char *topic_camera_info = "camera_info";
const char *topic_masked = "image_masked";
const char *topic_pointcloud2 = "points2";

const char *ground_frame = "ground";


void callback(const sensor_msgs::ImageConstPtr &msg_left,
              Helpers &helper) {

    try {
        cv::Mat hsv, blur, raw_mask, eroded_mask, masked;

        cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvCopy(msg_left, "");


        cv::cvtColor(cv_img->image, hsv, cv::COLOR_BGR2HSV);
        cv::GaussianBlur(hsv, blur, cv::Size(helper.blur_size, helper.blur_size), 0, 0);

        // Get white pixels
        cv::inRange(blur, helper.white_lower, helper.white_upper, raw_mask);

        // Flood Fill from the top of the mask to remove the sky in gazebo.
        cv::floodFill(raw_mask, cv::Point(raw_mask.cols / 2, 2), cv::Scalar(0));

        cv::erode(raw_mask, eroded_mask, helper.erosion_element, cv::Point(-1, -1), helper.erosion_iter);

        // TODO: Make sure we arent detecting any weird blobs and only the lane.

        cv_img->image.copyTo(masked, eroded_mask);


        if (helper.publish_masked)
            helper.pub_masked.publish(
                    cv_bridge::CvImage(cv_img->header, cv_img->encoding, masked).toImageMsg());


        std::vector<cv::Point> points; // All the points which is detected as part of the lane.
        cv::findNonZero(eroded_mask, points);


        // Initialize the point cloud:
        // See: https://answers.ros.org/question/212383/transformpointcloud-without-pcl/
        sensor_msgs::PointCloud2Ptr point_cloud = boost::make_shared<sensor_msgs::PointCloud2>();
        point_cloud->header.frame_id = helper.cameraModel.tfFrame();
        point_cloud->header.stamp = ros::Time::now();
        point_cloud->height = 1;
        point_cloud->width = points.size();
        point_cloud->is_bigendian = false;
        point_cloud->is_dense = false;

        sensor_msgs::PointCloud2Modifier pc_mod(*point_cloud);
        pc_mod.setPointCloud2FieldsByString(1, "xyz"); // Only want to publish spatial data.

        sensor_msgs::PointCloud2Iterator<float> x(*point_cloud, "x"), y(*point_cloud, "y"), z(*point_cloud, "z");
        for (const auto &point : points) {
            // ___________ Ray is a vector that points from the camera to the pixel: __________
            // Its calculation is pretty simple but is easier to use the image_geometry package.
            cv::Point3d ray = helper.cameraModel.projectPixelTo3dRay(point);
            /* ^ Basically:
             * cv::Point3d ray;
             * ray.x = (uv_rect.x - cx() - Tx()) / fx();
             * ray.y = (uv_rect.y - cy() - Ty()) / fy();
             * ray.z = 1.0;
             * f is focal length
             *
             */

            /* Note: the ray is not in the same frame as the tf_frame: camera_left
             * Ray frame:
             * x points to the right of the image, y points down, z points inward
             *
             * camera_left frame:
             * x points in the image and y points up
             *
             * If you look at the camera from above:
             *
             * ray: x <---(x) y  camera_left: z (.)---> y
             *             |                     |
             *             |                     |
             *             z                     x
             *
             * What we do is basically scale the ray such that the end touches the ground (we know the lane points are actually on the ground.
             * Then we add those co-ords to the point cloud.
             * ___________ We are basically checking the coords where the ray intersects the ground ________
             */

            if (!ray.y) continue; // For divide by zero (ray.y != 0)

            // Scaling and changing to camera_left frame:
            ray *= helper.height / ray.y;
            *x = ray.z;
            *y = -ray.x;
            *z = -ray.y; // Basically ground_height *z = helper.height

            // Go to next point in pointcloud.
            ++x;
            ++y;
            ++z;
        }

        helper.pub_point_cloud.publish(point_cloud);

    } catch (std::exception &e) {
        ROS_ERROR("Callback failed: %s", e.what());
    }
}

// This allows us to change params of the node while it is running: see cfg/lanes.cfg.
// Try running `rosrun rqt_reconfigure rqt_reconfigure` while node is running.
// This also auto loads any params initially set in the param server.
// The the ros page for 'dynamic_reconfigure'
//
void dynamic_reconfigure_callback(const igvc_bot::LanesConfig &config, const uint32_t &level, Helpers &helpers) {
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

    if (level & 1u << 3u) {
        ROS_INFO("Reconfiguring masked publishing.");
        helpers.publish_masked = config.publish_masked;
        // Deconstruct the helpers.pub_masked
    }

    if (level & 1u << 4u) {
        ROS_INFO("Reconfiguring blur size.");
        helpers.blur_size = 2 * config.blur_size + 1;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lane");
    ros::NodeHandle nh;

    ROS_INFO("Started");

    // For receiving and publishing the images in an easy way.
    image_transport::ImageTransport imageTransport(nh);

    Helpers helper{
            nh.advertise<sensor_msgs::PointCloud2>(topic_pointcloud2, 5),
            imageTransport.advertise(topic_masked, 1),

            (0, 0, 0),
            (180, 40, 255),

            2,
            1,
            cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(2 * 3 + 1, 2 * 3 + 1)),

            7,

            false
    };


    sensor_msgs::CameraInfo::ConstPtr camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
            topic_camera_info);
    helper.cameraModel.fromCameraInfo(camera_info);


    { // For getting the height.
        tf::TransformListener listener;
        while (true) {
            try {
                tf::StampedTransform transform;
                listener.lookupTransform(ground_frame, helper.cameraModel.tfFrame(), ros::Time(0), transform);
                helper.height = transform.getOrigin().z();
                break;
            } catch (const std::exception &e) {
                ROS_WARN("Transform not found, sleeping.");
                ros::Duration(1).sleep();
            }
        }
    }

    // For the dynamic parameter reconfigeration. see the function dynamic_reconfigure_callback
    dynamic_reconfigure::Server<igvc_bot::LanesConfig> server;
    dynamic_reconfigure::Server<igvc_bot::LanesConfig>::CallbackType dynamic_reconfigure_callback_function = boost::bind(
            &dynamic_reconfigure_callback, _1, _2, boost::ref(helper));
    server.setCallback(dynamic_reconfigure_callback_function);


    // Adds the callback
    image_transport::Subscriber sub = imageTransport.subscribe(topic_image, 2,
                                                               boost::bind(&callback, _1, boost::ref(helper)));


    ROS_INFO("Spinning...");
    ros::spin();
}