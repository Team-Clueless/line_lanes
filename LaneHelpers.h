#ifndef IGVC_BOT_LANEHELPERS_H
#define IGVC_BOT_LANEHELPERS_H

#include <ros/ros.h>
#include <igvc_bot/Lane.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class PCPublisher {
    ros::Publisher _pub;
    sensor_msgs::PointCloud2Ptr pc;

public:
    sensor_msgs::PointCloud2::_header_type *header{};

    PCPublisher(const ros::Publisher &pub);

    void clear_cloud();

    std::array<sensor_msgs::PointCloud2Iterator<float>, 3> get_iter(size_t size);

    void publish();
};

class LanePublisher {
    ros::Publisher _pub;
    igvc_bot::Lane _lane;
    const size_t _num_to_keep;

public:
    igvc_bot::Lane::_header_type &header;
    std::vector<std::pair<double, double> > vertices;

    LanePublisher(const ros::Publisher &pub, const size_t &points_to_keep);

    void publish();
};

class horiz_dist {
    // For a line at cur, perpendicular to prev -> cur.

    // operator() gives distance of point from line.
    // The points ont the other side of prev give positive values.

    const double first, second;
    double a, b;
public:
    horiz_dist(std::pair<double, double> cur, std::pair<double, double> prev);

    double operator()(std::pair<double, double> point) const;
};

// GETS ABSOLUTE VALUE
class vert_dist {
    // For a line passing through prev and cur.

    // operator() gives distance of point from line.

    const double first, second;
    double a, b;
public:
    vert_dist(std::pair<double, double> cur, std::pair<double, double> prev);

    // GETS ABSOLUTE VALUE
    double operator()(std::pair<double, double> point) const;
};

#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>

struct Helpers {
    struct Publishers {
        image_transport::Publisher masked;
        PCPublisher pc;
        LanePublisher lane;
    } pub;

    struct CVParams {
        cv::Scalar white_lower, white_upper, barrel_lower, barrel_upper; // HSV range for color white.

        double rect_frac;

        uint8_t erosion_size, erosion_iter;
        cv::Mat erosion_element;

        // for cv::blur
        uint8_t blur_size;
    } cv;

    bool publish_masked;

    struct LanesParams {
        size_t max_points, stride;
        double epsilon_dist, max_vert_dist;
        double max_horiz_dist, min_new_dist, max_new_dist;
        double cos_max_new_angle, cos_min_angle_reverse_pt;
    } lanes;

    // For projecting the image onto the ground.
    image_geometry::PinholeCameraModel cameraModel;
    tf::TransformListener listener;
};


#endif //IGVC_BOT_LANEHELPERS_H
