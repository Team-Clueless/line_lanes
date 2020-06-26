#ifndef LINE_LANES_LANEHELPERS_H
#define LINE_LANES_LANEHELPERS_H

#include <ros/ros.h>
#include <line_lanes/Lane.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <mutex>
#include <utility>
#include <opencv/cv.h>

template<typename T>
T getParam(const std::string &key, T default_value) {
    T val;
    if (ros::param::get(key, val))
        return val;
    return default_value;
}

cv::Scalar getCvScalar(std::vector<double> v);

/*
template int getParam<int>;
template double getParam<double>;
*/

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
    line_lanes::Lane _lane;
    const size_t _num_to_keep;

public:
    line_lanes::Lane::_header_type &header;
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

        uint8_t blur_size;

        bool publish_masked;
    } cv{
            getCvScalar(getParam<std::vector<double> >("cv/lane_lower", {0, 0, 0})),
            getCvScalar(getParam<std::vector<double> >("cv/lane_upper", {180, 40, 255})),

            getCvScalar(getParam<std::vector<double> >("cv/barrel_lower", {0, 250, 0})),
            getCvScalar(getParam<std::vector<double> >("cv/barrel_upper", {180, 255, 255})),

            (getParam("cv/upper_mask_percent", 60) % 100) / 100.0,

            (uint8_t) getParam("cv/erosion_size", 2),
            (uint8_t) getParam("cv/erosion_iter", 1),
            cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(2 * 3 + 1, 2 * 3 + 1)),

            (uint8_t) getParam("cv/blur_size", 7),
            getParam("cv/publish_masked", true)
    };

    struct LanesParams {
        size_t max_points, stride;
        double epsilon_dist, max_vert_dist;
        double max_horiz_dist, min_new_dist, max_new_dist;
        double cos_max_new_angle, cos_min_angle_reverse_pt;
    } lanes{
            (size_t) getParam("lanes/max_points", 10000),
            (size_t) getParam("lanes/stride", 20),
            getParam("lanes/epsilon_dist", 0.25),
            getParam("lanes/max_vert_dist", 0.74),
            getParam("lanes/max_horiz_dist", 4.5),
            getParam("lanes/min_new_dist", 0.5),
            getParam("lanes/max_new_dist", 4.0),
            getParam("lanes/cos_max_new_angle", 0.8),
            getParam("lanes/cos_min_angle_reverse_point", 0.2)
    };

    // For projecting the image onto the ground.
    image_geometry::PinholeCameraModel cameraModel;
    tf::TransformListener listener;

    bool dynamic_reconfigure{getParam("dynamic_reconfigure", false)};

    std::mutex mutex;

    enum Mode {
        MODIFYING, // Have some vertices
        SEARCHING  // Need to find vertices
    } mode{Mode::SEARCHING};

    Helpers(Publishers pubs) : pub(std::move(pubs)), mutex() {}
};

#endif //LINE_LANES_LANEHELPERS_H
