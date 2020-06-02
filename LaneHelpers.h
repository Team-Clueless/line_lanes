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

class vert_dist {
    // For a line passing through prev and cur.

    // operator() gives distance of point from line.

    const double first, second;
    double a, b;
public:
    vert_dist(std::pair<double, double> cur, std::pair<double, double> prev);

    double operator()(std::pair<double, double> point) const;
};


#endif //IGVC_BOT_LANEHELPERS_H
