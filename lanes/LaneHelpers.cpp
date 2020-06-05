#include "igvc_bot/LaneHelpers.h"

LanePublisher::LanePublisher(const ros::Publisher &pub, const size_t &points_to_keep) : _pub(pub),
                                                                                        _num_to_keep(points_to_keep),
                                                                                        _lane(),
                                                                                        header(_lane.header) {}

void LanePublisher::publish() {
    _lane.points.resize(vertices.size());
    auto it_pts = _lane.points.begin();
    for (auto &it : vertices) {
        it_pts->x = it.first;
        it_pts->y = it.second;
        ++it_pts;
    }

    _pub.publish(_lane);
    _lane.offset += vertices.size() - _num_to_keep;
    vertices.erase(vertices.begin(), vertices.end() - _num_to_keep);
}


horiz_dist::horiz_dist(std::pair<double, double> cur, std::pair<double, double> prev) : first(cur.first),
                                                                                        second(cur.second),
                                                                                        a(first - prev.first),
                                                                                        b(second - prev.second) {
    double factor = std::sqrt(a * a + b * b);
    if (factor == 0) factor = 1;
    a /= factor;
    b /= factor;
}

double horiz_dist::operator()(std::pair<double, double> point) const {
    return (point.first - first) * a +
           (point.second - second) * b;
}


vert_dist::vert_dist(std::pair<double, double> cur, std::pair<double, double> prev) : first(cur.first),
                                                                                      second(cur.second),
                                                                                      b(first - prev.first),
                                                                                      a(second - prev.second) {
    double factor = std::sqrt(a * a + b * b);
    if (factor == 0) factor = 1;
    a /= factor;
    b /= factor;
}

// GETS ABSOLUTE VALUE
double vert_dist::operator()(std::pair<double, double> point) const {
    return std::abs((point.first - first) * a -
                    (point.second - second) * b);
}


PCPublisher::PCPublisher(const ros::Publisher &pub) : _pub(pub), header(nullptr) {

}

using pc_iter = sensor_msgs::PointCloud2Iterator<float>;

std::array<pc_iter, 3> PCPublisher::get_iter(size_t size) {
    pc->width = size;

    sensor_msgs::PointCloud2Modifier(*pc).setPointCloud2FieldsByString(1, "xyz");

    return {
            pc_iter(*pc, "x"),
            pc_iter(*pc, "y"),
            pc_iter(*pc, "z"),
    };
}

void PCPublisher::clear_cloud() {
    pc = boost::make_shared<sensor_msgs::PointCloud2>();
    header = &pc->header;

    pc->height = 1;
    pc->is_bigendian = false;
    pc->is_dense = false;
}

void PCPublisher::publish() {
    _pub.publish(pc);
}

cv::Scalar getCvScalar(std::vector<double> v) {
    v.resize(3);
    return {v[0], v[1], v[2]};
}