#include "igvc_bot/LanesLayer.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(lanes_layer::LanesLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace lanes_layer {

    LanesLayer::LanesLayer() : dsrv_(nullptr) {}

    LanesLayer::~LanesLayer() {
        delete dsrv_;
        _sub.shutdown();
    }

    void LanesLayer::onInitialize() {
        ros::NodeHandle nh("~/" + name_);
        current_ = true;

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                &LanesLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        ROS_INFO("Initialisin layer...");

        std::string topic;
        if (!nh.getParam("topic", topic))
            throw ros::Exception("Missing topic");
        _sub = nh.subscribe(topic, 10, &LanesLayer::callback, this);

        ROS_INFO("initialized layer...");
    }


    void LanesLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level) {
        enabled_ = config.enabled;
    }

    void LanesLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double *min_x, double *min_y, double *max_x, double *max_y) {
        if (!enabled_)
            return;

        std::lock_guard<std::mutex> lock(mutex);

        *min_x = std::min(*min_x, _min_x);
        *min_y = std::min(*min_y, _min_y);
        *max_x = std::max(*max_x, _max_x);
        *max_y = std::max(*max_y, _max_y);

    }

    void LanesLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i,
                                 int max_j) {
        if (!enabled_)
            return;

        std::lock_guard<std::mutex> guard(mutex);

        //write_segments(master_grid, vertices_to_remove.begin(), vertices_to_remove.end(), costmap_2d::FREE_SPACE);
        if (vertices.empty())
            return;

        const auto start = vertices.begin() + (update_from == 0 ? 0 : update_from - 1);
        const auto end = vertices.end();
        write_segments(master_grid, start, end, costmap_2d::LETHAL_OBSTACLE);
    }

    void LanesLayer::write_segments(costmap_2d::Costmap2D &master_grid,
                                    const std::vector<std::pair<double, double>>::iterator &start,
                                    const std::vector<std::pair<double, double>>::iterator &end,
                                    unsigned char cost) {
        int px, py, cx, cy;
        master_grid.worldToMapNoBounds(start->first, start->second, px, py);

        for (auto it = start; it != end; ++it) {
            master_grid.worldToMapNoBounds(it->first, it->second, cx, cy);
            raytrace(master_grid, px, py, cx, cy, cost);
            px = cx;
            py = cy;
        }
    }


    void
    LanesLayer::raytrace(costmap_2d::Costmap2D &costmap, int x0, int y0, int x1, int y1, unsigned char cost) {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int x = x0, y = y0;
        int n = 1 + dx + dy;
        int x_inc = (x1 > x0) ? 1 : -1;
        int y_inc = (y1 > y0) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        for (; n > 0; --n) {
            costmap.setCost(x, y, cost);

            if (error > 0) {
                x += x_inc;
                error -= dy;
            } else {
                y += y_inc;
                error += dx;
            }
        }
    }


    void LanesLayer::callback(const igvc_bot::Lane::ConstPtr &msg) {
        std::lock_guard<std::mutex> guard(mutex);

        ROS_INFO("got msg:");

        // To support python like -ive indices.
        size_t msg_offset = msg->offset >= 0 ? msg->offset : vertices.size() + msg->offset;

        if (msg->offset > vertices.size()) {
            std::pair<double, double> last_point = vertices.back();
            vertices.resize(msg_offset, last_point);
        } else {
            /*vertices_to_remove.clear();
            vertices_to_remove.reserve(vertices.size() - msg_offset);
            vertices_to_remove.insert(vertices_to_remove.begin(),
                                      vertices.begin() + (msg_offset == 0 ? 0 : msg_offset - 1), vertices.end());*/
            vertices.erase(vertices.begin() + msg_offset, vertices.end());
        }

        vertices.reserve(vertices.size() + msg->points.size());

        for (auto &it : msg->points) {
            vertices.emplace_back(it.x, it.y);

            _min_x = std::min(it.x, _min_x);
            _min_y = std::min(it.y, _min_y);
            _max_x = std::max(it.x, _max_x);
            _max_y = std::max(it.y, _max_y);
        }
        update_from = std::min(update_from, msg_offset);

        ROS_INFO("Finished msg");

    }
} // end namespace lanes_layer