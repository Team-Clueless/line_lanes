#include "igvc_bot/LanesLayer.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(lanes_layer::LanesLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace lanes_layer {

    LanesLayer::LanesLayer() : dsrv_(0) {}

    LanesLayer::~LanesLayer() {
        delete dsrv_;
    }

    void LanesLayer::onInitialize() {
        ros::NodeHandle nh("~/" + name_);
        current_ = true;

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                &LanesLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }


    void LanesLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level) {
        enabled_ = config.enabled;
    }

    void LanesLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                  double *min_y, double *max_x, double *max_y) {
        if (!enabled_)
            return;

    }

    void LanesLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i,
                                 int max_j) {
        if (!enabled_)
            return;
    }

    void LanesLayer::callback(const igvc_bot::Lane &msg) {
        if (msg.offset + msg.points.size() <= points_offset) return; // TODO: raise error
        if (msg.offset <= points_offset) return; // TODO: implement

        vertices.erase(vertices.begin() + msg.offset - points_offset, vertices.end());
        vertices.reserve(msg.offset - points_offset + msg.points.size());
        for (auto &it : msg.points)
            vertices.emplace_back(it.x, it.y);
        update_from = msg.offset - points_offset;

    }

} // end namespace