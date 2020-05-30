//
// Created by suraj on 5/30/20.
//

#ifndef IGVC_BOT_LANESLAYER_H
#define IGVC_BOT_LANESLAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

// ref https://github.com/rst-tu-dortmund/costmap_prohibition_layer/blob/kinetic-devel/src/costmap_prohibition_layer.cpp
// ref https://github.com/rst-tu-dortmund/costmap_prohibition_layer/blob/kinetic-devel/include/costmap_prohibition_layer/costmap_prohibition_layer.h

namespace lanes_layer {
    class LanesLayer : public costmap_2d::Layer {

    public:
        LanesLayer();

        virtual ~LanesLayer();

        virtual void onInitialize();

        virtual void
        updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x,
                     double *max_y);

        virtual void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

    private:
        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

    };
}


#endif //IGVC_BOT_LANESLAYER_H
