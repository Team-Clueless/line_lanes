#ifndef IGVC_BOT_LANESLAYER_H
#define IGVC_BOT_LANESLAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <vector>
#include <mutex>

#include <igvc_bot/Lane.h>

// Costmap Prohibition layer:
// ref https://github.com/rst-tu-dortmund/costmap_prohibition_layer/blob/kinetic-devel/src/costmap_prohibition_layer.cpp
// ref https://github.com/rst-tu-dortmund/costmap_prohibition_layer/blob/kinetic-devel/include/costmap_prohibition_layer/costmap_prohibition_layer.h

namespace lanes_layer {
    class LanesLayer : public costmap_2d::Layer {

    public:
        LanesLayer();

        ~LanesLayer() override;

        void onInitialize() override;

        // Here we have to specify the region that is to be updated.
        void
        updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x,
                     double *max_y) override;

        // Actually write the data to the costmap
        void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override;

        void callback(const igvc_bot::Lane::ConstPtr &msg);

    private:
        ros::Subscriber _sub;
        std::mutex mutex; // stops updateCostmap and callback from running at the same time
        size_t update_from{};
        std::vector<std::pair<double, double> > vertices;
        std::vector<std::vector<std::pair<double, double> > > vertices_to_remove;

        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

        double _min_x{}, _min_y{}, _max_x{}, _max_y{};


        // Draws lines between pairs of vertices from start to end.
        static void write_segments(costmap_2d::Costmap2D &master_grid,
                                   const std::vector<std::pair<double, double>>::iterator &start,
                                   const std::vector<std::pair<double, double>>::iterator &end, unsigned char cost);

        // Draws a line b/w (x0,y0) -> (x1,y1) where x,y given in costmap co-ords
        // copied from costmap_prohibition_layer
        static void write_line(costmap_2d::Costmap2D &costmap, int x0, int y0, int x1, int y1, unsigned char cost);

    };
}


#endif //IGVC_BOT_LANESLAYER_H
