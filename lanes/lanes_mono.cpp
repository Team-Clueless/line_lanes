#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <igvc_bot/LanesConfig.h>

#include <vector>
#include <functional>
#include <nav_msgs/Path.h>

// Objects that the callback needs. Initialized in main().
class Helpers {
public:
    image_transport::Publisher pub_masked;
    ros::Publisher path_pub;

    cv::Scalar white_lower, white_upper; // HSV range for color white.

    double rect_frac;
    // For cv::erode
    uint8_t erosion_size, erosion_iter;
    cv::Mat erosion_element;

    // for cv::blur
    uint8_t blur_size;

    bool publish_masked;
    std::vector<std::pair<double, double> > path;


    // For projecting the image onto the ground.
    image_geometry::PinholeCameraModel cameraModel;
    tf::TransformListener listener;
};


const char *topic_image = "image_rect_color";
const char *topic_camera_info = "camera_info";
const char *topic_masked = "image_masked";

// In rviz map/odom seems to be a true horizontal plane located at ground level.
const char *ground_frame = "odom";

const char *path_topic = "path";

const size_t num_mutable_points = 2;
const size_t num_sections = num_mutable_points + 1;


/*double section_func(const std::pair<double, double> cur, const std::pair<double, double> prev,
                    const std::pair<double, double> point) {
    return (point.first - cur.first) * (cur.first - prev.first) +
           (point.second - cur.second) * (cur.second - prev.second);
}*/

class function_type {
    const double first, second;
    double a, b;
public:
    function_type(const std::pair<double, double> cur,
                  const std::pair<double, double> prev) : first(cur.first),
                                                          second(cur.second),
                                                          a(first - prev.first),
                                                          b(second - prev.second) {
        double factor = std::sqrt(a * a + b * b);
        if (factor == 0) factor = 1;
        a /= factor;
        b /= factor;
        ROS_INFO("a,b => %lf, %lf", a, b);
    }

    double operator()(const std::pair<double, double> point) const {
        return (point.first - first) * a +
               (point.second - second) * b;
    }
};


void callback(const sensor_msgs::ImageConstPtr &msg_left,
              Helpers &helper) {

    try {
        // Get transform asap for something from the right time without messing around with tf.
        tf::StampedTransform transform;
        helper.listener.lookupTransform(ground_frame, helper.cameraModel.tfFrame(), ros::Time(0), transform);

        std::vector<cv::Point> points; // All the points which is detected as part of the lane.

        // OpenCV Stuff Which fills up points.
        {
            cv::Mat hsv, blur, raw_mask, eroded_mask, masked;

            cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvCopy(msg_left, "bgr8");
            // TODO: Should we just downscale image?


            cv::cvtColor(cv_img->image, hsv, cv::COLOR_BGR2HSV);
            cv::GaussianBlur(hsv, blur, cv::Size(helper.blur_size, helper.blur_size), 0, 0);

            // Get white pixels
            cv::inRange(blur, helper.white_lower, helper.white_upper, raw_mask);

            // Flood Fill from the top of the mask to remove the sky in gazebo.
            cv::floodFill(raw_mask, cv::Point(raw_mask.cols / 2, 2), cv::Scalar(0));

            // Errors in projection increase as we approach the halfway point of the image:
            // Apply a mask to remove top 60%
            raw_mask(cv::Rect(0, 0, raw_mask.cols, (int) (raw_mask.rows * helper.rect_frac))) = 0;

            cv::erode(raw_mask, eroded_mask, helper.erosion_element, cv::Point(-1, -1), helper.erosion_iter);

            cv_img->image.copyTo(masked, eroded_mask);


            if (helper.publish_masked)
                helper.pub_masked.publish(
                        cv_bridge::CvImage(cv_img->header, cv_img->encoding, masked).toImageMsg());


            cv::findNonZero(eroded_mask, points);
        }
        ROS_INFO("OpenCV detected %lu points", points.size());

        // Reduce the num points
        const static size_t stride = 50;

        if (points.size() > 100000 || points.size() <= stride) // Quit on bad num points.
            return;

        // Downscales the number of points
        const auto point_strided_end = points.end() - (points.size() % stride);

        ROS_INFO("Only using %lu points.", (size_t) points.size() / stride);

        // Change the transform to a more useful form.
        tf::Quaternion trans_rot = transform.getRotation();
        cv::Vec3d trans_vec{trans_rot.x(), trans_rot.y(), trans_rot.z()};
        double trans_sca = trans_rot.w();

        auto &vertices = helper.path; // The current path.
        ROS_INFO("Have %lu existing vertices", vertices.size());

        // Stores points oredered and section_wise.
        std::array<std::vector<std::pair<double, double> >, num_sections> points_vectors;

        // Section wise and sorted, stores the distance of each point from the section line.
        std::array<std::vector<double>, num_sections> dist_vectors;

        // Generated by ide from using auto on the bind. // Valgrind error, screw it use a class
        /*using function_type = std::_Bind_helper<false, double (&)(const std::pair<double, double>,
                                                                  const std::pair<double, double>,
                                                                  const std::pair<double, double>),
                std::pair<double, double> &, std::pair<double, double> &, const std::_Placeholder<1> &>::type;*/

        std::vector<function_type> section_funcs;

        ROS_INFO("Creating the section_funcs");
        for (int i = 0; i < num_sections; i++) {
            /*section_funcs.push_back(std::bind(section_func, *(vertices.end() - (1 + i)), *(vertices.end() - (2 + i)),
                                              std::placeholders::_1));*/
            section_funcs.emplace_back(*(vertices.end() - (1 + i)), *(vertices.end() - (2 + i)));
        }
        ROS_INFO("Created %lu section funcs", section_funcs.size());

        const double max_horiz_dist = 4, max_vert_dist = 2, epsilon_dist = 0.4, min_new_dist = 0.4, max_new_dist = 4;

        //for (const auto &point : points) { // Normal for:range loop requires a messy custom container class for stride.
        auto point_iter = points.begin();
        for (auto &camera_point = *point_iter; point_iter != point_strided_end;
             point_iter += stride, camera_point = *point_iter) {
            // ___________ Ray is a vector that points from the camera to the pixel: __________
            // Its calculation is pretty simple but is easier to use the image_geometry package.
            /* Basically:
             * cv::Point3d ray_cameraModel_frame;
             * ray_cameraModel_frame.x = (uv_rect.x - cx() - Tx()) / fx();
             * ray_cameraModel_frame.y = (uv_rect.y - cy() - Ty()) / fy();
             * ray_cameraModel_frame.z = 1.0;
             * f is focal length
             */
            cv::Point3d ray_cameraModel_frame = helper.cameraModel.projectPixelTo3dRay(camera_point);
            //^ 3d d=>double.

            /* Note: the ray_cameraModel_frame is not in the same frame as the tf_frame: camera_left
             * Ray frame:
             * x points to the right of the image, y points down, z points inward
             *
             * camera_left frame:
             * x points in the image and y points up
             *
             * If you look at the camera from above:
             *
             * ray_cameraModel_frame: x <---(x) y  camera_left: z (.)---> y
             *             |                     |
             *             |                     |
             *             z                     x
             *
             * What we do is basically scale the ray_cameraModel_frame such that the end touches the ground (we know the lane points are actually on the ground.
             * Then we add those co-ords to the point cloud.
             * ___________ We are basically checking the co-ords where the ray_cameraModel_frame intersects the ground ________
             */
            cv::Vec3d ray{ray_cameraModel_frame.z, -ray_cameraModel_frame.x, -ray_cameraModel_frame.y};

            // Rotate ray by using the transform.
            // Kinda black magic on v_p = q * v * q'
            // https://gamedev.stackexchange.com/a/50545/90578
            cv::Vec3d ray_p = 2.0 * trans_vec.dot(ray) * trans_vec
                              + (trans_sca * trans_sca - trans_vec.dot(trans_vec)) * ray
                              + 2.0f * trans_sca * trans_vec.cross(ray);

            if (ray_p[2] == 0) // Horizontal rays. is it > or < for rays facing above the horizon TODO
                continue;

            // Scale factor for ray so it touches the ground
            const double scale = transform.getOrigin().z() / ray_p[2];
            std::pair<double, double> point = std::make_pair(transform.getOrigin().x() - ray_p[0] * scale,
                                                             transform.getOrigin().y() - ray_p[1] * scale);

            ROS_INFO("Sectioning point");
            double dist;
            for (int i = 0; i < num_sections; i++) {
                dist = section_funcs[i](point);
                if (dist >= 0) {
                    if (dist > max_horiz_dist) { // Point too far, ignore.
                        ROS_INFO("Point too far!");
                        break;
                    }
                    const auto it = std::lower_bound(dist_vectors[i].begin(), dist_vectors[i].end(), dist);
                    points_vectors[i].insert(points_vectors[i].begin() + (it - dist_vectors[i].begin()), point);
                    dist_vectors[i].insert(it, dist);
                    ROS_INFO("Point added. %i : %f", i, dist);
                    break;
                }
                ROS_INFO("Not Point added. %i : %f", i, dist);
            }
        }
        points.clear(); // Dun need points anymore, usefuls are copied to points_vectors.

        ROS_INFO("Done adding points to vectors.");
        for (const auto &vec : points_vectors) {
            ROS_INFO("%lu", vec.size());
        }


        bool recent = false;
        {
            if (!points_vectors[0].empty()) {
                auto &pt = vertices.back();
                auto &pt_new = points_vectors[0].back();
                double dist = std::sqrt((pt_new.first - pt.first) * (pt_new.first - pt.first) +
                                        (pt_new.second - pt.second) * (pt_new.second - pt.second));
                if (min_new_dist < dist && dist < max_new_dist) {
                    recent = true;
                    vertices.push_back(pt_new);

                    ROS_INFO("Added a recent point");
                }

                ROS_INFO("Dont need a recent point");
            }
            ROS_INFO("Checked back.");
        }

        for (size_t i = num_sections - (recent ? 1u : 2u);
             i >= 0 && i < num_sections; i--) { // Size_t is unsigned!! Thus never stop
            ROS_INFO("Starting %lu section back", i);

            auto &points_cur = points_vectors[i + (recent ? 0 : 1)];

            if (points_cur.empty()) {
                ROS_INFO("No changes req");
                continue;
            }
            ROS_INFO("Working with %lu", points_cur.size());


            const size_t start_index = (vertices.end() - (1 + i + 1)) - vertices.begin(); // the first point in section

            std::vector<double> dists; // Perpendicular distances
            dists.reserve(points_cur.size());

            auto start = *(vertices.begin() + start_index);
            auto end = *(vertices.begin() + start_index + 1);

            ROS_INFO("Starting distances check: ");
            const double to_dist = std::sqrt((start.first - end.first) * (start.first - end.first) +
                                             (end.second - start.second) * (end.second - start.second));

            auto dist = [&start, &end](const std::pair<double, double> &point) {
                return std::abs((start.first - point.first) * (end.second - start.second) -
                                (start.first - end.first) * (point.second - end.second));
            };

            double max_dist = 0, cur_dist;
            std::pair<double, double> &max_pt = points_cur.front();
            for (const auto &pt : points_cur) {
                cur_dist = dist(pt);
                if (cur_dist / to_dist > max_vert_dist)
                    continue;
                if (cur_dist > max_dist) {
                    max_dist = cur_dist;
                    max_pt = pt;
                }
            }

            if (max_dist / to_dist > epsilon_dist) {
                vertices.insert(vertices.begin() + start_index + 1, max_pt);
                // Now recursively.
            }
            // if (i == 0) break;
        }

        ROS_INFO("PAth has %lu vertices/", vertices.size());

        nav_msgs::Path p;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = ground_frame;

        for (const auto &vertex : vertices) {
            p.poses.emplace_back();
            p.poses.back().header = p.header;
            p.poses.back().pose.orientation.w = 1;
            p.poses.back().pose.position.x = vertex.first;
            p.poses.back().pose.position.y = vertex.second;
        }

        helper.path_pub.publish(p);
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

    if (level & 1u << 5u) {
        ROS_INFO("Reconfiguring rect mask");
        helpers.rect_frac = config.upper_mask_percent / 100.0;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lane");
    ros::NodeHandle nh;

    ROS_INFO("Started");

    // For receiving and publishing the images in an easy way.
    image_transport::ImageTransport imageTransport(nh);

    Helpers helper{
            imageTransport.advertise(topic_masked, 1),
            nh.advertise<nav_msgs::Path>(path_topic, 1),

            (0, 0, 0),
            (180, 40, 255),

            0.6,

            2,
            1,
            cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(2 * 3 + 1, 2 * 3 + 1)),

            7,

            false,
    };


    sensor_msgs::CameraInfo::ConstPtr camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
            topic_camera_info);
    helper.cameraModel.fromCameraInfo(camera_info);

    {
        std::vector<double> initial_x, initial_y;

        if (!nh.getParam("lanes_mono/initial_path/x", initial_x))
            initial_x = {0};
        if (!nh.getParam("lanes_mono/initial_path/y", initial_y))
            initial_y = {0};

        initial_x.resize(num_mutable_points + 1); //std::min(initial_x.size(), initial_y.size()));
        initial_y.resize(num_mutable_points + 1); //initial_x.size());

        std::transform(initial_x.begin(), initial_x.end(), initial_y.begin(), std::back_inserter(helper.path),
                       [](const double &x, const double &y) { return std::make_pair(x, y); });
    }

    // For the dynamic parameter reconfiguration. see the function dynamic_reconfigure_callback
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