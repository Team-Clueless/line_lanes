#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <line_lanes/LanesConfig.h>

#include <line_lanes/LaneHelpers.h>

const char *topic_image = "image_rect_color";
const char *topic_camera_info = "camera_info";
const char *topic_masked = "image_masked";

// In rviz map/odom seems to be a true horizontal plane located at ground level.
const char *ground_frame = "odom";

const char *pc_topic = "points2";
const char *lane_topic = "lane/updates";

const size_t num_mutable_points = 2;
const size_t num_sections = num_mutable_points + 1;

void process_image(const cv_bridge::CvImageConstPtr &cv_img, std::vector<cv::Point> &points,
                   const Helpers::CVParams &params, image_transport::Publisher *cv_pub,
                   std::vector<cv::Point> *hough = nullptr);

double dist_sq(const std::pair<double, double> &p1, const std::pair<double, double> &p2) {
    return std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2);
}

void callback(const sensor_msgs::ImageConstPtr &msg,
              Helpers &helper) {

    try {
        std::lock_guard<std::mutex> lock(helper.mutex);

        // Get transform asap for something from the right time without messing around with tf.
        tf::StampedTransform transform;
        helper.listener.lookupTransform(ground_frame, helper.cameraModel.tfFrame(), ros::Time(0),
                                        transform);

        // Change the transform to a more useful form.
        tf::Quaternion trans_rot = transform.getRotation();
        cv::Vec3d trans_vector{trans_rot.x(), trans_rot.y(), trans_rot.z()};
        double trans_scalar = trans_rot.w();


        // Projects a camera pixel to ground [ground_frame.z = 0]
        auto pixel_to_point = [&helper, &transform, &trans_rot, &trans_vector, &trans_scalar](
                const cv::Point &camera_point) -> std::pair<double, double> {
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
            cv::Vec3d ray_p = 2.0 * trans_vector.dot(ray) * trans_vector
                              + (trans_scalar * trans_scalar - trans_vector.dot(trans_vector)) * ray
                              + 2.0f * trans_scalar * trans_vector.cross(ray);

            /*if (ray_p[2] == 0) // Horizontal rays. is it > or < for rays facing above the horizon TODO
                continue;*/

            // Scale factor for ray so it touches the ground
            const double scale = transform.getOrigin().z() / ray_p[2];

            // Get endpoints of ray
            return std::make_pair(transform.getOrigin().x() - ray_p[0] * scale,
                                  transform.getOrigin().y() - ray_p[1] * scale);
        };

        auto &vertices = helper.pub.lane.vertices;
        bool path_updated = false; // Was new point added


        static const double max_camera_dist_sq = 8 * 8;
        if (helper.mode == helper.MODIFYING &&
            dist_sq(vertices.back(), {transform.getOrigin().x(), transform.getOrigin().y()}) > max_camera_dist_sq) {

            helper.mode = helper.SEARCHING;
            ROS_ERROR("Cant find lane, switching to searching mode.");
        }

        std::vector<cv::Point> points; // All the points which is detected as part of the lane

        if (helper.mode == helper.SEARCHING) {
            std::vector<cv::Point> endpixels;
            process_image(cv_bridge::toCvCopy(msg, "bgr8"), points, helper.cv,
                          &helper.pub.masked, &endpixels);

            if (endpixels.size() == 2) { // Line Found
                // TODO: Check that this doesnt intersect with the other lane. Give actions to the costmap layers.
                helper.mode = helper.MODIFYING;

                auto p0 = pixel_to_point(endpixels[0]);
                auto p1 = pixel_to_point(endpixels[1]);

                const auto d0 = dist_sq(p0, vertices.back());
                const auto d1 = dist_sq(p1, vertices.back());
                vertices.push_back(d0 >= d1 ? p1 : p0);
                vertices.push_back(d0 >= d1 ? p0 : p1);

                path_updated = true;
                ROS_ERROR("Found lane from line of length %f", std::sqrt(dist_sq(p0, p1)));
            }
        } else {
            process_image(cv_bridge::toCvCopy(msg, "bgr8"), points, helper.cv, &helper.pub.masked);
        }


        auto &params = helper.lanes;

        if (points.size() > params.max_points || points.size() <= params.stride) // Quit on bad num points.
            return;

        // Reduce the num points by iterating through with a gap of params.stride
        const auto point_strided_end = points.end() - (points.size() % params.stride); // Last point

        // PC Publishing
        helper.pub.pc.clear_cloud();
        //auto[x, y, z] = helper.pub.pc.get_iter(points.size() / stride);
        auto _pc_iters = helper.pub.pc.get_iter(points.size() / params.stride);
        auto &x = _pc_iters[0], &y = _pc_iters[1], &z = _pc_iters[2];
        helper.pub.pc.header->frame_id = ground_frame;
        helper.pub.pc.header->stamp = ros::Time::now();


        // Stores points section_wise and ordered by increasing horizontal distance.
        std::array<std::vector<std::pair<double, double> >, num_sections> points_sectioned;

        // Stores the perpendicular distance of the point from the section line. This stays sorted.
        std::array<std::vector<double>, num_sections> horiz_dist_vectors;

        std::vector<horiz_dist> section_funcs; // Functions that give horizontal distance
        auto cur_vertex = vertices.end() - 1;
        for (int i = 0; i < num_sections; i++) {
            section_funcs.emplace_back(*(cur_vertex), *(cur_vertex - 1));
            --cur_vertex;
        }
        // Note:
        // vertices.end() is an iter right after the last element.
        // *(vertices.end() - 1) ==> Last element.. *(... - 2) ==> second last element

        /* params. :
         * Max_horizontal dist:
         * if horiz_dist(point) > max ==> Point is skipped
         * Sim for vertical dist
         *
         * epsilon_dist: if distance of point from existing segment > epsilon dist ===> Then add that as a new vertice.
         *
         * min/max new_dist: For a point to be added at the end of the existing path, it must satisfy the above conditions
         */

        //for (const auto &point : points) { // Normal for:range loop requires a messy custom container class for stride.

        for (auto point_iter = points.begin(); point_iter != point_strided_end; point_iter += params.stride) {

            auto point = pixel_to_point(*point_iter);

            // Add to point cloud
            *x = point.first;
            *y = point.second;
            *z = 0;
            ++x;
            ++y;
            ++z;

            // Sort them section wise
            if (helper.mode == helper.MODIFYING) {
                double horiz_dist;
                for (int i = 0; i < num_sections; i++) {
                    horiz_dist = section_funcs[i](point); // perpendicular dist of the  point from the end.
                    if (horiz_dist >= 0) { // i.e. point is to the right of the line i.e. in this section
                        if (horiz_dist > params.max_horiz_dist) { // Point too far, ignore.
                            break;
                        }
                        // Inserts point and dist to the respective vectors while soring by increasing dist..
                        const auto it = std::lower_bound(horiz_dist_vectors[i].begin(), horiz_dist_vectors[i].end(),
                                                         horiz_dist);

                        points_sectioned[i].insert(points_sectioned[i].begin() + (it - horiz_dist_vectors[i].begin()),
                                                   point);
                        horiz_dist_vectors[i].insert(it, horiz_dist);
                        break;
                    }
                }
            }
        }
        points.clear(); // Dont need points anymore, useful are copied to points_sectioned.
        // Instead of copying points to points_sectioned, store references there?

        helper.pub.pc.publish();
        if (helper.mode == helper.SEARCHING)
            return;

        /* "Heruistics"?
         *
         * 1. Points' perpendicular distance from segment / perpendicular and vertex is within a range.
         * 2. Points whose perpendicular distance from a segment is above a threshold are inserted into the path.
         *    These points must also have a horizontal_dist : vertical_dist ratio greater than a threshold
         * 3. If after addition of a new point, the two created segments form a sharp angle, the vertex after the
         *    inserted one is removed.
         * 4. New points are inserted at the end of the path if they lie within a certain distance and angle of the
         *    most recent segment.
         *
         */

        bool recent = false;

        // This checks whether we need to append a new point to the path
        if (!points_sectioned[0].empty()) {
            auto &pt = vertices.back();

            auto horiz_dist_it = horiz_dist_vectors[0].end() - 1; // Iterator to horizontal distances
            for (auto it = points_sectioned[0].end() - 1;
                 it != points_sectioned[0].begin(); --it, --horiz_dist_it) {
                // Loop through points and their horizontal distances

                auto &pt_new = *it;

                // TODO: Square the parameter instead of sqrt-ing the expr
                double dist = std::sqrt((pt_new.first - pt.first) * (pt_new.first - pt.first) +
                                        (pt_new.second - pt.second) * (pt_new.second - pt.second));

                // If dist comes in the right range, add it to the path.
                if (params.min_new_dist < dist) {
                    // TODO: Stop new points from being added backwards..
                    if (dist < params.max_new_dist && *horiz_dist_it > dist * params.cos_max_new_angle) {
                        vertices.push_back(pt_new);
                        recent = true;
                        path_updated = true;

                        ROS_INFO("Added a new point to path, now %lu vertices", vertices.size());
                        break;
                    }
                } else { break; }
            }
        }


        // For all the segments, see if any point has a perpendicular distance more than epsilon_dist
        for (size_t i = num_sections - (recent ? 1u : 2u); i >= 0 && i < num_sections; i--) {

            // Set of points in the current segment
            auto &current_points = points_sectioned[i + (recent ? 0 : 1)];

            if (current_points.empty()) // No points in this segment
                continue;

            // The index of start vertex of the segment
            const size_t start_index = (vertices.end() - (1 + i + 1)) - vertices.begin();

            std::vector<double> vdists; // Vector of vertical distances
            vdists.reserve(current_points.size());

            // The start and end vertices of the segment
            auto start = *(vertices.begin() + start_index);
            auto end = *(vertices.begin() + start_index + 1);

            vert_dist dist(end, start);
            double max_vert_dist = 0, cur_vert_dist;
            std::pair<double, double> &max_pt = current_points.front();

            // Find point with max distance from segment (but within max_vert_dist)
            for (const auto &pt : current_points) {
                cur_vert_dist = dist(pt);
                if (cur_vert_dist > params.max_vert_dist)
                    continue;
                if (cur_vert_dist > max_vert_dist) {
                    max_vert_dist = cur_vert_dist;
                    max_pt = pt;
                }
            }

            // Check  if that point should be added.
            if (max_vert_dist > params.epsilon_dist) {
                vertices.insert(vertices.begin() + start_index + 1, max_pt);
                path_updated = true;
                ROS_INFO("Added a vertex to path, now %lu vertices.", vertices.size());

                // i.e. if the point after this goes backwards,  delete it.
                const auto prev_pt = vertices.begin() + start_index + 1;
                const auto next_pt = vertices.begin() + start_index + 2;

                if (horiz_dist(max_pt, *prev_pt)(*next_pt) <=
                    params.cos_min_angle_reverse_pt * vert_dist(max_pt, *prev_pt)(*next_pt)) {
                    vertices.erase(vertices.begin() + start_index + 2);
                    ROS_WARN("Removed a backwards point.");
                }

                // TODO: Stop new points from being added backwards..
            }
        }

        // Update the path if required.
        if (path_updated) helper.pub.lane.publish();

    } catch (const std::exception &e) {
        ROS_ERROR("Callback failed: %s", e.what());
    }
}

void process_image(const cv_bridge::CvImageConstPtr &cv_img, std::vector<cv::Point> &points,
                   const Helpers::CVParams &params, image_transport::Publisher *cv_pub,
                   std::vector<cv::Point> *hough) {
    cv::Mat hsv, blur, raw_mask, eroded_mask, masked, barrel_mask;

    // TODO: Should we just downscale image?
    cv::cvtColor(cv_img->image, hsv, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(hsv, blur, cv::Size(params.blur_size, params.blur_size), 0, 0);
    // Get white pixels
    cv::inRange(blur, params.white_lower, params.white_upper, raw_mask);

    // Flood Fill from the top of the mask to remove the sky in gazebo.
    cv::floodFill(raw_mask, cv::Point(raw_mask.cols / 2, 2), cv::Scalar(0));

    // Errors in projection increase as we approach the halfway point of the image:
    // Apply a mask to remove top 60%
    raw_mask(cv::Rect(0, 0, raw_mask.cols, (int) (raw_mask.rows * params.rect_frac))) = 0;


    // TODO: Very expensive; switch to laser scan
    std::vector<cv::Point> barrel_points; // Yellow points are part of barrel
    cv::inRange(blur, params.barrel_lower, params.barrel_upper, barrel_mask);
    cv::findNonZero(barrel_mask, barrel_points);
    if (!barrel_points.empty()) {
        int minx = barrel_mask.cols, maxx = 0;

        for (auto &v : barrel_points) {
            minx = std::min(minx, v.x);
            maxx = std::max(maxx, v.x);
        }

        if (minx < maxx)
            raw_mask(cv::Rect(minx, 0, maxx - minx, raw_mask.rows)) = 0;
    }


    cv::erode(raw_mask, eroded_mask, params.erosion_element, cv::Point(-1, -1), params.erosion_iter);

    cv_img->image.copyTo(masked, eroded_mask);

    if (hough) {
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(eroded_mask, lines, 10, CV_PI / 180, 250, 80, 10);
        ROS_ERROR("No lines detected: %lu", lines.size());
        if (!lines.empty()) {
            hough->emplace_back(lines[0][0], lines[0][1]);
            hough->emplace_back(lines[0][2], lines[0][3]);
        }
    }

    if (params.publish_masked && cv_pub)
        cv_pub->publish(cv_bridge::CvImage(cv_img->header, cv_img->encoding, masked).toImageMsg());

    cv::findNonZero(eroded_mask, points);
}

// This allows us to change params of the node while it is running: see cfg/lanes.cfg.
// Try running `rosrun rqt_reconfigure rqt_reconfigure` while node is running.
// This also auto loads any params initially set in the param server.
// The the ros page for 'dynamic_reconfigure'
//
void dynamic_reconfigure_callback(const line_lanes::LanesConfig &config, const uint32_t &level, Helpers &helper) {
    std::lock_guard<std::mutex> lock(helper.mutex);

    if (level & 1u << 0u) {
        ROS_INFO("Reconfiguring lanes params.");
        auto &params = helper.cv;

        params.white_lower = cv::Scalar(config.h_lower, config.s_lower, config.v_lower);
        params.white_upper = cv::Scalar(config.h_upper, config.s_upper, config.v_upper);
        params.erosion_size = config.erosion_size;
        params.erosion_iter = config.erosion_iter;
        params.erosion_element = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE,
                                                           cv::Size(2 * params.erosion_size + 1,
                                                                    2 * params.erosion_size + 1));
        params.blur_size = 2 * config.blur_size + 1;
        params.rect_frac = config.upper_mask_percent / 100.0;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lane");
    ros::NodeHandle nh;

    ROS_INFO("Started");

    // For receiving and publishing the images in an easy way.
    image_transport::ImageTransport imageTransport(nh);

    Helpers helper({
                           imageTransport.advertise(topic_masked, 1),
                           {nh.advertise<sensor_msgs::PointCloud2>(pc_topic, 2)},
                           {nh.advertise<line_lanes::Lane>(lane_topic, 10), num_sections}
                   });

    sensor_msgs::CameraInfo::ConstPtr camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
            topic_camera_info);
    helper.cameraModel.fromCameraInfo(camera_info);

    // Get initial path from params
    {
        std::vector<double> initial_x, initial_y;

        if (!nh.getParam("lanes_mono/initial_path/x", initial_x))
            initial_x = {0};
        if (!nh.getParam("lanes_mono/initial_path/y", initial_y))
            initial_y = {0};

        initial_x.resize(std::max(num_mutable_points + 1, std::min(initial_x.size(), initial_y.size())));
        initial_y.resize(initial_x.size());

        std::transform(initial_x.begin(), initial_x.end(), initial_y.begin(),
                       std::back_inserter(helper.pub.lane.vertices),
                       [](const double &x, const double &y) { return std::make_pair(x, y); });
        helper.pub.lane.publish();
        helper.mode = helper.MODIFYING;
    }

    if (helper.dynamic_reconfigure) {
        // For the dynamic parameter reconfiguration. see the function dynamic_reconfigure_callback
        dynamic_reconfigure::Server<line_lanes::LanesConfig> server({"cv"});
        dynamic_reconfigure::Server<line_lanes::LanesConfig>::CallbackType dynamic_reconfigure_callback_function = boost::bind(
                &dynamic_reconfigure_callback, _1, _2, boost::ref(helper));
        server.setCallback(dynamic_reconfigure_callback_function);
    }


    // Adds the callback
    image_transport::Subscriber sub = imageTransport.subscribe(
            topic_image, 2,
            boost::bind(&callback, _1, boost::ref(helper))
    );


    ROS_INFO("Spinning...");
    ros::spin();
}
