#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <cmath>
#include <vector>
#include "environment/Environment.h"
#include "lidar/Lidar.h"
#include "robot/Robot.h"

struct Scene {
    std::shared_ptr<environment::Environment> env;
    std::shared_ptr<lidar::Lidar> lidar;
    cv::Mat display_map;
    double resolution;
    std::string map_path; 
    std::vector<geometry::Point2d> clicked_points; // Pridané
};

// Callback funkcia pre OpenCV
void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        Scene* scene = static_cast<Scene*>(userdata);
        // Prepočet z pixelov na metre
        scene->clicked_points.push_back({x * scene->resolution, y * scene->resolution});
    }
}

int main() {
    try {
        std::string path = "map/opk-map.png";
        double res = 0.05;

        environment::Config env_cfg;
        env_cfg.map_filename = path;
        env_cfg.resolution = res;
        auto env = std::make_shared<environment::Environment>(env_cfg);

        lidar::Config lidar_cfg;
        lidar_cfg.beam_count = 360;
        lidar_cfg.first_ray_angle = 0;
        lidar_cfg.last_ray_angle = 2 * M_PI;
        lidar_cfg.max_range = 5.0; 
        auto ldr = std::make_shared<lidar::Lidar>(lidar_cfg, env);

        robot::Config rob_cfg;
        rob_cfg.simulation_period_ms = 10;
        rob_cfg.command_duration = 0.5;
        rob_cfg.accelerations = {4.0, 8.0};
        rob_cfg.emergency_decelerations = {12.0, 20.0}; // Pridané

        double robot_radius = 8 * res;

        // kolízia
        auto col_fn = [env, robot_radius](geometry::RobotState s) { 
            
            // kontrola stredu 
            if (env->isOccupied(s.x, s.y)) return true;

            int samples = 128;

            for (int i = 0; i < samples; i++) {
                double angle = 2 * M_PI * i / samples;

                double check_x = s.x + robot_radius * cos(angle);
                double check_y = s.y + robot_radius * sin(angle);

                if (env->isOccupied(check_x, check_y)) {
                    return true;
                }
            }

            return false;
        };

        robot::Robot my_robot(rob_cfg, col_fn);

        Scene scene;
        scene.env = env;
        scene.lidar = ldr;
        scene.resolution = res;
        scene.map_path = path;

        cv::namedWindow("Simulator", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
        cv::resizeWindow("Simulator", 900, 700);
        cv::setMouseCallback("Simulator", onMouse, &scene);

        while (true) {
            geometry::RobotState state = my_robot.getState();
            auto scan_results = ldr->scan(state);

            cv::Mat canvas = cv::imread(path, cv::IMREAD_COLOR);
            if (canvas.empty()) break;

            // Vykreslenie kliknutých bodov a ich lidaru (Bod 91 zadania)
            for (const auto& pos : scene.clicked_points) {
                cv::drawMarker(canvas, cv::Point(pos.x / res, pos.y / res), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 2);
                
                geometry::RobotState temp_state;
                temp_state.x = pos.x; temp_state.y = pos.y; temp_state.theta = 0;
                auto click_scan = ldr->scan(temp_state);
                for (const auto& hit : click_scan) {
                    cv::circle(canvas, cv::Point(hit.x / res, hit.y / res), 1, cv::Scalar(0, 255, 255), -1);
                }
            }

            // lidar aktuálneho robota
            for (const auto& hit : scan_results) {
                cv::circle(canvas, cv::Point(hit.x / res, hit.y / res), 1, cv::Scalar(0, 0, 255), -1);
            }

            // robot väčší
            int radius_px = static_cast<int>(robot_radius / res);
            int cx = state.x / res;
            int cy = state.y / res;

            cv::circle(canvas, cv::Point(cx, cy), radius_px, cv::Scalar(0, 255, 0), -1);

            // smer 
            double nose_len = robot_radius * 1.5;
            int nx = (state.x + nose_len * cos(state.theta)) / res;
            int ny = (state.y + nose_len * sin(state.theta)) / res;

            cv::circle(canvas, cv::Point(nx, ny), 3, cv::Scalar(255, 0, 0), -1);

            cv::imshow("Simulator", canvas);

            int key = cv::waitKey(10);
            if (key == 27) break;

            geometry::Twist current_cmd = {0.0, 0.0};

            if (key == 'w') current_cmd.linear = 3.0;
            if (key == 's') current_cmd.linear = -3.0;
            if (key == 'a') current_cmd.angular = -3.5;
            if (key == 'd') current_cmd.angular = 3.5;

            my_robot.setVelocity(current_cmd);
        }

    } catch (const std::exception& e) {
        std::cerr << "Chyba: " << e.what() << std::endl;
    }

    return 0;
}
