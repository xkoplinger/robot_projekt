#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <cmath>
#include "environment/Environment.h"
#include "lidar/Lidar.h"

// globalne premenne pre callback myši
struct Scene {
    std::shared_ptr<environment::Environment> env;
    std::shared_ptr<lidar::Lidar> lidar;
    cv::Mat display_map;
    double resolution;
    std::string map_path; 
};

void onMouse(int event, int x, int y, int flags, void* userdata) {
    Scene* scene = static_cast<Scene*>(userdata);
    if (!scene) return;

    // reset mapy
    if (event == cv::EVENT_RBUTTONDOWN) {
        scene->display_map = cv::imread(scene->map_path, cv::IMREAD_COLOR);
        if (!scene->display_map.empty()) {
            cv::imshow("Simulator", scene->display_map);
            std::cout << "Mapa bola vymazana." << std::endl;
        }
        return;
    }

    if (event != cv::EVENT_LBUTTONDOWN) return;

    cv::Vec3b pixel = scene->display_map.at<cv::Vec3b>(y, x);
    if (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0) {
        std::cout << "Varovanie: Klikol si na prekazku! Vyber volne miesto." << std::endl;
        return; 
    }

    // pixely na metre
    geometry::RobotState state;
    state.x = x * scene->resolution;
    state.y = y * scene->resolution;
    state.theta = 0; 

    auto scan_points = scene->lidar->scan(state);

    cv::Mat &canvas = scene->display_map;

    // body
    for (const auto& pt : scan_points) {
        int px = static_cast<int>(pt.x / scene->resolution);
        int py = static_cast<int>(pt.y / scene->resolution);
        
        if (px >= 0 && px < canvas.cols && py >= 0 && py < canvas.rows) {
            cv::circle(canvas, cv::Point(px, py), 1, cv::Scalar(0, 0, 255), -1);
        }
    }

    // robot
    cv::circle(canvas, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);

    cv::imshow("Simulator", canvas);
}

int main() {
    try {
        double res = 0.05; 
        std::string path = "map/opk-map.png";
        
        // Inicializácia prostredia
        environment::Config env_cfg;
        env_cfg.map_filename = path;
        env_cfg.resolution = res;
        auto env = std::make_shared<environment::Environment>(env_cfg);

        // 360 stupňov
        lidar::Config lidar_cfg;
        lidar_cfg.beam_count = 360;
        lidar_cfg.first_ray_angle = 0;
        lidar_cfg.last_ray_angle = 2 * M_PI;
        lidar_cfg.max_range = 5.0; 
        auto ldr = std::make_shared<lidar::Lidar>(lidar_cfg, env);

        Scene scene;
        scene.env = env;
        scene.lidar = ldr;
        scene.resolution = res;
        scene.map_path = path;
        
        scene.display_map = cv::imread(path, cv::IMREAD_COLOR);

        if (scene.display_map.empty()) throw std::runtime_error("Mapa nenajdena!");

        // Resizable
        cv::namedWindow("Simulator", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
        cv::resizeWindow("Simulator", 800, 600);
        
        cv::setMouseCallback("Simulator", onMouse, &scene);

        std::cout << "OVLADANIE:" << std::endl;
        std::cout << "  Lave tlacidlo: Umiestni robota" << std::endl;
        std::cout << "  Prave tlacidlo: Vymaz mapu" << std::endl;
        std::cout << "  ESC: Ukonci program" << std::endl;

        cv::imshow("Simulator", scene.display_map);

        while (true) {
            int key = cv::waitKey(10);
            if (key == 27) break;
        }

    } catch (const std::exception& e) {
        std::cerr << "Chyba: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
