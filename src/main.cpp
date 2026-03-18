#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include "environment/Environment.h"
#include "lidar/Lidar.h"

// Globálne premenné pre callback myši
struct Scene {
    std::shared_ptr<environment::Environment> env;
    std::shared_ptr<lidar::Lidar> lidar;
    cv::Mat display_map;
    double resolution;
};

void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event != cv::EVENT_LBUTTONDOWN) return;

    Scene* scene = static_cast<Scene*>(userdata);

    // pixely na metre
    geometry::RobotState state;
    state.x = x * scene->resolution;
    state.y = y * scene->resolution;
    state.theta = 0; // robot sa pozerá "dopredu"

    // spustenie lidaru
    auto scan_points = scene->lidar->scan(state);

    // vytvorime pokiu cistej mapy
    cv::Mat canvas = scene->display_map.clone();
    cv::cvtColor(canvas, canvas, cv::COLOR_GRAY2BGR); // farebny pre cervene bodky

    // vykreslenie bodov lidaru cervene
    for (const auto& pt : scan_points) {
        int px = static_cast<int>(pt.x / scene->resolution);
        int py = static_cast<int>(pt.y / scene->resolution);
        cv::circle(canvas, cv::Point(px, py), 2, cv::Scalar(0, 0, 255), -1);
    }

    // vykreslenie robota
    cv::circle(canvas, cv::Point(x, y), 5, cv::Scalar(0, 255, 0), -1);

    cv::imshow("Simulator", canvas);
}

int main() {
    try {
        double res = 0.05; 
        
        // Inicializácia prostredia
        environment::Config env_cfg;
        env_cfg.map_filename = "map/opk-map.png";
        env_cfg.resolution = res;
        auto env = std::make_shared<environment::Environment>(env_cfg);

        // 360 stupnov, 360 lucov
        lidar::Config lidar_cfg;
        lidar_cfg.beam_count = 360;
        lidar_cfg.first_ray_angle = 0;
        lidar_cfg.last_ray_angle = 2 * M_PI;
        lidar_cfg.max_range = 5.0; // 5 metrov dosah
        auto ldr = std::make_shared<lidar::Lidar>(lidar_cfg, env);

        // priprava sceny pre mys
        Scene scene;
        scene.env = env;
        scene.lidar = ldr;
        scene.resolution = res;
        scene.display_map = cv::imread(env_cfg.map_filename, cv::IMREAD_GRAYSCALE);

        if (scene.display_map.empty()) throw std::runtime_error("Mapa nenajdena!");

        cv::namedWindow("Simulator");
        cv::setMouseCallback("Simulator", onMouse, &scene);

        std::cout << "Klikni niekam do mapy pre umiestnenie robota..." << std::endl;
        cv::imshow("Simulator", scene.display_map);
        cv::waitKey(0);

    } catch (const std::exception& e) {
        std::cerr << "Chyba: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
