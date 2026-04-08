#include "environment/Environment.h"
#include <opencv2/opencv.hpp>
#include <stdexcept>

namespace environment {

// uloženie obrazu
Environment::Environment(const Config& config) : config_(config) {
    map_image = cv::imread(config.map_filename, cv::IMREAD_GRAYSCALE);
    if (map_image.empty()) {
        throw std::runtime_error("Nepodarilo sa nacitat mapu: " + config.map_filename);
    }
}

bool Environment::isOccupied(double x, double y) const {
    // prepocet metrov na pixely, rozlisenie: x_pixel = x_meter 
    int px = static_cast<int>(x / config_.resolution);
    int py = static_cast<int>(y / config_.resolution);

    if (px < 0 || px >= map_image.cols || py < 0 || py >= map_image.rows) return true;
    
    // 0 stena
    return map_image.at<uchar>(py, px) == 0;
}

}
