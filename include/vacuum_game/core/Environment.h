#pragma once
#include <string>
#include <opencv2/opencv.hpp>

namespace environment {

struct Config {
    std::string map_filename;
    double resolution;
};

class Environment {
public:
    explicit Environment(const Config& config);
    bool isOccupied(double x, double y) const;
    bool isSpawnable(double x, double y) const; // PRIDANÉ
    double getWidth() const;
    double getHeight() const;

private:
    Config config_;
    cv::Mat map_image; 
};

}
