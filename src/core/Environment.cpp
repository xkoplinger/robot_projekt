#include "vacuum_game/core/Environment.h"
#include <opencv2/opencv.hpp>

namespace environment {

Environment::Environment(const Config& config)
    : config_(config)
{
    // MUSI byt COLOR
    map_image = cv::imread(config.map_filename, cv::IMREAD_COLOR);

    if (map_image.empty()) {
        throw std::runtime_error("Nepodarilo sa nacitat mapu!");
    }
}

bool Environment::isOccupied(double x, double y) const
{
    // meter -> pixel
    int px = static_cast<int>(
        (x + getWidth() / 2.0) / config_.resolution
    );

    int py = static_cast<int>(
        map_image.rows -
        ((y + getHeight() / 2.0) / config_.resolution)
    );

    // mimo mapy = stena
    if (px < 0 || px >= map_image.cols ||
        py < 0 || py >= map_image.rows)
    {
        return true;
    }

    cv::Vec3b color = map_image.at<cv::Vec3b>(py, px);

    int b = color[0];
    int g = color[1];
    int r = color[2];

    // cierna
    bool is_black =
        (r < 60 && g < 60 && b < 60);

    // cervena (ROBUSTNEJSIE)
    bool is_red =
        (r > 120 && r > g + 40 && r > b + 40);

    return is_black || is_red;
}

bool Environment::isSpawnable(double x, double y) const
{
    return !isOccupied(x, y);
}

double Environment::getWidth() const
{
    return map_image.cols * config_.resolution;
}

double Environment::getHeight() const
{
    return map_image.rows * config_.resolution;
}

}
