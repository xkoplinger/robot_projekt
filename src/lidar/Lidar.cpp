#include "lidar/Lidar.h"
#include <cmath>

namespace lidar {
Lidar::Lidar(const Config& config, std::shared_ptr<environment::Environment> env) 
    : config_(config), env_(env) {}

std::vector<geometry::Point2d> Lidar::scan(const geometry::RobotState& state) const {
    std::vector<geometry::Point2d> hits;
    double angle_increment = (config_.last_ray_angle - config_.first_ray_angle) / (config_.beam_count - 1);

    for (int i = 0; i < config_.beam_count; ++i) {
        double beam_angle = state.theta + config_.first_ray_angle + (i * angle_increment);
        
        // luce svetla
        for (double range = 0; range < config_.max_range; range += 0.05) { // krok 5cm
            double test_x = state.x + range * std::cos(beam_angle);
            double test_y = state.y + range * std::sin(beam_angle);

            if (env_->isOccupied(test_x, test_y)) {
                hits.push_back({test_x, test_y});
                break;
            }
        }
    }
    return hits;
}
}
