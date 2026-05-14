#include "vacuum_game/core/Lidar.h"

#include <cmath>

namespace lidar {

Lidar::Lidar(
    const Config& config,
    std::shared_ptr<environment::Environment> env,
    CollisionCheck collision_check)
    : config_(config),
      env_(env),
      collision_check_(collision_check)
{
}

std::vector<geometry::Point2d>
Lidar::scan(const geometry::RobotState& state) const
{
    std::vector<geometry::Point2d> hits;

    double angle_increment =
        (config_.last_ray_angle -
         config_.first_ray_angle)
        / (config_.beam_count - 1);

    for (int i = 0; i < config_.beam_count; ++i)
    {
        double beam_angle =
            state.theta +
            config_.first_ray_angle +
            i * angle_increment;

        for (double range = 0.0;
             range <= config_.max_range;
             range += 0.03)
        {
            double test_x =
                state.x +
                range * std::cos(beam_angle);

            double test_y =
                state.y +
                range * std::sin(beam_angle);

            // TERAZ lidar vidi AJ obstacle objekty
            if (collision_check_(test_x, test_y))
            {
                hits.push_back({test_x, test_y});
                break;
            }
        }
    }

    return hits;
}

}
