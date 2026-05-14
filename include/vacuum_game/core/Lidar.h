#pragma once

#include <vector>
#include <memory>
#include <functional>

#include "vacuum_game/core/Geometry.h"
#include "vacuum_game/core/Environment.h"

namespace lidar {

struct Config
{
    double max_range;
    int beam_count;
    double first_ray_angle;
    double last_ray_angle;
};

class Lidar
{
public:

    // CALLBACK pre collision
    using CollisionCheck =
        std::function<bool(double, double)>;

    Lidar(
        const Config& config,
        std::shared_ptr<environment::Environment> env,
        CollisionCheck collision_check);

    std::vector<geometry::Point2d>
    scan(const geometry::RobotState& state) const;

private:

    Config config_;

    std::shared_ptr<environment::Environment>
        env_;

    CollisionCheck collision_check_;
};

}
