#pragma once
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include "types/Geometry.h"

namespace robot {

struct Config {
    geometry::Twist accelerations;
    geometry::Twist emergency_decelerations;
    double command_duration;
    int simulation_period_ms;
};

class Robot {
public:
    using CollisionCb = std::function<bool(geometry::RobotState)>;
    Robot(const Config& config, const CollisionCb& collision_cb = nullptr);
    ~Robot();
    void setVelocity(const geometry::Twist& velocity);
    geometry::RobotState getState() const;
    bool isInCollision() const;

protected:
    void update(const geometry::Twist& velocity, double dt);

private:
    Config config_;
    geometry::RobotState state_; 
    CollisionCb collision_cb_;

    std::thread worker_thread_;
    mutable std::mutex mutex_;
    std::atomic<bool> running_{false};
    geometry::Twist target_velocity_;
    std::chrono::steady_clock::time_point last_command_time_;
};
}
