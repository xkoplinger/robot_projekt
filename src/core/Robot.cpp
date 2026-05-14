#include "vacuum_game/core/Robot.h"
#include <cmath>
#include <algorithm>

namespace robot {

Robot::Robot(
    const Config& config,
    const CollisionCb& collision_cb)
    : config_(config),
      collision_cb_(collision_cb)
{
    state_.x = 0.0;
    state_.y = 0.0;
    state_.theta = 0.0;

    state_.velocity = {0.0, 0.0};
    target_velocity_ = {0.0, 0.0};

    running_ = true;

    worker_thread_ = std::thread([this]()
    {
        while (running_)
        {
            double dt =
                config_.simulation_period_ms / 1000.0;

            geometry::Twist cmd;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                cmd = target_velocity_;
            }

            update(cmd, dt);

            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    config_.simulation_period_ms));
        }
    });
}

Robot::~Robot()
{
    running_ = false;

    if (worker_thread_.joinable())
        worker_thread_.join();
}

void Robot::setVelocity(
    const geometry::Twist& velocity)
{
    std::lock_guard<std::mutex> lock(mutex_);
    target_velocity_ = velocity;
}

geometry::RobotState Robot::getState() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void Robot::update(
    const geometry::Twist& target,
    double dt)
{
    // linear
    double acc_lin = 10.0;

    if (state_.velocity.linear < target.linear)
    {
        state_.velocity.linear =
            std::min(
                target.linear,
                state_.velocity.linear + acc_lin * dt);
    }
    else
    {
        state_.velocity.linear =
            std::max(
                target.linear,
                state_.velocity.linear - acc_lin * dt);
    }

    // angular
    double acc_ang = 15.0;

    if (state_.velocity.angular < target.angular)
    {
        state_.velocity.angular =
            std::min(
                target.angular,
                state_.velocity.angular + acc_ang * dt);
    }
    else
    {
        state_.velocity.angular =
            std::max(
                target.angular,
                state_.velocity.angular - acc_ang * dt);
    }

    // uloz staru poziciu
    double old_x = state_.x;
    double old_y = state_.y;
    double old_theta = state_.theta;

    // update
    state_.theta += state_.velocity.angular * dt;

    state_.x +=
        state_.velocity.linear *
        dt *
        std::cos(state_.theta);

    state_.y +=
        state_.velocity.linear *
        dt *
        std::sin(state_.theta);

    // collision
    if (collision_cb_ && collision_cb_(state_))
    {
        state_.x = old_x;
        state_.y = old_y;
        state_.theta = old_theta;

        state_.velocity.linear = 0.0;
        state_.velocity.angular = 0.0;
    }
}

void Robot::setPose(
    double x,
    double y,
    double theta)
{
    std::lock_guard<std::mutex> lock(mutex_);

    state_.x = x;
    state_.y = y;
    state_.theta = theta;
}

void Robot::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);

    state_.x = 0.0;
    state_.y = 0.0;
    state_.theta = 0.0;

    state_.velocity.linear = 0.0;
    state_.velocity.angular = 0.0;

    target_velocity_.linear = 0.0;
    target_velocity_.angular = 0.0;
}
}
