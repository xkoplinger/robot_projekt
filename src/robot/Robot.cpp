#include "robot/Robot.h"
#include <cmath>
#include <algorithm>

namespace robot {

Robot::Robot(const Config& config, const CollisionCb& collision_cb)
    : config_(config), collision_cb_(collision_cb) {
    
    // Inicializácia stavu (x, y, theta)
    state_.x = 10.0; 
    state_.y = 10.0;
    state_.theta = 0.0;
    state_.velocity = {0.0, 0.0};
    
    target_velocity_ = {0.0, 0.0};
    last_command_time_ = std::chrono::steady_clock::now();
    running_ = true; // Atómová premenná z tvojho .h [cite: 72]

    // Vytvorenie vlákna pre simuláciu dynamiky
    worker_thread_ = std::thread([this]() {
        while (running_) {
            double dt = config_.simulation_period_ms / 1000.0;

            geometry::Twist current_cmd;
            {
                // Ošetrený prístup cez mutex 
                std::lock_guard<std::mutex> lock(mutex_);
                current_cmd = target_velocity_;
            }

            // Periodická aktualizácia stavu
            update(current_cmd, dt);

            std::this_thread::sleep_for(std::chrono::milliseconds(config_.simulation_period_ms));
        }
    });
}

Robot::~Robot() {
    running_ = false; // prerušenie vlákna
    if (worker_thread_.joinable()) {
        worker_thread_.join(); // korektné ukončenie
    }
}

void Robot::setVelocity(const geometry::Twist& velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_velocity_ = velocity;
    last_command_time_ = std::chrono::steady_clock::now(); // reset času platnosti príkazu
}

geometry::RobotState Robot::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void Robot::update(const geometry::Twist& velocity, double dt) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Kontrola platnosti príkazu
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_command_time_).count();
    
    geometry::Twist effective_target = velocity;
    bool is_emergency = false;

    if (elapsed > config_.command_duration) {
        effective_target = {0.0, 0.0}; // zastavenie, ak nedostal príkaz
        is_emergency = true;
    }

    // 2. Výber zrýchlenia
    double acc_lin = is_emergency ? config_.emergency_decelerations.linear : config_.accelerations.linear;
    double acc_ang = is_emergency ? config_.emergency_decelerations.angular : config_.accelerations.angular;

    // Lineárna rýchlosť
    if (state_.velocity.linear < effective_target.linear) {
        state_.velocity.linear = std::min(effective_target.linear, state_.velocity.linear + acc_lin * dt);
    } else if (state_.velocity.linear > effective_target.linear) {
        state_.velocity.linear = std::max(effective_target.linear, state_.velocity.linear - acc_lin * dt);
    }

    // Uhlová rýchlosť
    if (state_.velocity.angular < effective_target.angular) {
        state_.velocity.angular = std::min(effective_target.angular, state_.velocity.angular + acc_ang * dt);
    } else if (state_.velocity.angular > effective_target.angular) {
        state_.velocity.angular = std::max(effective_target.angular, state_.velocity.angular - acc_ang * dt);
    }

    // 4. Kinematika - výpočet polohy
    state_.theta += state_.velocity.angular * dt;
    state_.x += state_.velocity.linear * dt * std::cos(state_.theta);
    state_.y += state_.velocity.linear * dt * std::sin(state_.theta);

    // 5. Detekcia kolízie 
    if (collision_cb_ && collision_cb_(state_)) {
        state_.velocity = {0.0, 0.0}; // Zastavenie pri náraze
    }
}

bool Robot::isInCollision() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (collision_cb_) return collision_cb_(state_);
    return false;
}

} // namespace robot
