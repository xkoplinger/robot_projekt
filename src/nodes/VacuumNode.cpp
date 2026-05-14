#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "vacuum_game/core/Robot.h"
#include "vacuum_game/core/ObjectFactory.hpp"
#include "vacuum_game/core/Environment.h"
#include "vacuum_game/core/Lidar.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <map>

class VacuumNode : public rclcpp::Node
{
public:

    VacuumNode()
        : Node("vacuum_node")
    {
        std::string pkg_path =
    ament_index_cpp::get_package_share_directory(
        "vacuum_game");

this->declare_parameter("map_file", "opk-map.png");
this->declare_parameter("resolution", 0.05);

this->declare_parameter("robot_radius", 0.45);

this->declare_parameter("max_capacity", 10);

this->declare_parameter("obstacle_spawn_count", 8);
this->declare_parameter("trash_spawn_count", 5);

this->declare_parameter("lidar_range", 6.0);
this->declare_parameter("lidar_beams", 150);

this->declare_parameter("station_x", 2.5);
this->declare_parameter("station_y", 2.5);

this->declare_parameter(
    "trash_radius_min",
    0.2);

this->declare_parameter(
    "trash_radius_max",
    0.5);

// =====================================================

std::string map_file =
    this->get_parameter("map_file").as_string();

double resolution =
    this->get_parameter("resolution").as_double();

double robot_radius =
    this->get_parameter("robot_radius").as_double();

max_capacity_ =
    this->get_parameter("max_capacity").as_int();

int obstacle_count =
    this->get_parameter("obstacle_spawn_count").as_int();

int trash_count =
    this->get_parameter("trash_spawn_count").as_int();

double lidar_range =
    this->get_parameter("lidar_range").as_double();

int lidar_beams =
    this->get_parameter("lidar_beams").as_int();

station_pos_.x =
    this->get_parameter("station_x").as_double();

station_pos_.y =
    this->get_parameter("station_y").as_double();
    
trash_radius_min_ =
    this->get_parameter(
        "trash_radius_min").as_double();

trash_radius_max_ =
    this->get_parameter(
        "trash_radius_max").as_double();

// =====================================================

environment::Config env_cfg{
    pkg_path + "/map/" + map_file,
    resolution
};

        env_ptr_ =
            std::make_shared<environment::Environment>(
                env_cfg);

        // LIDAR
        lidar::Config lid_cfg{
    lidar_range,
    lidar_beams,
    0.0,
    6.28
};

        lidar_ptr_ =
            std::make_unique<lidar::Lidar>(
                lid_cfg,
                env_ptr_,
                [this](double x, double y)
                {
                    return this->isObstacleCollision(x, y);
                });

        // ROBOT
        robot::Config rob_cfg;
        rob_cfg.simulation_period_ms = 10;
        this->declare_parameter(
              "game_mode",
              "ai");

        game_mode_ =
            this->get_parameter(
                "game_mode").as_string();



        my_robot_ =
            std::make_unique<robot::Robot>(
                rob_cfg,
                [this, robot_radius](geometry::RobotState s)
                {
                    for (int i = 0; i < 32; i++)
                    {
                        double angle =
                            2.0 * M_PI * i / 32.0;

                        double check_x =
                            s.x +
                            robot_radius *
                            std::cos(angle);

                        double check_y =
                            s.y +
                            robot_radius *
                            std::sin(angle);

                        if (this->isObstacleCollision(
                                check_x,
                                check_y))
                        {
                            return true;
                        }
                    }

                    return false;
                });
        
        if (game_mode_ == "ai")
        {
            // AI ignoruje kolizie
            enemy_robot_ =
                std::make_unique<robot::Robot>(
                    rob_cfg,
                    nullptr);
        }
        else
        {
            // PVP ma kolizie
            enemy_robot_ =
                std::make_unique<robot::Robot>(
                    rob_cfg,
                    [this, robot_radius](geometry::RobotState s)
                    {
                        for (int i = 0; i < 32; i++)
                        {
                            double angle =
                                2.0 * M_PI * i / 32.0;

                            double check_x =
                                s.x +
                                robot_radius *
                                std::cos(angle);

                            double check_y =
                                s.y +
                                robot_radius *
                                std::sin(angle);

                            if (this->isObstacleCollision(
                                    check_x,
                                    check_y))
                            {
                                return true;
                            }
                        }

                        return false;
                    });
        }

        enemy_robot_->reset();
        enemy_robot_->setPose(3.0, 3.0, 0.0);
        
        // publishers
        marker_array_pub_ =
            this->create_publisher
            <visualization_msgs::msg::MarkerArray>(
                "visualization_marker_array",
                10);

        lidar_pub_ =
            this->create_publisher
            <visualization_msgs::msg::Marker>(
                "lidar_marker",
                10);
        scan_pub_ =
            this->create_publisher
            <sensor_msgs::msg::LaserScan>(
                "scan",
                10);
        
        reset_service_ =
            this->create_service<std_srvs::srv::Empty>(
                "reset_game",
                [this](
                    const std::shared_ptr<std_srvs::srv::Empty::Request>,
                    std::shared_ptr<std_srvs::srv::Empty::Response>)
                {
                    this->world_version_++;
                    world_objects_.clear();
                    
                    player1_inventory_.clear();
                    player2_inventory_.clear();

                    player1_load_ = 0;
                    player2_load_ = 0;

                    player1_score_ = 0;
                    player2_score_ = 0;
                    
                    game_finished_ = false;

                    game_start_time_ = this->now();

                    // reset robota
                    my_robot_->reset();
                    enemy_robot_->reset();

                    enemy_robot_->setPose(
                        3.0,
                        3.0,
                        0.0);

                    // reset path
                    robot_path_.poses.clear();
                    enemy_path_.poses.clear();

                    // spawn znova
                    for(int i = 0; i < 8; i++)
                        spawnObject("obstacle");

                    for(int i = 0; i < 5; i++)
                        spawnObject("trash");

                    RCLCPP_INFO(
                        this->get_logger(),
                        "Hra resetovana");
                });
        
        map_pub_ =
            this->create_publisher
            <nav_msgs::msg::OccupancyGrid>(
                "map",
                rclcpp::QoS(1).transient_local());
        odom_pub_ =
            this->create_publisher
            <nav_msgs::msg::Odometry>(
                "odom",
                10);
        
        enemy_odom_pub_ =
            this->create_publisher
            <nav_msgs::msg::Odometry>(
                "/robot2/odom",
                10);
        
        path_pub_ =
            this->create_publisher
            <nav_msgs::msg::Path>(
                "path",
                10);
                
        enemy_path_pub_ =
            this->create_publisher
            <nav_msgs::msg::Path>(
                "/robot2/path",
                10);
        
        
        game_state_pub_string_ =
            this->create_publisher
            <std_msgs::msg::String>(
                "game_state_string",
                10);

        robot_path_.header.frame_id = "map";
        enemy_path_.header.frame_id = "map";

        // TF
        tf_static_broadcaster_ =
            std::make_shared
            <tf2_ros::StaticTransformBroadcaster>(
                this);

        publish_static_tf();

        // CMD VEL
        sub_ =
            this->create_subscription
            <geometry_msgs::msg::Twist>(
                "cmd_vel",
                10,
                [this](
                    const geometry_msgs::msg::Twist::SharedPtr msg)
                {
                    geometry::Twist t;

                    t.linear =
                        msg->linear.x * 4.0;

                    t.angular =
                        msg->angular.z * 3.5;

                    this->my_robot_->setVelocity(t);
                });
        
        enemy_sub_ =
            this->create_subscription
            <geometry_msgs::msg::Twist>(
                "/robot2/cmd_vel",
                10,
                [this](
                    const geometry_msgs::msg::Twist::SharedPtr msg)
                {
                    geometry::Twist t;

                    t.linear =
                        msg->linear.x * 4.0;

                    t.angular =
                        msg->angular.z * 3.5;

                    enemy_robot_->setVelocity(t);
                });
        
        player1_load_ = 0;
        player2_load_ = 0;

        // SPAWN
        for(int i = 0; i < obstacle_count; i++)
            spawnObject("obstacle");

        for(int i = 0; i < trash_count; i++)
            spawnObject("trash");

        publish_static_map();

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(30),
                std::bind(
                    &VacuumNode::update_loop,
                    this));

        map_timer_ =
            this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(
                    &VacuumNode::publish_static_map,
                    this));
                    
        game_start_time_ = this->now();
    }

private:

    // ======================================================
    // NOVÁ FUNKCIA
    // ======================================================

    bool isObstacleCollision(double x, double y)
    {
        // mapa
        if (env_ptr_->isOccupied(x, y))
            return true;

        // obstacle objekty
        for (const auto& obj : world_objects_)
        {
            if (obj->getType() == "obstacle")
            {
                double dx = x - obj->x;
                double dy = y - obj->y;

                double dist =
                    std::sqrt(dx * dx + dy * dy);

                if (dist < 0.8)
                    return true;
            }
        }

        return false;
    }

    // ======================================================
    
    void publish_path(
    const geometry::RobotState& state)
{
    geometry_msgs::msg::PoseStamped pose;

    pose.header.stamp = this->now();
    pose.header.frame_id = "map";

    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.position.z = 0.0;

    double qz =
        std::sin(state.theta / 2.0);

    double qw =
        std::cos(state.theta / 2.0);

    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;

    robot_path_.header.stamp = this->now();

    robot_path_.poses.push_back(pose);

    path_pub_->publish(robot_path_);
}
    
    void publish_odometry(
    const geometry::RobotState& state)
{
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = this->now();
    odom.header.frame_id = "map";

    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = state.x;
    odom.pose.pose.position.y = state.y;
    odom.pose.pose.position.z = 0.0;

    double qz =
        std::sin(state.theta / 2.0);

    double qw =
        std::cos(state.theta / 2.0);

    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;

    odom.twist.twist.linear.x =
        state.velocity.linear;

    odom.twist.twist.angular.z =
        state.velocity.angular;

    odom_pub_->publish(odom);
}
    
    void publish_static_tf()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->now();

        t.header.frame_id = "map";
        t.child_frame_id = "base_link";

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        t.transform.rotation.w = 1.0;

        tf_static_broadcaster_->sendTransform(t);
    }

    void spawnObject(std::string type)
    {
        for(int i = 0; i < 100; i++)
        {
            double rx =
                (rand() % 6000 - 3000) / 100.0;

            double ry =
                (rand() % 4000 - 2000) / 100.0;

            if (std::hypot(
                    rx - station_pos_.x,
                    ry - station_pos_.y) < 2.5)
            {
                continue;
            }

            if (!isObstacleCollision(rx, ry))
            {
                bool too_close = false;

                for (const auto& obj : world_objects_)
                {
                    double dist =
                        std::hypot(
                            obj->x - rx,
                            obj->y - ry);

                    if (dist < 1.5)
                    {
                        too_close = true;
                        break;
                    }
                }

                if (too_close)
                    continue;
                double radius =
                    trash_radius_min_ +
                    ((double) rand() / RAND_MAX) *
                    (trash_radius_max_ - trash_radius_min_);

                auto obj =
                    ObjectFactory::createObject(
                        type,
                        rx,
                        ry,
                        radius);
                obj->version = world_version_;
                world_objects_.push_back(std::move(obj));

                break;
            }
        }
    }

    void update_loop()
    {
        if (game_finished_)
        {
            return;
        }
        auto state = my_robot_->getState();
        auto enemy_state = enemy_robot_->getState();
        if (game_mode_ == "ai")
        {
            controlEnemyBot(enemy_state);
        }

        auto hits =
            lidar_ptr_->scan(state);

        publish_lidar(hits);
        publish_scan(state, hits);
        publish_odometry(state);
        publish_path(state);
        publish_enemy_path(enemy_state);

        static int ticks = 0;

        if (++ticks > 40)
        {
            ticks = 0;
            spawnObject("trash");
        }


        auto it = world_objects_.begin();

        while (it != world_objects_.end())
        {
            
            auto& obj = *it;

            if (obj->version != world_version_)
            {
                it = world_objects_.erase(it);
                continue;
            }              
            
            if (obj->getType() != "obstacle")
            {
                double dist =
                    std::hypot(
                        obj->x - state.x,
                        obj->y - state.y);
                double enemy_dist =
                    std::hypot(
                        obj->x - enemy_state.x,
                        obj->y - enemy_state.y);

                if (dist < 0.7 || enemy_dist < 0.7)
                {
                    if (
                        (dist < 0.7 && player1_load_ < max_capacity_)
                        ||
                        (enemy_dist < 0.7 && player2_load_ < max_capacity_)
                    )
                    {
                        std::string color =
                            obj->getColor();
                        
                        if (dist < 0.7)
                        {
                            player1_inventory_[color]++;
                            player1_load_++;
                        }
                        else
                        {
                            player2_inventory_[color]++;
                            player2_load_++;
                        }
                        
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Nazbierany odpad: %s",
                            color.c_str());

                        if (dist < 0.7)
                        {
                            RCLCPP_INFO(
                                this->get_logger(),
                                "PLAYER 1 LOAD: %d/%d",
                                player1_load_,
                                max_capacity_);
                        }
                        else
                        {
                            RCLCPP_INFO(
                                this->get_logger(),
                                "PLAYER 2 LOAD: %d/%d",
                                player2_load_,
                                max_capacity_);
                        }

                        it = world_objects_.erase(it);


                        break;
                    }
                }
            }

            ++it;
        }

        // PLAYER 1

        if (std::hypot(
                state.x - station_pos_.x,
                state.y - station_pos_.y) < 1.2
            && player1_load_ > 0)
        {
            player1_score_ +=
                player1_load_ * 10;

            RCLCPP_INFO(
                this->get_logger(),
                "PLAYER 1 SCORE: %d",
                player1_score_);

            RCLCPP_INFO(
                this->get_logger(),
                "--- PLAYER 1 VYLOZIL ODPAD ---");

            for (auto const& [color, count]
                : player1_inventory_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "P1 %s: %d ks",
                    color.c_str(),
                    count);
            }

            player1_load_ = 0;

            player1_inventory_.clear();
        }

        // PLAYER 2

        if (std::hypot(
                enemy_state.x - station_pos_.x,
                enemy_state.y - station_pos_.y) < 1.2
            && player2_load_ > 0)
        {
            player2_score_ +=
                player2_load_ * 10;

            RCLCPP_INFO(
                this->get_logger(),
                "PLAYER 2 SCORE: %d",
                player2_score_);

            RCLCPP_INFO(
                this->get_logger(),
                "--- PLAYER 2 VYLOZIL ODPAD ---");

            for (auto const& [color, count]
                : player2_inventory_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "P2 %s: %d ks",
                    color.c_str(),
                    count);
            }

            player2_load_ = 0;

            player2_inventory_.clear();
        }
        publish_game_state_string();

        publish_markers(state, enemy_state);
        publish_enemy_odometry(enemy_state);
        double elapsed =
            (this->now() - game_start_time_).seconds();

        if (elapsed >= game_duration_seconds_)
        {
            game_finished_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "===== GAME OVER =====");

            RCLCPP_INFO(
                this->get_logger(),
                "PLAYER 1 SCORE: %d",
                player1_score_);

            RCLCPP_INFO(
                this->get_logger(),
                "PLAYER 2 SCORE: %d",
                player2_score_);

            if (player1_score_ > player2_score_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "WINNER: PLAYER 1");
            }
            else if (player2_score_ > player1_score_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "WINNER: PLAYER 2");
            }
            else
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "DRAW");
            }
        }
    }

    void publish_lidar(
        const std::vector<geometry::Point2d>& hits)
    {
        visualization_msgs::msg::Marker m;

        m.header.frame_id = "map";
        m.header.stamp = this->now();

        m.ns = "lidar";
        m.id = 0;

        m.type = 8;

        m.scale.x = 0.1;
        m.scale.y = 0.1;

        m.color.r = 1.0;
        m.color.a = 0.6;

        for (auto& h : hits)
        {
            geometry_msgs::msg::Point p;

            p.x = h.x;
            p.y = h.y;
            p.z = 0.1;

            m.points.push_back(p);
        }

        lidar_pub_->publish(m);
    }
    
    void publish_scan(
    const geometry::RobotState& state,
    const std::vector<geometry::Point2d>& hits)
{
    sensor_msgs::msg::LaserScan scan;

    scan.header.stamp = this->now();
    scan.header.frame_id = "base_link";

    scan.angle_min = 0.0;
    scan.angle_max = 6.28;

    scan.angle_increment =
        (scan.angle_max - scan.angle_min)
        / 150.0;

    scan.range_min = 0.0;
    scan.range_max = 6.0;

    scan.ranges.resize(150);

    for (size_t i = 0; i < scan.ranges.size(); i++)
    {
        scan.ranges[i] = scan.range_max;
    }

    for (size_t i = 0;
         i < hits.size() && i < scan.ranges.size();
         i++)
    {
        double dx =
            hits[i].x - state.x;

        double dy =
            hits[i].y - state.y;

        double dist =
            std::sqrt(dx * dx + dy * dy);

        scan.ranges[i] = dist;
    }

    scan_pub_->publish(scan);
}
    
    void publish_enemy_odometry(
    const geometry::RobotState& state)
{
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = this->now();
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = state.x;
    odom.pose.pose.position.y = state.y;

    double qz =
        std::sin(state.theta / 2.0);

    double qw =
        std::cos(state.theta / 2.0);

    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;

    enemy_odom_pub_->publish(odom);
}
    
    void reset_game(
    const std::shared_ptr
    <std_srvs::srv::Empty::Request>,
    
    std::shared_ptr
    <std_srvs::srv::Empty::Response>)
{
    world_objects_.clear();

    player1_load_ = 0;
    player2_load_ = 0;

    player1_inventory_.clear();
    player2_inventory_.clear();

    robot_path_.poses.clear();

    for(int i = 0; i < 8; i++)
        spawnObject("obstacle");

    for(int i = 0; i < 5; i++)
        spawnObject("trash");

    RCLCPP_INFO(
        this->get_logger(),
        "Hra resetovana");
}

WorldObject* findNearestTrash(
    const geometry::RobotState& state)
{
    WorldObject* best = nullptr;

    double best_dist = 999999.0;

    for (auto& obj : world_objects_)
    {
        if (obj->getType() == "trash")
        {
            double d =
                std::hypot(
                    obj->x - state.x,
                    obj->y - state.y);

            if (d < best_dist)
            {
                best_dist = d;
                best = obj.get();
            }
        }
    }

    return best;
}

void controlEnemyBot(
    const geometry::RobotState& enemy_state)
{
    geometry::Twist cmd;

    double target_x;
    double target_y;

    if (player2_load_ >= max_capacity_)
    {
        target_x = station_pos_.x;
        target_y = station_pos_.y;
    }
    else
    {
        auto target =
            findNearestTrash(enemy_state);

        if (!target)
            return;

        target_x = target->x;
        target_y = target->y;
    }

    double dx =
        target_x - enemy_state.x;

    double dy =
        target_y - enemy_state.y;

    double desired_angle =
        std::atan2(dy, dx);

    double angle_error =
        desired_angle - enemy_state.theta;

    while (angle_error > M_PI)
        angle_error -= 2.0 * M_PI;

    while (angle_error < -M_PI)
        angle_error += 2.0 * M_PI;

    cmd.angular = angle_error * 2.0;

    cmd.linear =
        2.0 * std::max(
            0.0,
            1.0 - std::abs(angle_error));

    enemy_robot_->setVelocity(cmd);
}

void publish_markers(
    geometry::RobotState& state,
    geometry::RobotState& enemy_state)
{
    visualization_msgs::msg::MarkerArray ma;

    // =====================================================
    // CLEAR
    // =====================================================

    visualization_msgs::msg::Marker clear_marker;

    clear_marker.action =
        visualization_msgs::msg::Marker::DELETEALL;

    ma.markers.push_back(clear_marker);

    auto now = this->now();

    // =====================================================
    // PLAYER 1
    // =====================================================

    visualization_msgs::msg::Marker m;

    double qz =
        std::sin(state.theta / 2.0);

    double qw =
        std::cos(state.theta / 2.0);

    m.header.frame_id = "map";
    m.header.stamp = now;

    m.ns = "player1";
    m.id = 0;

    m.type =
        visualization_msgs::msg::Marker::CYLINDER;

    m.action =
        visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = state.x;
    m.pose.position.y = state.y;
    m.pose.position.z = 0.25;

    m.pose.orientation.z = qz;
    m.pose.orientation.w = qw;

    m.scale.x = 0.8;
    m.scale.y = 0.8;
    m.scale.z = 0.4;

    // ORANZOVY
    m.color.r = 1.0;
    m.color.g = 0.5;
    m.color.b = 0.0;
    m.color.a = 1.0;

    ma.markers.push_back(m);
    
    // =====================================================
    // PLAYER 1 ARROW
    // =====================================================

    visualization_msgs::msg::Marker arrow;

    arrow.header.frame_id = "map";
    arrow.header.stamp = now;

    arrow.ns = "player1_arrow";
    arrow.id = 500;

    arrow.type =
        visualization_msgs::msg::Marker::ARROW;

    arrow.action =
        visualization_msgs::msg::Marker::ADD;

    arrow.pose.position.x = state.x;
    arrow.pose.position.y = state.y;
    arrow.pose.position.z = 0.5;

    arrow.pose.orientation.z = qz;
    arrow.pose.orientation.w = qw;

    arrow.scale.x = 1.0;
    arrow.scale.y = 0.12;
    arrow.scale.z = 0.12;

    arrow.color.r = 1.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;

    ma.markers.push_back(arrow);

    // =====================================================
    // PLAYER 2
    // =====================================================

    qz =
        std::sin(enemy_state.theta / 2.0);

    qw =
        std::cos(enemy_state.theta / 2.0);

    m.ns = "player2";
    m.id = 1000;

    m.pose.position.x = enemy_state.x;
    m.pose.position.y = enemy_state.y;
    m.pose.position.z = 0.25;

    m.pose.orientation.z = qz;
    m.pose.orientation.w = qw;

    m.scale.x = 0.8;
    m.scale.y = 0.8;
    m.scale.z = 0.4;

    // MODRY
    m.color.r = 0.0;
    m.color.g = 0.4;
    m.color.b = 1.0;
    m.color.a = 1.0;

    ma.markers.push_back(m);
    
    // =====================================================
    // PLAYER 2 ARROW
    // =====================================================

    visualization_msgs::msg::Marker enemy_arrow;

    enemy_arrow.header.frame_id = "map";
    enemy_arrow.header.stamp = now;

    enemy_arrow.ns = "player2_arrow";
    enemy_arrow.id = 1500;

    enemy_arrow.type =
        visualization_msgs::msg::Marker::ARROW;

    enemy_arrow.action =
        visualization_msgs::msg::Marker::ADD;

    enemy_arrow.pose.position.x = enemy_state.x;
    enemy_arrow.pose.position.y = enemy_state.y;
    enemy_arrow.pose.position.z = 0.5;

    enemy_arrow.pose.orientation.z = qz;
    enemy_arrow.pose.orientation.w = qw;

    enemy_arrow.scale.x = 1.0;
    enemy_arrow.scale.y = 0.12;
    enemy_arrow.scale.z = 0.12;

    enemy_arrow.color.r = 0.0;
    enemy_arrow.color.g = 1.0;
    enemy_arrow.color.b = 1.0;
    enemy_arrow.color.a = 1.0;

    ma.markers.push_back(enemy_arrow);

    // =====================================================
    // STANICA
    // =====================================================

    m.ns = "station";
    m.id = 2000;

    m.type =
        visualization_msgs::msg::Marker::CYLINDER;

    m.pose.position.x = station_pos_.x;
    m.pose.position.y = station_pos_.y;
    m.pose.position.z = 0.05;

    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    m.scale.x = 2.0;
    m.scale.y = 2.0;
    m.scale.z = 0.1;

    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 0.5;

    ma.markers.push_back(m);

    // =====================================================
    // WORLD OBJECTS
    // =====================================================

    int id = 3000;

    for (auto& obj : world_objects_)
    {
        visualization_msgs::msg::Marker obj_marker;

        obj_marker.header.frame_id = "map";
        obj_marker.header.stamp = now;

        obj_marker.ns = "objects";
        obj_marker.id = id++;

        obj_marker.action =
            visualization_msgs::msg::Marker::ADD;

        obj_marker.pose.position.x = obj->x;
        obj_marker.pose.position.y = obj->y;
        obj_marker.pose.position.z = 0.3;

        obj_marker.pose.orientation.w = 1.0;

        obj_marker.color.a = 1.0;

        if (obj->getType() == "obstacle")
        {
            obj_marker.type =
                visualization_msgs::msg::Marker::CUBE;

            obj_marker.scale.x = 1.2;
            obj_marker.scale.y = 1.2;
            obj_marker.scale.z = 1.2;

            obj_marker.color.r = 1.0;
            obj_marker.color.g = 0.0;
            obj_marker.color.b = 0.0;
        }
        else
        {
            obj_marker.type =
                visualization_msgs::msg::Marker::SPHERE;

            obj_marker.scale.x = 0.5;
            obj_marker.scale.y = 0.5;
            obj_marker.scale.z = 0.5;

            if (obj->getColor() == "yellow")
            {
                obj_marker.color.r = 1.0;
                obj_marker.color.g = 1.0;
                obj_marker.color.b = 0.0;
            }
            else if (obj->getColor() == "blue")
            {
                obj_marker.color.r = 0.0;
                obj_marker.color.g = 0.0;
                obj_marker.color.b = 1.0;
            }
            else if (obj->getColor() == "green")
            {
                obj_marker.color.r = 0.0;
                obj_marker.color.g = 1.0;
                obj_marker.color.b = 0.0;
            }
        }

        ma.markers.push_back(obj_marker);
    }

    marker_array_pub_->publish(ma);
}
    
    void publish_enemy_path(
    const geometry::RobotState& state)
{
    geometry_msgs::msg::PoseStamped pose;

    pose.header.stamp = this->now();
    pose.header.frame_id = "map";

    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;

    double qz =
        std::sin(state.theta / 2.0);

    double qw =
        std::cos(state.theta / 2.0);

    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;

    enemy_path_.header.stamp = this->now();

    enemy_path_.poses.push_back(pose);

    enemy_path_pub_->publish(enemy_path_);
}
    
  
    
void publish_game_state_string()
{
    std_msgs::msg::String msg;

    msg.data =
        "P1 SCORE: "
        + std::to_string(player1_score_)
        + " | P2 SCORE: "
        + std::to_string(player2_score_)
        + " | P1 LOAD: "
        + std::to_string(player1_load_)
        + "/"
        + std::to_string(max_capacity_)
        + " | P2 LOAD: "
        + std::to_string(player2_load_)
        + "/"
        + std::to_string(max_capacity_);

    game_state_pub_string_->publish(msg);
}
    
    void publish_static_map()
    {
        auto msg =
            nav_msgs::msg::OccupancyGrid();

        msg.header.frame_id = "map";
        msg.header.stamp = this->now();

        msg.info.resolution = 0.05;

        cv::Mat map =
            cv::imread(
                ament_index_cpp::
                get_package_share_directory(
                    "vacuum_game")
                + "/map/opk-map.png",
                0);

        if (map.empty())
            return;

        msg.info.width = map.cols;
        msg.info.height = map.rows;

        msg.info.origin.position.x =
            -(map.cols * 0.05) / 2.0;

        msg.info.origin.position.y =
            -(map.rows * 0.05) / 2.0;

        msg.info.origin.orientation.w = 1.0;

        msg.data.resize(map.cols * map.rows);

        for (int y = 0; y < map.rows; y++)
        {
            for (int x = 0; x < map.cols; x++)
            {
                int ros_y =
                    map.rows - 1 - y;

                msg.data[
                    ros_y * map.cols + x] =
                    (map.at<uchar>(y, x) < 127)
                    ? 100
                    : 0;
            }
        }

        map_pub_->publish(msg);
    }

    std::shared_ptr<environment::Environment>
        env_ptr_;

    std::unique_ptr<lidar::Lidar>
        lidar_ptr_;

    std::unique_ptr<robot::Robot>
        my_robot_;
    std::unique_ptr<robot::Robot>
        enemy_robot_;
    
    std::string game_mode_;
    
    std::vector<std::unique_ptr<WorldObject>>
        world_objects_;

    std::map<std::string, int>
        player1_inventory_;

    std::map<std::string, int>
        player2_inventory_;

    struct
    {
        double x, y;
    } station_pos_;

    int player1_load_;
    int player2_load_;
    int max_capacity_;

    rclcpp::TimerBase::SharedPtr
        timer_;

    rclcpp::TimerBase::SharedPtr
        map_timer_;
        
    rclcpp::Publisher
    <std_msgs::msg::String>::SharedPtr
        game_state_pub_string_;

    rclcpp::Publisher
    <visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_array_pub_;

    rclcpp::Publisher
    <visualization_msgs::msg::Marker>::SharedPtr
        lidar_pub_;
    rclcpp::Publisher
    <sensor_msgs::msg::LaserScan>::SharedPtr
        scan_pub_;
    
    rclcpp::Subscription
    <geometry_msgs::msg::Twist>::SharedPtr
        enemy_sub_;
    
    rclcpp::Publisher
    <nav_msgs::msg::OccupancyGrid>::SharedPtr
        map_pub_;
    rclcpp::Publisher
    <nav_msgs::msg::Odometry>::SharedPtr
        odom_pub_;
    rclcpp::Publisher
    <nav_msgs::msg::Odometry>::SharedPtr
        enemy_odom_pub_;
    rclcpp::Service
    <std_srvs::srv::Empty>::SharedPtr
        reset_service_;

    rclcpp::Publisher
    <nav_msgs::msg::Path>::SharedPtr
        path_pub_;
        
    rclcpp::Publisher
    <nav_msgs::msg::Path>::SharedPtr
        enemy_path_pub_;
    
    nav_msgs::msg::Path enemy_path_;
    
    nav_msgs::msg::Path
        robot_path_;
    
    bool game_finished_ = false;

    rclcpp::Time game_start_time_;

    int game_duration_seconds_ = 120;
    
    rclcpp::Subscription
    <geometry_msgs::msg::Twist>::SharedPtr
        sub_;

    std::shared_ptr
    <tf2_ros::StaticTransformBroadcaster>
        tf_static_broadcaster_;
    double trash_radius_min_;
    double trash_radius_max_;
    int world_version_ = 0;
    int player1_score_ = 0;
    int player2_score_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<VacuumNode>());

    rclcpp::shutdown();

    return 0;
}
