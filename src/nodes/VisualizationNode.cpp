#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/pose.hpp"

class VisualizationNode : public rclcpp::Node
{
public:

    VisualizationNode()
        : Node("visualization_node")
    {
        marker_pub_ =
            this->create_publisher
            <visualization_msgs::msg::MarkerArray>(
                "visualization_markers",
                10);

        odom_sub_ =
            this->create_subscription
            <nav_msgs::msg::Odometry>(
                "odom",
                10,
                std::bind(
                    &VisualizationNode::odomCallback,
                    this,
                    std::placeholders::_1));

        enemy_sub_ =
            this->create_subscription
            <nav_msgs::msg::Odometry>(
                "/robot2/odom",
                10,
                std::bind(
                    &VisualizationNode::enemyCallback,
                    this,
                    std::placeholders::_1));

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(30),
                std::bind(
                    &VisualizationNode::publishMarkers,
                    this));
    }

private:

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        player_pose_ = msg->pose.pose;
    }

    void enemyCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        enemy_pose_ = msg->pose.pose;
    }

    void publishMarkers()
    {
        visualization_msgs::msg::MarkerArray ma;

        auto now = this->now();

        // =====================================================
        // PLAYER
        // =====================================================

        visualization_msgs::msg::Marker player;

        player.header.frame_id = "map";
        player.header.stamp = now;

        player.ns = "player";
        player.id = 0;

        player.type =
            visualization_msgs::msg::Marker::CYLINDER;

        player.action =
            visualization_msgs::msg::Marker::ADD;

        player.pose = player_pose_;

        player.pose.position.z = 0.2;

        player.scale.x = 0.8;
        player.scale.y = 0.8;
        player.scale.z = 0.4;

        player.color.r = 1.0;
        player.color.g = 0.5;
        player.color.b = 0.0;
        player.color.a = 1.0;

        ma.markers.push_back(player);

        // =====================================================
        // PLAYER ARROW
        // =====================================================

        visualization_msgs::msg::Marker arrow;

        arrow.header.frame_id = "map";
        arrow.header.stamp = now;

        arrow.ns = "arrow";
        arrow.id = 1;

        arrow.type =
            visualization_msgs::msg::Marker::ARROW;

        arrow.action =
            visualization_msgs::msg::Marker::ADD;

        arrow.pose = player_pose_;

        arrow.pose.position.z = 0.5;

        arrow.scale.x = 1.0;
        arrow.scale.y = 0.15;
        arrow.scale.z = 0.15;

        arrow.color.r = 1.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;

        ma.markers.push_back(arrow);

        // =====================================================
        // ENEMY
        // =====================================================

        visualization_msgs::msg::Marker enemy;

        enemy.header.frame_id = "map";
        enemy.header.stamp = now;

        enemy.ns = "enemy";
        enemy.id = 2;

        enemy.type =
            visualization_msgs::msg::Marker::CYLINDER;

        enemy.action =
            visualization_msgs::msg::Marker::ADD;

        enemy.pose = enemy_pose_;

        enemy.pose.position.z = 0.2;

        enemy.scale.x = 0.8;
        enemy.scale.y = 0.8;
        enemy.scale.z = 0.4;

        enemy.color.r = 0.0;
        enemy.color.g = 0.3;
        enemy.color.b = 1.0;
        enemy.color.a = 1.0;

        ma.markers.push_back(enemy);

        // =====================================================
        // ENEMY ARROW
        // =====================================================

        visualization_msgs::msg::Marker enemy_arrow;

        enemy_arrow.header.frame_id = "map";
        enemy_arrow.header.stamp = now;

        enemy_arrow.ns = "enemy_arrow";
        enemy_arrow.id = 3;

        enemy_arrow.type =
            visualization_msgs::msg::Marker::ARROW;

        enemy_arrow.action =
            visualization_msgs::msg::Marker::ADD;

        enemy_arrow.pose = enemy_pose_;

        enemy_arrow.pose.position.z = 0.5;

        enemy_arrow.scale.x = 1.0;
        enemy_arrow.scale.y = 0.15;
        enemy_arrow.scale.z = 0.15;

        enemy_arrow.color.r = 0.0;
        enemy_arrow.color.g = 1.0;
        enemy_arrow.color.b = 1.0;
        enemy_arrow.color.a = 1.0;

        ma.markers.push_back(enemy_arrow);

        marker_pub_->publish(ma);
    }

    geometry_msgs::msg::Pose player_pose_;
    geometry_msgs::msg::Pose enemy_pose_;

    rclcpp::Publisher
    <visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub_;

    rclcpp::Subscription
    <nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    rclcpp::Subscription
    <nav_msgs::msg::Odometry>::SharedPtr
        enemy_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<VisualizationNode>());

    rclcpp::shutdown();

    return 0;
}
