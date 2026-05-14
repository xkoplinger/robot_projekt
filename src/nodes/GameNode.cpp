#include "rclcpp/rclcpp.hpp"

class GameNode : public rclcpp::Node
{
public:

    GameNode()
        : Node("game_node")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Game node started");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<GameNode>());

    rclcpp::shutdown();

    return 0;
}

