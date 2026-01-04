#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

class RobotNewsStationNode : public rclcpp::Node
{
    public:
        RobotNewsStationNode() : Node("robot_news_station"), robot_name("R2D")
        {
            publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news",10);
            timer_ = this->create_wall_timer(0.5s,std::bind(&RobotNewsStationNode::publishNews,this));
            RCLCPP_INFO(this->get_logger(), "Robot News station has strated");
        }


    private:
        void publishNews()
        {
            auto msg = example_interfaces::msg::String();
            msg.data = std::string("Hi.this is ")+ robot_name  ;
            publisher_ ->publish(msg);
        }
        std::string robot_name;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // initialize ros2 communication
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}