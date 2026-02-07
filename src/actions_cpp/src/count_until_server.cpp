#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
using namespace std::placeholders;
 
class CountUntilServerNode : public rclcpp::Node
{
    public:
        CountUntilServerNode() : Node("count_until_server")
        {
            cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            count_until_server_ = rclcpp_action::create_server<CountUntil>(
                this,
                "count_until",
                std::bind(&CountUntilServerNode::goal_callback, this, _1, _2),
                std::bind(&CountUntilServerNode::cancel_callback, this, _1),
                std::bind(&CountUntilServerNode::handle_accepted_callback, this, _1),
                rcl_action_server_get_default_options(),
                cb_group_
                 );
            RCLCPP_INFO(this->get_logger(),"Action server ahs been started"); 

         }


    private:

        rclcpp_action::GoalResponse goal_callback(
            const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
        {   
            (void)uuid;
            RCLCPP_INFO(this->get_logger(),"Recieved a goal"); 
            if (goal->target_number <= 0){
                RCLCPP_INFO(this->get_logger(),"Rejecting the goal"); 
                return rclcpp_action::GoalResponse::REJECT;
            }
            RCLCPP_INFO(this->get_logger(),"Accepting the goal"); 
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancel_callback(
            const std::shared_ptr<CountUntilGoalHandle> goal_handle)
        {   
            RCLCPP_INFO(this->get_logger(),"Recieved a cancel request"); 
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted_callback(
            const std::shared_ptr<CountUntilGoalHandle> goal_handle)
        {   
            RCLCPP_INFO(this->get_logger(),"Executing the goal"); 
            execute_goal(goal_handle);
        }

        void execute_goal(
            const std::shared_ptr<CountUntilGoalHandle> goal_handle)
        {
            //get request from goal
            int target_number = goal_handle->get_goal()->target_number;
            double period = goal_handle->get_goal()->period;

            //execute the action
            int counter = 0;
            auto result = std::make_shared<CountUntil::Result>();
            auto feedback = std::make_shared<CountUntil::Feedback>();
            rclcpp::Rate loop_rate(1.0/period);  // loops run in periodically
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            for (int i = 0 ;i < target_number; i++){
                if (goal_handle->is_canceling()){
                    result->reached_number = counter;
                    goal_handle->canceled(result);
                    return;
                }
                counter++;
                RCLCPP_INFO(this->get_logger(),"%d", counter); 
                feedback->current_number = counter;
                goal_handle->publish_feedback(feedback);
                loop_rate.sleep();
            }

            //set final state and return result
            result->reached_number = counter;
            goal_handle->succeed(result);
            // goal_handle->abort(result);


        }


        rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_;
        rclcpp::CallbackGroup::SharedPtr cb_group_;
        
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // initialize ros2 communication
    auto node = std::make_shared<CountUntilServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}