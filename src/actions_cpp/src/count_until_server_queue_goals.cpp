#include <queue>
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
            goal_queue_thread_ = std::thread(&CountUntilServerNode::run_goal_queue_thread, this);
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

         ~CountUntilServerNode()
         {
            goal_queue_thread_.join();
         }
        

    private:

        rclcpp_action::GoalResponse goal_callback(
            const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
        {   
            (void)uuid;
            RCLCPP_INFO(this->get_logger(),"Recieved a goal"); 

            //Policy: refuce new goal if one goal is being active
            // {
            //     std::lock_guard<std::mutex> lock(mutex_);
            //     if (goal_handle_){
            //         if (goal_handle_->is_active()){
            //             RCLCPP_INFO(this->get_logger(),"A goal is still active, Rejecting the new goal"); 
            //             return rclcpp_action::GoalResponse::REJECT;
            //         }
            //     }
            // }

            if (goal->target_number <= 0){
                RCLCPP_INFO(this->get_logger(),"Rejecting the goal"); 
                return rclcpp_action::GoalResponse::REJECT;
            }

            //Policy: preempt existing goal when receiving a new valid goal
            // {
            //     std::lock_guard<std::mutex> lock(mutex_);
            //     if (goal_handle_){
            //         if (goal_handle_->is_active()){
            //             RCLCPP_INFO(this->get_logger(),"Aborting current goal and accept new goal"); 
            //             preempted_goal_id_ = goal_handle_->get_goal_id();

            //         }
            //     }
            // }
            

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
            // RCLCPP_INFO(this->get_logger(),"Executing the goal"); 
            // execute_goal(goal_handle);
            std::lock_guard<std::mutex> lock(mutex_);
            goal_queue_.push(goal_handle);
            RCLCPP_INFO(this->get_logger(),"Add goal to the queue"); 
            RCLCPP_INFO(this->get_logger(),"Queue size: %d", (int)goal_queue_.size()); 
        }

        void run_goal_queue_thread()
        {
            rclcpp::Rate loop_rate(1000.0);
            while (rclcpp::ok()){
                std::shared_ptr<CountUntilGoalHandle> next_goal;
                {       
                    std::lock_guard<std::mutex> lock(mutex_);          
                    if (goal_queue_.size()>0){
                        next_goal = goal_queue_.front();
                        goal_queue_.pop();
                    }
                }
                if (next_goal){
                    RCLCPP_INFO(this->get_logger(),"Execute next goal in the queue"); 
                    execute_goal(next_goal);
                }
                loop_rate.sleep();
            }
        }

        void execute_goal(
            const std::shared_ptr<CountUntilGoalHandle> goal_handle)
        {   
            // {
            //     std::lock_guard<std::mutex> lock(mutex_);
            //     this->goal_handle_ = goal_handle;
            // }

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
                // { // this is for preepting policy
                //     std::lock_guard<std::mutex> lock(mutex_);
                //     if (goal_handle->get_goal_id() == preempted_goal_id_){
                //         result->reached_number = counter;
                //         goal_handle->abort(result);
                //         return;
                //     }
                // }
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
        std::mutex mutex_;
        std::shared_ptr<CountUntilGoalHandle> goal_handle_;
        // rclcpp_action::GoalUUID preempted_goal_id_;
        std::queue<std::shared_ptr<CountUntilGoalHandle>> goal_queue_;
        std::thread goal_queue_thread_;
        
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