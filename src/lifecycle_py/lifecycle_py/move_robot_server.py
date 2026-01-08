 #!/usr/bin/env python3
 
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import MoveRobot
import time
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class MoveRobotServerNode(LifecycleNode):

    def __init__(self):
        super().__init__("move_robot_server")
        self.goal_lock = threading.Lock()
        self.goal_handle_ : ServerGoalHandle = None
        self.robot_position_ = 50
        self.server_activated_ = False
        self.get_logger().info("Robot position: "+ str(self.robot_position_))

    def on_configure(self, previous_state: LifecycleNode):
        self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        self.robot_name_ = self.get_parameter("robot_name").value
        self.get_logger().info("In on_configure")
        self.move_robot_server_ = ActionServer(
            self,
            MoveRobot,
            "move_robot_" + self.robot_name_,
            goal_callback = self.goal_callback,
            cancel_callback= self.cancel_callback,
            execute_callback= self.execute_callback,
            callback_group= ReentrantCallbackGroup()
            )
        self.get_logger().info("Move Robot action server has started")
        return TransitionCallbackReturn.SUCCESS
    
    # Destroy ROS2 communication, disconnect from HW
    def on_cleanup(self, previous_state: LifecycleNode):
        self.undeclare_parameter("robot_name")
        self.robot_name_=""
        self.get_logger().info("In on_cleanup")
        #self.destroy_lifecycle_publisher(self.move_robot_server_)
        self.move_robot_server_.destroy()
        return TransitionCallbackReturn.SUCCESS
    
     # Acticate/enable HW
    def on_activate(self, previous_state: LifecycleNode):
        self.get_logger().info("In on_activate")
        self.server_activated_ = True
        return super().on_activate(previous_state)
    
    #Deactivate/diable HW
    def on_deactivate(self, previous_state: LifecycleNode):
        self.get_logger().info("In on_deactivate")
        self.server_activated_ = False
        with self.goal_lock:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.goal_handle_.abort()
        return super().on_deactivate(previous_state)
    
    # Shutdown
    def on_shutdown(self, previou_state: LifecycleNode):
        self.undeclare_parameter("robot_name")
        self.robot_name_=""
        self.get_logger().info("In on_shutdown")
        #self.destroy_lifecycle_publisher(self.move_robot_server_)
        self.move_robot_server_.destroy()
        return TransitionCallbackReturn.SUCCESS
    


    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Recieved a new goal")

        if not self.server_activated_:
            self.get_logger().warn("Node not activated yet.")
            return GoalResponse.REJECT
        
        if goal_request.position not in range(0, 100) or goal_request.velocity <= 0:
            self.get_logger().warn("Invalid position or velocity, reject goal")
            return GoalResponse.REJECT
        
        #new goal if valid abrot previos goal and acept new goal
        with self.goal_lock:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.goal_handle_.abort()

        self.get_logger().info("Accept goal")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().warn("Recieved a cancel request")
        return CancelResponse.ACCEPT



    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock:
            self.goal_handle_ = goal_handle

        
        goal_position = goal_handle.request.position
        velocity = goal_handle.request.velocity

        result = MoveRobot.Result()
        feedback = MoveRobot.Feedback()

        self.get_logger().info("Execute goal")

        while rclpy.ok():   # if we terminate the node by pressing ctrl + C , then loop will be terminated
            
            if not goal_handle.is_active:
                result.position = self.robot_position_
                result.message = "Preempted by another goal"
                return result
            
            if goal_handle.is_cancel_requested:
                result.position = self.robot_position_
                if goal_position == self.robot_position_:
                    result.message = "Success"
                    goal_handle.succeed()
                else:
                    result.message = "Canceled"
                    goal_handle.canceled()
                return result


            diff = goal_position - self.robot_position_

            if diff == 0:
                result.position = self.robot_position_
                result.message = "Success"
                goal_handle.succeed()
                return result
            if diff > 0:
                if diff >= velocity:
                    self.robot_position_ += velocity
                else:
                    self.robot_position_ += diff               
            else:
                if abs(diff) >= velocity:
                    self.robot_position_ -= velocity
                else:
                    self.robot_position_ -= abs(diff)

            self.get_logger().info("Robot position: " + str(self.robot_position_))
            feedback.current_position = self.robot_position_
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0)
    

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == '__main__':
    main()