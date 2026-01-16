#include "dobo_hardware/arm_hardware_interface.hpp"

namespace arm_hardware{


// hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init
//     (const hardware_interface::HardwareInfo & info)
hardware_interface::CallbackReturn
    ArmHardwareInterface::on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params)

    {
        if (hardware_interface::SystemInterface::on_init(params) !=  // info -> params
            hardware_interface::CallbackReturn::SUCCESS)
            {
                return hardware_interface::CallbackReturn::ERROR;
            }
        // info_ = info;

        // left_motor_id_ = 10;
        // right_motor_id_= 20;
        // port_ = "/dev/tty/ACM0";

        // Get HardwareInfo
        const auto & info_ = params.hardware_info;
        // Read parameters from ros2_control
        joint1_motor_id_  = std::stoi(info_.hardware_parameters.at("joint1_motor_id"));
        joint2_motor_id_ = std::stoi(info_.hardware_parameters.at("joint2_motor_id"));
        port_           = info_.hardware_parameters.at("port");

        driver_ = std::make_shared<XL330Driver>(port_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

hardware_interface::CallbackReturn ArmHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        if (driver_->init() != 0){
            return hardware_interface::CallbackReturn::ERROR;
        }


        for (const auto & [name, descr] : joint_command_interfaces_)
        {
            RCLCPP_INFO(get_logger(), "COMMAND INTERFACE NAME: ");
            RCLCPP_INFO(get_logger(), name.c_str());
        }
        for (const auto & [name, descr] : joint_state_interfaces_)
        {
            RCLCPP_INFO(get_logger(), "STATE INTERFACE NAME: ");
            RCLCPP_INFO(get_logger(), name.c_str());
        }


        return hardware_interface::CallbackReturn::SUCCESS;
    }

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        
        driver_->activateWithPositionMode(joint1_motor_id_);
        driver_->activateWithPositionMode(joint2_motor_id_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
    {
        (void)previous_state;
        driver_->deactivate(joint1_motor_id_);
        driver_->deactivate(joint2_motor_id_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

hardware_interface::return_type ArmHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        (void)period;

        set_state("arm_joint1/position", driver_->getPositionRadian(joint1_motor_id_)); 
        set_state("arm_joint2/position", driver_->getPositionRadian(joint2_motor_id_));        
       
        RCLCPP_INFO(get_logger(), "STATE joint 1: %lf, joint 2: %lf", 
             get_state("arm_joint1/position"), get_state("arm_joint2/position"));

        return hardware_interface::return_type::OK;
    }

hardware_interface::return_type ArmHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        (void)period;
        driver_->setTargetPositionRadian(joint1_motor_id_, 
            get_command("arm_joint1/position"));
        driver_->setTargetPositionRadian(joint2_motor_id_, 
            get_command("arm_joint2/position")); 

        RCLCPP_INFO(get_logger(), "COMMAND joint 1: %lf, joint 2: %lf", 
             get_command("arm_joint1/position"), get_command("arm_joint2/position"));

        return hardware_interface::return_type::OK;
    }


} //namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, 
                        hardware_interface::SystemInterface)

