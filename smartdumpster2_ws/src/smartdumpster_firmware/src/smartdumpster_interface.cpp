#include <smartdumpster_firmware/smartdumpster_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace smartdumpster_interface
{
    SmartdumpsterInterface::SmartdumpsterInterface()
    {

    }

    SmartdumpsterInterface::~SmartdumpsterInterface()
    {
        
    }

    CallbackReturn SmartdumpsterInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);

        if(result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        velocity_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());
        engine_.reserve(info_.joints.size());
        encoder_.reserve(info_.joints.size());

        PhidgetDCMotor_create(&engine_[L_ENGINE]);
        PhidgetDCMotor_create(&engine_[R_ENGINE]);
	    PhidgetEncoder_create(&encoder_[L_ENGINE]);
        PhidgetEncoder_create(&encoder_[R_ENGINE]);

        Phidget_setOnAttachHandler((PhidgetHandle)engine_[L_ENGINE], SmartdumpsterInterface::onEngineL_Attach, NULL);
        Phidget_setOnAttachHandler((PhidgetHandle)engine_[R_ENGINE], SmartdumpsterInterface::onEngineR_Attach, NULL);
        Phidget_setOnAttachHandler((PhidgetHandle)encoder_[L_ENGINE], SmartdumpsterInterface::onEncoderL_Attach, NULL);
        Phidget_setOnAttachHandler((PhidgetHandle)encoder_[R_ENGINE], SmartdumpsterInterface::onEncoderR_Attach, NULL);

        Phidget_setOnDetachHandler((PhidgetHandle)engine_[L_ENGINE], SmartdumpsterInterface::onEngineL_Detach, NULL);
        Phidget_setOnDetachHandler((PhidgetHandle)engine_[R_ENGINE], SmartdumpsterInterface::onEngineR_Detach, NULL);
        Phidget_setOnDetachHandler((PhidgetHandle)encoder_[L_ENGINE], SmartdumpsterInterface::onEncoderL_Detach, NULL);
        Phidget_setOnDetachHandler((PhidgetHandle)encoder_[R_ENGINE], SmartdumpsterInterface::onEncoderR_Detach, NULL);

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> SmartdumpsterInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, 
                hardware_interface::HW_IF_POSITION, &position_states_.at(i)));

            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, 
                hardware_interface::HW_IF_VELOCITY, &velocity_states_.at(i)));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> SmartdumpsterInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints.at(i).name, 
                hardware_interface::HW_IF_VELOCITY, &velocity_commands_.at(i)));
        }

        return command_interfaces;
    }

    CallbackReturn SmartdumpsterInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("SmartdumpserInterface"), "Starting robot hardware...");

        velocity_commands_ = {0.0, 0.0, 0.0, 0.0};
        position_states_ = {0.0, 0.0, 0.0, 0.0};
        velocity_states_ = {0.0, 0.0, 0.0, 0.0};

        try
        {
            phidgetReturn_ = Phidget_setHubPort((PhidgetHandle)engine_[L_ENGINE], L_ENGINE);
            if (phidgetReturn_ != EPHIDGET_OK)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "No left Hub Port configured! Aborting");
            }

            phidgetReturn_ = Phidget_setHubPort((PhidgetHandle)engine_[R_ENGINE], R_ENGINE);
            if (phidgetReturn_ != EPHIDGET_OK)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "No right Hub Port configured! Aborting");
            }
        }
        catch(...)
        {
            RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "Something went wrong while interacting with HUB");
            return CallbackReturn::FAILURE;
        }

        try
        {
            phidgetReturn_ = Phidget_openWaitForAttachment((PhidgetHandle)engine_[L_ENGINE], ATTACH_TIMEOUT);
            if (phidgetReturn_ != EPHIDGET_OK)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "Left Engine not attach");
            }

            phidgetReturn_ = Phidget_openWaitForAttachment((PhidgetHandle)engine_[R_ENGINE], ATTACH_TIMEOUT);
            if (phidgetReturn_ != EPHIDGET_OK)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "Right Engine not attach");
            }

            phidgetReturn_ = Phidget_openWaitForAttachment((PhidgetHandle)encoder_[L_ENGINE], ATTACH_TIMEOUT);
            if (phidgetReturn_ != EPHIDGET_OK)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "Left Encoder not attach");
            }

            phidgetReturn_ = Phidget_openWaitForAttachment((PhidgetHandle)encoder_[R_ENGINE], ATTACH_TIMEOUT);
            if (phidgetReturn_ != EPHIDGET_OK)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "Right Encoder not attach");
            }
        }
        catch(...)
        {
            RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "Something went wrong while attach hardware...");
            return CallbackReturn::FAILURE;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("SmartdumpserInterface"), "Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }
            
    CallbackReturn SmartdumpsterInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {

    }

}