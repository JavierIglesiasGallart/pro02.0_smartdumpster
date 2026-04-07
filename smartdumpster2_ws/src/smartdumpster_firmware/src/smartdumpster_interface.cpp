#include <smartdumpster_firmware/smartdumpster_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace smartdumpster_firmware
{
    SmartdumpsterInterface::SmartdumpsterInterface()
    {
        kp_ = 0.0;
        ki_ = 0.0;
        kd_ = 0.0;
        l_error_integral_ = 0.0;
        r_error_integral_ = 0.0;
        l_prev_error_ = 0.0;
        r_prev_error_ = 0.0;
        l_prev_output_ = 0.0;
        r_prev_output_ = 0.0;
        pid_enable_ = false;
    }

    SmartdumpsterInterface::~SmartdumpsterInterface()
    {
        
    }

    CallbackReturn SmartdumpsterInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);

        if(result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        this->info_ = hardware_info.hardware_info;

        velocity_commands_.assign(info_.joints.size(), 0.0);
        position_states_.assign(info_.joints.size(), 0.0);
        velocity_states_.assign(info_.joints.size(), 0.0);
        last_run_ = rclcpp::Clock().now();

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

        auto it_kp = info_.hardware_parameters.find("kp");
        if (it_kp != info_.hardware_parameters.end()) kp_ = std::stod(it_kp->second);

        auto it_ki = info_.hardware_parameters.find("ki");
        if (it_ki != info_.hardware_parameters.end()) ki_ = std::stod(it_ki->second);

        auto it_kd = info_.hardware_parameters.find("kd");
        if (it_kd != info_.hardware_parameters.end()) kd_ = std::stod(it_kd->second);

        auto it_pid_enable = info_.hardware_parameters.find("pid_enable");
        if (it_pid_enable != info_.hardware_parameters.end()) pid_enable_ = (it_pid_enable->second == "true");

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> SmartdumpsterInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
                hardware_interface::HW_IF_POSITION, &position_states_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, 
                hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> SmartdumpsterInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, 
                hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn SmartdumpsterInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("SmartdumpserInterface"), "Starting robot hardware...");

        velocity_commands_ = {0.0, 0.0};
        position_states_ = {0.0, 0.0};
        velocity_states_ = {0.0, 0.0};

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

        PhidgetDCMotor_setTargetVelocity(engine_[L_ENGINE], 0.0);
        PhidgetDCMotor_setTargetVelocity(engine_[R_ENGINE], 0.0);
        
        RCLCPP_INFO(rclcpp::get_logger("SmartdumpserInterface"), "Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }
            
    CallbackReturn SmartdumpsterInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("SmartdumpserInterface"), "Stopping robot hardware...");

        PhidgetDCMotor_setTargetVelocity(engine_[L_ENGINE], 0.0);
        PhidgetDCMotor_setTargetVelocity(engine_[R_ENGINE], 0.0);

        try
        {
            phidgetReturn_ = Phidget_close((PhidgetHandle)engine_[L_ENGINE]);
            if (phidgetReturn_ != EPHIDGET_OK)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "Left Engine fail in close operation....");
            }

            phidgetReturn_ = Phidget_close((PhidgetHandle)engine_[R_ENGINE]);
            if (phidgetReturn_ != EPHIDGET_OK)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "Right Engine fail in close operation....");
            }

            phidgetReturn_ = Phidget_close((PhidgetHandle)encoder_[L_ENGINE]);
            if (phidgetReturn_ != EPHIDGET_OK)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "Left Encoder fail in close operation....");
            }

            phidgetReturn_ = Phidget_close((PhidgetHandle)encoder_[R_ENGINE]);
            if (phidgetReturn_ != EPHIDGET_OK)
            {
                RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "Right Encoder fail in close operation....");
            }
        }
        catch(...)
        {
            RCLCPP_FATAL(rclcpp::get_logger("SmartdumpserInterface"), "Something went wrong while close hardware...");
            return CallbackReturn::FAILURE;
        }

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type SmartdumpsterInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        auto dt = (rclcpp::Clock().now() - last_run_).seconds();
        int64_t l_counts = 0;
        int64_t r_counts = 0;
        const double CPR = 5400.0; // 300 * 18

        PhidgetEncoder_getPosition(encoder_[L_ENGINE], &l_counts);
        PhidgetEncoder_getPosition(encoder_[R_ENGINE], &r_counts);

        double l_pos_rad = (static_cast<double>(-l_counts) / CPR) * 2.0 * 3.14;
        double r_pos_rad = (static_cast<double>(r_counts) / CPR) * 2.0 * 3.14;

        velocity_states_[L_ENGINE] = (l_pos_rad - position_states_[L_ENGINE]) / dt;
        velocity_states_[R_ENGINE] = (r_pos_rad - position_states_[R_ENGINE]) / dt;

        position_states_[L_ENGINE] = l_pos_rad;
        position_states_[R_ENGINE] = r_pos_rad;

        last_run_ = rclcpp::Clock().now();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SmartdumpsterInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        const double MAX_RAD_S = 18.32;
        double dt = period.seconds();

        double l_vel_setpoint = velocity_commands_[L_ENGINE];
        double r_vel_setpoint = velocity_commands_[R_ENGINE];

        if(pid_enable_)     // ----- LAZO CERRADO -----
        {
            double l_vel_feedback = velocity_states_[L_ENGINE];
            double r_vel_feedback = velocity_states_[R_ENGINE];

            // --- PID MOTOR IZQUIERDO ---
            double l_error = l_vel_setpoint - l_vel_feedback;
            
            if (std::abs(l_prev_output_) < 1.0 || (l_error * l_prev_output_ < 0)) {
                l_error_integral_ += l_error * dt;
            }
            
            double l_derivative = (l_error - l_prev_error_) / dt;
            double l_output = (kp_ * l_error) + (ki_ * l_error_integral_) + (kd_ * l_derivative);
            
            if (std::abs(l_vel_setpoint) < 0.1) l_error_integral_ = 0.0;

            // --- PID MOTOR DERECHO ---
            double r_error = r_vel_setpoint - r_vel_feedback;
            
            if (std::abs(r_prev_output_) < 1.0 || (r_error * r_prev_output_ < 0)) {
                r_error_integral_ += r_error * dt;
            }
            
            double r_derivative = (r_error - r_prev_error_) / dt;
            double r_output = (kp_ * r_error) + (ki_ * r_error_integral_) + (kd_ * r_derivative);
            
            if (std::abs(r_vel_setpoint) < 0.1) r_error_integral_ = 0.0;

            l_prev_error_ = l_error;
            r_prev_error_ = r_error;
            
            l_prev_output_ = std::max(-1.0, std::min(1.0, l_output));
            r_prev_output_ = std::max(-1.0, std::min(1.0, r_output));

            if (std::abs(r_output) < 0.1) r_prev_output_ = 0.0;
            if (std::abs(l_output) < 0.1) l_prev_output_ = 0.0;
        }
        else // ----- LAZO CERRADO -----
        {
            l_prev_output_ = std::max(-1.0, std::min(1.0, l_vel_setpoint / MAX_RAD_S));
            r_prev_output_ = std::max(-1.0, std::min(1.0, r_vel_setpoint / MAX_RAD_S));
        }


        PhidgetDCMotor_setTargetVelocity(engine_[L_ENGINE], l_prev_output_);
        PhidgetDCMotor_setTargetVelocity(engine_[R_ENGINE], -r_prev_output_);

        return hardware_interface::return_type::OK;
    }

    
    // --------------------- PHIDGET LIBRARY FCN ---------------------
    void CCONV SmartdumpsterInterface::onEngineL_Attach(PhidgetHandle ch, void * ctx)
    {
        RCLCPP_INFO(rclcpp::get_logger("SmartdumpserInterface"), "Engine left connected, ready to use!");
    }

    void CCONV SmartdumpsterInterface::onEngineR_Attach(PhidgetHandle ch, void * ctx)
    {
        RCLCPP_INFO(rclcpp::get_logger("SmartdumpserInterface"), "Engine right connected, ready to use!");
    }

    void CCONV SmartdumpsterInterface::onEncoderL_Attach(PhidgetHandle ch, void * ctx)
    {
        RCLCPP_INFO(rclcpp::get_logger("SmartdumpserInterface"), "Encoder left connected, ready to use!");
    }

    void CCONV SmartdumpsterInterface::onEncoderR_Attach(PhidgetHandle ch, void * ctx)
    {
        RCLCPP_INFO(rclcpp::get_logger("SmartdumpserInterface"), "Encoder right connected, ready to use!");
    }

    void CCONV SmartdumpsterInterface::onEngineL_Detach(PhidgetHandle ch, void * ctx)
    {
        RCLCPP_WARN(rclcpp::get_logger("SmartdumpserInterface"), "Engine left disconnected!");
    }

    void CCONV SmartdumpsterInterface::onEngineR_Detach(PhidgetHandle ch, void * ctx)
    {
        RCLCPP_WARN(rclcpp::get_logger("SmartdumpserInterface"), "Engine right disconnected!");
    }

    void CCONV SmartdumpsterInterface::onEncoderL_Detach(PhidgetHandle ch, void * ctx)
    {
        RCLCPP_WARN(rclcpp::get_logger("SmartdumpserInterface"), "Encoder left disconnected!");
    }

    void CCONV SmartdumpsterInterface::onEncoderR_Detach(PhidgetHandle ch, void * ctx)
    {
        RCLCPP_WARN(rclcpp::get_logger("SmartdumpserInterface"), "Encoder right disconnected!");
    }

}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smartdumpster_firmware::SmartdumpsterInterface, hardware_interface::SystemInterface);
