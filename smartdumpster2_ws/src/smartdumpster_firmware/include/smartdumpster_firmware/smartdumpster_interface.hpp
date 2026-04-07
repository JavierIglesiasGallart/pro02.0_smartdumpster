#ifndef SMARTDUMPSTER_INTERFACE_HPP
#define SMARTDUMPSTER_INTERFACE_HPP


#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/system_interface.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <vector>

#include <phidget22.h>
#include <stdio.h>


namespace smartdumpster_firmware
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class SmartdumpsterInterface : public hardware_interface::SystemInterface
    {
        public:
            SmartdumpsterInterface();
            virtual ~SmartdumpsterInterface();

            virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
            
            virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

            virtual CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & hardware_info) override;

            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            const int L_ENGINE = 0;
            const int R_ENGINE = 1;
            const int ATTACH_TIMEOUT = 5000; //ms

            std::vector<int> port_;
            std::vector<double> velocity_commands_;
            std::vector<double> position_states_;
            std::vector<double> velocity_states_;

            rclcpp::Time last_run_;

            double kp_, ki_, kd_;
            bool pid_enable_;
            double l_error_integral_, r_error_integral_;
            double l_prev_error_, r_prev_error_;
            double l_prev_output_, r_prev_output_;

            std::vector<PhidgetDCMotorHandle> engine_;
	        std::vector<PhidgetEncoderHandle> encoder_;

            PhidgetReturnCode phidgetReturn_;

            static void CCONV onEngineL_Attach(PhidgetHandle ch, void * ctx);
            static void CCONV onEngineR_Attach(PhidgetHandle ch, void * ctx);
            static void CCONV onEncoderL_Attach(PhidgetHandle ch, void * ctx);
            static void CCONV onEncoderR_Attach(PhidgetHandle ch, void * ctx);

            static void CCONV onEngineL_Detach(PhidgetHandle ch, void * ctx);
            static void CCONV onEngineR_Detach(PhidgetHandle ch, void * ctx);
            static void CCONV onEncoderL_Detach(PhidgetHandle ch, void * ctx);
            static void CCONV onEncoderR_Detach(PhidgetHandle ch, void * ctx);
    };
}


#endif