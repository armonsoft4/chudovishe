#include "chudovishe_hardware/chudovishe_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace chudovishe_hardware {
hardware_interface::CallbackReturn ChudovisheSystemHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& /* params */) {
    // Reading config file
    port_name_ = info_.hardware_parameters["port_name"];
    baudrate_ = std::stoi(info_.hardware_parameters["baudrate"]);
    timeout_ms_ = std::stoi(info_.hardware_parameters["timeout_ms"]);

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        // DiffBotSystem has exactly two states and one command interface on
        // each joint
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                logger_,
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                logger_,
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(logger_,
                         "Joint '%s' has %zu state interface. 2 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                logger_,
                "Joint '%s' have '%s' as first state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name !=
            hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(logger_,
                         "Joint '%s' have '%s' as second state interface. '%s' "
                         "expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[1].name.c_str(),
                         hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    left_wheel_.name = info_.joints[0].name;
    right_wheel_.name = info_.joints[1].name;
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ChudovisheSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        left_wheel_.name, hardware_interface::HW_IF_POSITION,
        &left_wheel_.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        left_wheel_.name, hardware_interface::HW_IF_VELOCITY,
        &left_wheel_.velocity));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        right_wheel_.name, hardware_interface::HW_IF_POSITION,
        &right_wheel_.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        right_wheel_.name, hardware_interface::HW_IF_VELOCITY,
        &right_wheel_.velocity));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ChudovisheSystemHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        left_wheel_.name, hardware_interface::HW_IF_VELOCITY,
        &left_wheel_.command));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        right_wheel_.name, hardware_interface::HW_IF_VELOCITY,
        &right_wheel_.command));

    return command_interfaces;
}

hardware_interface::CallbackReturn ChudovisheSystemHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(logger_, "Configuring ...please wait...");

    // reset values always when configuring hardware
    for (const auto& [name, descr] : joint_state_interfaces_) {
        set_state(name, 0.0);
    }
    for (const auto& [name, descr] : joint_command_interfaces_) {
        set_command(name, 0.0);
    }

    RCLCPP_INFO(logger_, "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ChudovisheSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(logger_, "Activating ...please wait...");

    // command and state should be equal when starting
    for (const auto& [name, descr] : joint_command_interfaces_) {
        set_command(name, get_state(name));
    }

    try {
        serial::Serial serial_ = serial::Serial(
            port_name_, baudrate_, serial::Timeout::simpleTimeout(timeout_ms_));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s",
                     port_name_.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ChudovisheSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(logger_, "Deactivating ...please wait...");

    try {
        serial_.close();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to close serial port %s: %s",
                     port_name_.c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ChudovisheSystemHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /* period */) {
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
chudovishe_hardware::ChudovisheSystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    return hardware_interface::return_type::OK;
}

}  // namespace chudovishe_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(chudovishe_hardware::ChudovisheSystemHardware,
                       hardware_interface::SystemInterface)