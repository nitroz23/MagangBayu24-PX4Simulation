#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

int missionStep = 0;
float droneX[10] = {0.00, 0.00, 0.00, 3.00, 3.000, 3.000, 3.000, 0.000, 0.000, 0.00};
float droneY[10] = {0.00, 1.50, 1.50, 1.50, 1.500, -1.50, -1.50, -1.50, -1.50, 0.00};
float droneRad = 0.00;
//float droneYaw[10] = {1.57, 1.57, 0.00, 0.00, -1.57, -1.57, -3.14, -3.14, -4.71, -4.71};

class MissionControl : public rclcpp::Node
{
public:
    MissionControl() : Node("mission_control")
    {

        offboardControlModePublisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectorySetpointPublisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicleCommandPublisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        offboardSetpointCounter_ = 0;

        auto timerCallback = [this]() -> void {

            if (offboardSetpointCounter_ == 0){
                this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
            }

            publishOffboardControlMode();
            publishTrajectorySetpoint();

            if (offboardSetpointCounter_ % 40 == 0 && offboardSetpointCounter_ != 0 && missionStep != 0 && missionStep < 10){
                missionStep++;
                if (missionStep % 2 == 0){droneRad -= 1.57;}
            } 
            else if (offboardSetpointCounter_ == 70){missionStep = 1;} 
            else if (missionStep == 10){
                this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND, 1, 0);
                if (offboardSetpointCounter_ == 550){
                    this->disarm();
                    missionStep++;
                }
            }

            if (missionStep < 11){
                offboardSetpointCounter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timerCallback);
    }

    void arm();
    void disarm();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboardControlModePublisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectorySetpointPublisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicleCommandPublisher_;

    std::atomic<uint64_t> timestamp_;

    uint64_t offboardSetpointCounter_;

    void publishOffboardControlMode();
    void publishTrajectorySetpoint();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void MissionControl::arm(){
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void MissionControl::disarm(){
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void MissionControl::publishOffboardControlMode(){
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboardControlModePublisher_->publish(msg);
}

void MissionControl::publishTrajectorySetpoint(){
    TrajectorySetpoint msg{};
    msg.position = {droneX[missionStep], droneY[missionStep], -5.0};
    msg.yaw = droneRad - 1.57;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectorySetpointPublisher_->publish(msg);
}

void MissionControl::publishVehicleCommand(uint16_t command, float param1, float param2){
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicleCommandPublisher_->publish(msg);
}

int main(int argc, char *argv[]){
    std::cout << "Starting mission control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControl>());

    rclcpp::shutdown();
    return 0;
}
