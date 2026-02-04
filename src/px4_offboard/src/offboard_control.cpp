#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>

// 引入 px4_msgs 定义的消息
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		// 1. 创建发布者
		//用于发送控制模式心跳
		offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
			"/fmu/in/offboard_control_mode", 10);
		
		// 用于发送位置/速度指令
		trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
			"/fmu/in/trajectory_setpoint", 10);
		
		// 用于发送命令（解锁、切模式）
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
			"/fmu/in/vehicle_command", 10);

		// 2. 创建定时器，100ms (10Hz) 运行一次
		// 注意：Offboard模式要求频率必须大于 2Hz
		timer_ = this->create_wall_timer(
			100ms, std::bind(&OffboardControl::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Offboard Control Node Started!");
	}

private:
	void timer_callback()
	{
		// 计数器，用于控制流程顺序
		if (offboard_setpoint_counter_ == 0) {
            RCLCPP_INFO(this->get_logger(), "Starting warmup (sending setpoints)...");
        }

		// --- 步骤 1: 持续发送“控制模式”心跳 ---
		// 告诉飞控：我要控制位置 (position=true)，其他的我不控制
		publish_offboard_control_mode();

		// --- 步骤 2: 持续发送“位置设定点” ---
		// NED坐标系: x=0(北), y=0(东), z=-5.0(向上5米)
		publish_trajectory_setpoint(0.0, 0.0, -5.0);

		// --- 步骤 3: 状态机流程控制 ---
		
		// 运行10个周期(1秒)后，尝试切换到 Offboard 模式
		// (PX4要求必须先收到一段时间的设定点，才允许切换模式)
		if (offboard_setpoint_counter_ == 10) {
			RCLCPP_INFO(this->get_logger(), "Switching to Offboard Mode");
			publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            // 参数1=1 (Custom Mode), 参数2=6 (Offboard)
		}

		// 运行20个周期(2秒)后，尝试解锁 (Arm)
		if (offboard_setpoint_counter_ == 20) {
			RCLCPP_INFO(this->get_logger(), "Arming...");
			publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            // 参数1=1.0 (Arm)
		}

		// 计数器增加
		if (offboard_setpoint_counter_ < 100) {
			offboard_setpoint_counter_++;
		}
	}

	// 辅助函数：发送解锁/模式切换命令
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
	{
		px4_msgs::msg::VehicleCommand msg{};
		msg.param1 = param1;
		msg.param2 = param2;
		msg.command = command;
		msg.target_system = 1;
		msg.target_component = 1;
		msg.source_system = 1;
		msg.source_component = 1;
		msg.from_external = true;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		vehicle_command_publisher_->publish(msg);
	}

	// 辅助函数：发送 Offboard 控制模式
	void publish_offboard_control_mode()
	{
		px4_msgs::msg::OffboardControlMode msg{};
		msg.position = true;       // 启用位置控制
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		offboard_control_mode_publisher_->publish(msg);
	}

	// 辅助函数：发送位置设定点
	void publish_trajectory_setpoint(float x, float y, float z)
	{
		px4_msgs::msg::TrajectorySetpoint msg{};
		msg.position = {x, y, z};  // NED 坐标
		msg.yaw = -3.14; // 机头朝向 (弧度)，-3.14 ≈ 朝南/北取决于定义，这里保持固定即可
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publisher_->publish(msg);
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

	uint64_t offboard_setpoint_counter_ = 0;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();
	return 0;
}