#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <cmath> // 用于 sin, cos, atan2

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace std::chrono_literals;

class Figure8Control : public rclcpp::Node
{
public:
    Figure8Control() : Node("figure8_control")
    {
        // 发布者
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // 订阅者 (可选：如果你想知道当前高度再开始飞8字，可以订阅 local_position)
        // 这里为了代码简洁，我们使用简单的延时逻辑

        // 定时器：50ms (20Hz)
        timer_ = this->create_wall_timer(
            50ms, std::bind(&Figure8Control::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Figure 8 Node Started!");
    }

private:
    void timer_callback()
    {
        // 1. 发送 Offboard 心跳 (必须持续发送)
        publish_offboard_control_mode();

        // 2. 状态机逻辑
        // 0-10 (0.5秒): 预热，只发位置流
        // 10: 切 Offboard
        // 20: 解锁
        // 20-200 (1秒-10秒): 起飞并爬升
        // >200: 开始飞 8 字

        if (counter_ == 10) {
            // 切换到 Offboard 模式
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            RCLCPP_INFO(this->get_logger(), "Switching to Offboard Mode");
        }

        if (counter_ == 20) {
            // 解锁
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            RCLCPP_INFO(this->get_logger(), "Arming");
        }

        // 计算控制指令
        float x = 0.0;
        float y = 0.0;
        float z = -5.0; // 目标高度 5米 (NED系负数为上)
        float yaw = 0.0; // 默认朝北

        if (counter_ < 200) {
            // --- 起飞阶段 ---
            // 保持在 (0,0) 位置，只控制高度
            x = 0.0;
            y = 0.0;
            z = -5.0; 
            yaw = 0.0;
            if(counter_ % 50 == 0) RCLCPP_INFO(this->get_logger(), "Taking off...");
        } 
        else {
            // --- 8 字飞行阶段 ---
            // 增加角度 theta
            theta_ += delta_theta_;

            // 8字参数
            float radius = 5.0f; // 8字半径 5米
            
            // 数学公式 (Lemniscate of Gerono / 8字形)
            // x = R * sin(theta)
            // y = R * sin(2 * theta)
            x = radius * sin(theta_);
            y = radius * sin(2 * theta_);
            z = -5.0;

            // --- 计算 Yaw (机头朝向) ---
            // 想要机头朝前，需要计算速度向量的方向
            // dx/dt = R * cos(theta)
            // dy/dt = R * 2 * cos(2*theta)
            float dx = radius * cos(theta_);
            float dy = radius * 2 * cos(2 * theta_);
            yaw = atan2(dy, dx); 
        }

        // 发送位置和Yaw指令
        publish_trajectory_setpoint(x, y, z, yaw);

        counter_++;
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;      // 我们控制位置
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint(float x, float y, float z, float yaw)
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {x, y, z};
        msg.yaw = yaw; 
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

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

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    uint64_t counter_ = 0;      // 计数器
    double theta_ = 0.0;        // 8字轨迹的当前角度参数
    double delta_theta_ = 0.02; // 每次更新的角度增量 (决定飞行速度)
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Figure8Control>());
    rclcpp::shutdown();
    return 0;
}