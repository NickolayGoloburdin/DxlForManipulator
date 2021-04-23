#include <iostream>
#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif
#include <boost/bind.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB0"
#define DEVICE_NUMBER 0

class Manipulator : public DynamixelWorkbench {
public:
  uint16_t device_number = DEVICE_NUMBER;
  int motor_q;
  bool torque;
  std::vector<int32_t> goal_position; // zero poses
  int32_t velocity;
  std::vector<int32_t> goal_velocity;
  std::vector<int32_t> goal_acceleration;
  std::vector<uint8_t> dxl_id;
  std::vector<uint8_t> ax_id;
  std::vector<uint8_t> mx_id;
  std::vector<double> present_position;
  std::vector<double> present_speed;
  std::vector<double> present_temp;
  std::vector<double> present_load;
  uint8_t d_gain = 0;   // 128
  uint8_t i_gain = 128; // 16
  uint8_t p_gain = 64;  // 16
  void set_start_pos();
  void arm_init();
  void sync_write_pos();
  void disable_all_torque();
  void enable_all_torque();
  void read_all_pos();
  void read_all_vel();
  void read_all_temp();
  void read_all_load();
  void motor_torque_control(bool torque_flag);
  void disable_motor_torque(uint8_t id);
  void enable_motor_torque(uint8_t id);
  void read_motor_pos(uint8_t id);
  void read_motor_vel(uint8_t id);
  void read_motor_temp(uint8_t id);
  void read_motor_load(uint8_t id);
  bool ping_motor(uint8_t id, uint16_t device_number);
};

class Angle : public Manipulator {
public:
  Angle() {
    motor_q = 6;
    mx_id = {1, 2, 3, 4};
    ax_id = {5, 6};
    dxl_id = {1, 2, 3, 4, 5, 6};
    goal_position = {2048, 2048, 2048, 2048, 512, 680};
    goal_velocity = {40, 40, 40, 40, 40, 40};
    goal_acceleration = {2, 2, 2, 2};
  }
};
class Palletizier : public Manipulator {
public:
  Palletizier() {
    motor_q = 6;
    mx_id = {1, 2, 3};
    dxl_id = {1, 2, 3, 4, 5};
    goal_position = {2048, 1930, 2230, 512, 512};
    goal_velocity = {40, 40, 40, 40, 40};
    goal_acceleration = {2, 2, 2};
  }
};
class Delta : public Manipulator {
public:
  Delta() {
    motor_q = 3;
    mx_id = {1, 2, 3};
    dxl_id = {1, 2, 3};
    goal_position = {2048, 2048, 2048};
    goal_velocity = {40, 40, 40};
    goal_acceleration = {2, 2, 2};
  }
};
class ROSArm : public rclcpp::Node {
public:
  explicit ROSArm(Manipulator *manip);

  Manipulator *manip_;

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisherjs_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publishertl_;
  size_t count_;
  sensor_msgs::msg::JointState joints_msg_;
  sensor_msgs::msg::JointState temp_load_;
  bool torque_flag_;
};
