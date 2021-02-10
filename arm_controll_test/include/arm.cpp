#include <arm.hpp>
#include <ros/ros.h>

void Manipulator::set_start_pos() {
  for (auto i : dxl_id)

  {
    present_load.push_back(0);
    present_temp.push_back(0);
    present_speed.push_back(0);
    present_position.push_back(0);
  }
}

void Manipulator::arm_init() {
  const char* log;
  if (!init(DEVICENAME, BAUDRATE, &log)) {
    ROS_ERROR_STREAM(log);
    ROS_ERROR_STREAM("Failed to init");
  }
  for (auto i : dxl_id) {
    if (!ping_motor(i, device_number)) {
      ROS_ERROR_STREAM("Failed to init motor" << i + 1);
    }
  }
  for (auto i : dxl_id) {
    if (!jointMode(i, velocity, 100, &log)) {
      ROS_ERROR_STREAM(log);
      ROS_ERROR_STREAM("Failed to change joint mode");
    }
  }

  addSyncWriteHandler(*(dxl_id.begin()), "Goal_Position", &log);
  addSyncWriteHandler(*(dxl_id.begin()), "Moving_Speed", &log);
  addSyncWriteHandler(*(dxl_id.begin()), "Goal_Acceleration", &log);
  for (auto i : mx_id) {
    writeRegister(i, uint16_t(26), uint16_t(1), &d_gain, &log);
    writeRegister(i, uint16_t(27), uint16_t(1), &i_gain, &log);
    writeRegister(i, uint16_t(28), uint16_t(1), &p_gain, &log);
  }
}

void Manipulator::sync_write_pos() {
  const char* log;

  for (int i = 0; i < motor_q; i++) {
    writeRegister(dxl_id[i], "Goal_Position", goal_position[i], &log);
    writeRegister(dxl_id[i], "Moving_Speed", goal_velocity[i], &log);
    writeRegister(dxl_id[i], "Goal_Acceleration", goal_acceleration[i], &log);
  }
}
void Manipulator::disable_all_torque() {
  for (auto i : dxl_id) {
    disable_motor_torque(i);
  }
}
void Manipulator::enable_all_torque() {
  for (auto i : dxl_id) {
    enable_motor_torque(i);
  }
}

void Manipulator::read_all_pos() {
  for (auto i : dxl_id) {
    read_motor_pos(i);
  }
}
void Manipulator::read_all_vel() {
  for (auto i : dxl_id) {
    read_motor_vel(i);
  }
}
void Manipulator::read_all_temp() {
  for (auto i : dxl_id) {
    read_motor_temp(i);
  }
}
void Manipulator::read_all_load() {
  for (auto i : dxl_id) {
    read_motor_load(i);
  }
}
void Manipulator::motor_torque_control(bool torque_flag) {
  if (!(torque && torque_flag) && (torque_flag == true))
    enable_all_torque();
  else
    disable_all_torque();
}

bool Manipulator::ping_motor(uint8_t id, uint16_t device_number) {
  const char* log;
  return ping(id, &device_number, &log);
}
void Manipulator::disable_motor_torque(uint8_t id) {
  const char* log;
  torqueOff(id, &log);
}
void Manipulator::enable_motor_torque(uint8_t id) {
  const char* log;
  torqueOn(id, &log);
}

void Manipulator::read_motor_pos(uint8_t id) {
  const char* log;
  int32_t get_data = 0;
  itemRead(id, "Present_Position", &get_data, &log);
  present_position[id - 1] = get_data;
}

void Manipulator::read_motor_vel(uint8_t id) {
  const char* log;
  int32_t get_data = 0;
  itemRead(id, "Present_Speed", &get_data, &log);
  present_speed[id - 1] = get_data;
}

void Manipulator::read_motor_temp(uint8_t id) {
  const char* log;
  int32_t get_data = 0;
  itemRead(id, "Present_Temperature", &get_data, &log);
  present_temp[id - 1] = get_data;
}

void Manipulator::read_motor_load(uint8_t id) {
  const char* log;
  int32_t get_data = 0;
  itemRead(id, "Present_Load", &get_data, &log);
  present_load[id - 1] = get_data;
}
