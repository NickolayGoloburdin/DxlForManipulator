#include <iostream>
#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif
#include <arm.cpp>

#define PROTOCOL_VERSION 1.0
#define PERIOD_PROTOCOL 15
#define PERIOD_MOVEMENT 15
using namespace std;

// void messageJointscmd(const sensor_msgs::JointState::ConstPtr& toggle_msg,
// Manipulator* manip )  //control position
//     {
//         for(int i = 0; i<toggle_msg->position.size(); i++){
//             manip->goal_position[i] = (int)toggle_msg->position[i];
//         }
//         for(int i = 0; i<toggle_msg->position.size(); i++){
//             manip->goal_velocity[i] = (int)toggle_msg->velocity[i];
//         }
//         for(int i = 0; i<toggle_msg->position.size(); i++){
//             manip->goal_acceleration[i] = (int)toggle_msg->effort[i];
//         }
//     }

// void messageTorqueMotor(const std_msgs::Bool::ConstPtr& msg, bool &
// torque_flag)
//     {
//     torque_flag = msg->data;

//     }
int main(int argc, char **argv) {

  // ros::init(argc, argv, "Dxl_Arm");
  // ros::NodeHandle nh;
  // bool param;
  // bool torque_flag = false;
  // std::string manipulator_type;
  // nh.getParam("/arm/manipulator_type", manipulator_type);
  // if (manipulator_type == "angle")
  //   Manipulator *manip = new Angle;
  // else if (manipulator_type == "delta")
  //   Manipulator *manip = new Palletizier;

  Manipulator *manip = new Angle;

  // sensor_msgs::JointState joints_msg;
  // sensor_msgs::JointState temp_load;
  // // ros::Subscriber jointcmd = nh.subscribe<sensor_msgs::JointState>
  // // ("cmd_joints",10, boost::bind(messageJointscmd, _1, manip));
  // // ros::Subscriber torque_motor = nh.subscribe<std_msgs::Bool>
  // // ("disable_torque",10,
  // // boost::bind(messageTorqueMotor, _1, torque_flag));
  // ros::Publisher Joint_State =
  //     nh.advertise<sensor_msgs::JointState>("arm_joint_states", 64);
  // ros::Publisher Temp_And_Load =
  //     nh.advertise<sensor_msgs::JointState>("temp_load", 64);
  // ros::Rate loop_rate(PERIOD_PROTOCOL);
  // manip->set_start_pos();
  // manip->arm_init();
  // struct timeval t_s, t_c;
  // while (ros::ok()) {
  //   manip->read_all_pos();
  //   manip->read_all_vel();
  //   manip->sync_write_pos();
  //   manip->motor_torque_control(torque_flag);

  //   joints_msg.velocity = manip->present_speed;
  //   joints_msg.position = manip->present_position;
  //   joints_msg.header.stamp = ros::Time::now();
  //   Joint_State.publish(joints_msg);

  //   temp_load.velocity = manip->present_load;
  //   temp_load.position = manip->present_temp;
  //   temp_load.header.stamp = ros::Time::now();
  //   Temp_And_Load.publish(temp_load);

  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  // manip->disable_all_torque();
  // return 0;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSArm>(ROSArm(manip)));
  rclcpp::shutdown();
  manip->disable_all_torque();
  return 0;
}
