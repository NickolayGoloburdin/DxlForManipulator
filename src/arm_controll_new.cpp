#include <iostream>
#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <time.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"

#define PROTOCOL_VERSION                1.0 
#define PERIOD_PROTOCOL                 15
#define PERIOD_MOVEMENT                 15
#define DEVICE_NUMBER                   0

using namespace std;

class Manipulator {

    public:
    uint16_t device_number = DEVICE_NUMBER;
    int motor_q;
    bool torque;
    std::vector<int32_t> goal_position; //zero poses
    int32_t velocity = 30;
    std::vector<int32_t> goal_velocity;
    std::vector<int32_t> goal_acceleration;
    std::vector<uint8_t> dxl_id;
    std::vector<uint8_t> ax_id;
    std::vector<uint8_t> mx_id;
    std::vector<double> present_position;
    std::vector<double> present_speed;
    std::vector<double> present_temp;
    std::vector<double> present_load;
    uint8_t d_gain = 0; //128
    uint8_t i_gain = 128; //16
    uint8_t p_gain = 64; //16
        Manipulator(){}
        void set_start_pos()
            {
                for(auto i : dxl_id)
            
                    {
                        present_load.push_back(0);
                        present_temp.push_back(0);
                        present_speed.push_back(0);
                        present_position.push_back(0);
                    }
            }

        void arm_init()
            {
                const char* log;
                if (!dxl_wb.init(DEVICENAME, BAUDRATE, &log))
                    {
                        ROS_ERROR_STREAM(log);
                        ROS_ERROR_STREAM("Failed to init");
                    }
                for (auto i : dxl_id)
                    {
                        if (!ping_motor(*i,device_number))
                            {
                                ROS_ERROR_STREAM("Failed to init motor"<<i+1);
                            }   
                    }
                for (auto i : dxl_id)
                    {
                        if (!dxl_wb.jointMode(*i, velocity, 100, &log);)
                            {
                                ROS_ERROR_STREAM(log);
                                ROS_ERROR_STREAM("Failed to change joint mode");      
                            } 
                    }
            
                dxl_wb.addSyncWriteHandler(*(dxl_id.begin()), "Goal_Position",&log);
	            dxl_wb.addSyncWriteHandler(*(dxl_id.begin()), "Moving_Speed",&log);
	            dxl_wb.addSyncWriteHandler(*(dxl_id.begin(), "Goal_Acceleration",&log);
                for (auto i: mx_id)
                    {
                        dxl_wb.writeRegister(*i, uint16_t(26), uint16_t(1), &d_gain, &log);
                        dxl_wb.writeRegister(*i, uint16_t(27), uint16_t(1), &i_gain, &log);
                        dxl_wb.writeRegister(*i, uint16_t(28), uint16_t(1), &p_gain, &log);
                    }

            }

        void sync_write_pos()
            {
                const char*log;

                for(int i = 0; i < motor_q; i++)
                    {
                    dxl_wb.writeRegister(dxl_id[i], "Goal_Position", goal_position[i], &log);
                    dxl_wb.writeRegister(dxl_id[i], "Moving_Speed", goal_velocity[i], &log);
                    dxl_wb.writeRegister(dxl_id[i], "Goal_Acceleration", goal_acceleration[i], &log);
                    }
            }
        void disable_all_torque()
        {
            (auto i : dxl_id)
            {disable_motor_torque(*i)}
        }
        void enable_all_torque()
        {
            (auto i : dxl_id)
            {enable_motor_torque(*i)}
        }

        void read_all_pos()
        {
            (auto i : dxl_id)
            {read_motor_pos(*i, this)}
        }
        void read_all_vel()
        {
            (auto i : dxl_id)
            {read_motor_vel(*i, this)}
        }
        void read_all_temp()
        {
            (auto i : dxl_id)
            {read_motor_temp(*i, this)}
        }
        void read_all_load()
        {
            (auto i : dxl_id)
            {read_motor_load(*i, this)}
        }
        void motor_torque_control(bool torque_flag)
        {
            if (torque && )
                enable_all_torque();
            else
                disable_all_torque();
        }
        
        
    
};

class Angle: public Manipulator

{
    public:

        Angle()
    {
        motor_q = 6;
        mx_id = {1,2,3,4}
        ax_id = {5,6};
        dxl_id = {1,2,3,4,5,6};
        goal_position = {2048, 2048, 2048, 2048, 512, 680};;
        goal_velocity = {40,40,40,40,40,40};
        goal_acceleration = {2,2,2,2};
    }
};
void disable_motor_torque(uint8_t id)
    {
        const char*log;
        dxl_wb.torqueOff(id, &log);
    }
void enable_motor_torque(uint8_t id)
    {
        const char*log;
        dxl_wb.torqueOn(id, &log);
    }
    
void read_motor_pos(uint8_t id, Manipulator * manip)
    {
        const char*log;
        int32_t get_data = 0;
        dxl_wb.itemRead(id, "Present_Position", &get_data, &log);
        manip->present_position[id-1] = get_data;

    }

void read_motor_vel(uint8_t id, Manipulator * manip)
    {
        const char*log;
        int32_t get_data = 0;
        dxl_wb.itemRead(id,"Present_Speed", &get_data, &log);
        manip->present_speed[id-1] = get_data; 
    }

void read_motor_temp(uint8_t id, Manipulator * manip){
    const char*log;
    int32_t get_data = 0;
    dxl_wb.itemRead(id,"Present_Temperature", &get_data, &log);
    manip->present_temp[id-1] = get_data;
}

void read_motor_load(uint8_t id, Manipulator * manip){
    const char*log;
    int32_t get_data = 0;
    dxl_wb.itemRead(id,"Present_Load", &get_data, &log);
    manip->present_load[id-1] = get_data;
}

void messageJointscmd(const sensor_msgs::JointState::ConstPtr& toggle_msg)  //controll position
{	
    for(int i = 0; i<toggle_msg->position.size(); i++){
        goal_position[i] = (int)toggle_msg->position[i];
    }
    for(int i = 0; i<toggle_msg->position.size(); i++){
        goal_velocity[i] = (int)toggle_msg->velocity[i];
    }
    for(int i = 0; i<toggle_msg->position.size(); i++){
        goal_acceleration[i] = (int)toggle_msg->effort[i];
    }
}

void messageTorqueMotor(const std_msgs::Bool::ConstPtr& msg){
    torque_flag = msg->data;
}
int main(int argc, char **argv)
{   
    
    ms0 = millis();
    ros::init(argc,argv, "Dxl_Arm");
    ros::NodeHandle nh;
    bool param;
    bool torque_flag;
    sensor_msgs::JointState joints_msg;
    sensor_msgs::JointState temp_load;
    ros::Subscriber jointcmd = nh.subscribe<sensor_msgs::JointState> ("cmd_joints",10, messageJointscmd );
    ros::Subscriber torque_motor = nh.subscribe<std_msgs::Bool> ("disable_torque",10, messageTorqueMotor);
    ros::Publisher Joint_State = nh.advertise<sensor_msgs::JointState>("arm_joint_states", 64);
    ros::Publisher Temp_And_Load = nh.advertise<sensor_msgs::JointState>("temp_load", 64);

    ros::Rate loop_rate(PERIOD_PROTOCOL);
    std::string manipulator_type;
    nh.getParam("/arm/manipulator_type",manipulator_type);
    if (manipulator_type=="angle")
        {
            Angle arm;
            Manipulator * manip = &arm;
        }
    manip->set_start_pos();
    manip->arm_init();

    struct timeval t_s,t_c;
    gettimeofday(&t_s, NULL);
    //std::cout<<"start";
    while(ros::ok()){

        manip->read_all_pos();
        manip->read_all_vel();
        manip->sync_write_pos();
        manip->motor_torque_control(torque_flag);

        joints_msg.velocity = present_speed;
        joints_msg.position = present_position;
        joints_msg.header.stamp = ros::Time::now();
        Joint_State.publish(joints_msg);
        
        temp_load.velocity = present_load;
        temp_load.position = present_temp;
        temp_load.header.stamp = ros::Time::now();
        Temp_And_Load.publish(temp_load);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
        manip->disable_all_torque();
    return result;
}
