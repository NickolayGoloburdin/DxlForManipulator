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

    }

}

void sync_write_pos(Manipulator * manip)
    {
        const char*log;

        for(int i = 0; i < manip->motor_q; i++)
            {
            dxl_wb.writeRegister(manip->dxl_id[i], "Goal_Position", manip->goal_position[i], &log);
            dxl_wb.writeRegister(manip->dxl_id[i], "Moving_Speed", manip->goal_velocity[i], &log);
            dxl_wb.writeRegister(manip->dxl_id[i], "Goal_Acceleration", manip->goal_acceleration[i], &log);
            }
    }

}
void disableTorque(uint8_t id)
    {
        const char*log;
        bool result = false;
        dxl_wb.torqueOff(id, &log);
    }


void read_pos(uint8_t id, Manipulator * manip)
    {
        const char*log;
        int32_t get_data = 0;
        dxl_wb.itemRead(id, "Present_Position", &get_data, &log);
        manip->present_position[id-1] = get_data;

    }

void read_vel(uint8_t id, Manipulator * manip)
    {
        const char*log;
        int32_t get_data = 0;
        dxl_wb.itemRead(id,"Present_Speed", &get_data, &log);
        manip->present_speed[id-1] = get_data; 
    }

void read_temp(uint8_t id, Manipulator * manip){
    const char*log;
    int32_t get_data = 0;
    dxl_wb.itemRead(id,"Present_Temperature", &get_data, &log);
    manip->present_temp[id-1] = get_data;
}

void read_load(uint8_t id, Manipulator * manip){
    const char*log;
    int32_t get_data = 0;
    dxl_wb.itemRead(id,"Present_Load", &get_data, &log);
    manip->present_load[id-1] = get_data;
}

