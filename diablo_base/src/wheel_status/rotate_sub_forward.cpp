#include <ros/ros.h>
#include <std_msgs/String.h>
#include "OSDK_Vehicle.hpp"
#include <thread>
#include <chrono>

DIABLO::OSDK::Movement_Ctrl* pMovementCtrl;
int rcount = 0;
// float right_pos, left_pos, right_enc, left_enc, right_init, right_offset, left_init, left_offset;
// int init_values = 0;
// float adj_right, adj_left;

void teleop_ctrl(const std_msgs::String::ConstPtr& msg)
{
    if(!pMovementCtrl->in_control())
    {
        printf("to get movement ctrl.\n");
        uint8_t result = pMovementCtrl->obtain_control();
        return;
    }
    if(pMovementCtrl->ctrl_mode_data.height_ctrl_mode == 1)
        pMovementCtrl->ctrl_data.up = 0.0f;
    pMovementCtrl->ctrl_data.forward = 0.0f;
    pMovementCtrl->ctrl_data.left = 0.0f;
    for(const char& c : msg->data)
    {
        switch(c)
        {
            case 's':
                if(rcount < 10){
                    pMovementCtrl->ctrl_data.forward = 1.0f;
                    // adj_right = right_pos - right_offset;
                    // adj_left = left_pos - left_offset;
                    rcount++;
                    std::cout << "---------" << std::endl;
                    std::cout << "Forward step: " << rcount << std::endl;
                }
                else{
                    std::cout << "---------" << std::endl;
                    std::cout << "Ending" << std::endl;
                    ros::shutdown();
                }
                break;
            
            default:
                break;
    }
        }
    if(pMovementCtrl->ctrl_mode_cmd)
    {uint8_t result = pMovementCtrl->SendMovementModeCtrlCmd();}
    else
    {uint8_t result = pMovementCtrl->SendMovementCtrlCmd();}
}

// void motor_callback(const diablo_sdk::OSDK_LEGMOTORS::ConstPtr& information)
// {
//     right_pos = information->right_wheel_pos;
//     left_pos = information->left_wheel_pos;
//     right_enc = information->right_wheel_enc_rev;
//     left_enc = information->left_wheel_enc_rev;
//     if(init_values < 2){
//         right_init = information->right_wheel_enc_rev;
//         left_init = information->left_wheel_enc_rev;
//         right_offset = information->right_wheel_pos;
//         left_offset = information->left_wheel_pos;
//         init_values++;
//     }
// }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_testing");
    ros::NodeHandle nh("~");

    DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init()) return -1;

    DIABLO::OSDK::Vehicle vehicle(&Hal);                                  //Initialize Onboard SDK
    if(vehicle.init()) return -1;

    vehicle.telemetry->activate();
    //vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_1Hz);
    //vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_100Hz);
    // vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_100Hz);
    // vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_100Hz);
    //vehicle.telemetry->configUpdate(); 
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_POWER);
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_QUATERNION);
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_ACCL);
    //vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_GYRO);
    pMovementCtrl = vehicle.movement_ctrl;
    ros::Subscriber sub = nh.subscribe("/move", 1, teleop_ctrl); //subscribe to ROS topic
    // ros::Subscriber sub2 = nh.subscribe("/robot_status_example/diablo_ros_LEGMOTORS_b", 10, motor_callback);
    ros::spin();

    return 0;
}
