#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "diablo_sdk/OSDK_LEGMOTORS.h"
#define _USE_MATH_DEFINES
#include <cmath>

float right_pos, left_pos, right_enc, left_enc, right_init, right_offset, left_init, left_offset;
int init_values = 0;
void motor_callback(const diablo_sdk::OSDK_LEGMOTORS::ConstPtr& information){
    right_pos = information->right_wheel_pos;
    left_pos = information->left_wheel_pos;
    right_enc = information->right_wheel_enc_rev;
    left_enc = information->left_wheel_enc_rev;
    init_values++;
    if(init_values == 3){
        std::ofstream outputFile("src/diablo-sdk-v1/wheel_status/log.txt", std::ios::app);
        float right_displacement = 9*(2 * M_PI * (right_enc) + (right_pos));
        float left_displacement = 9*(2 * M_PI * (left_enc) + (left_pos));
        outputFile << left_displacement << "," << right_displacement << std::endl;
        outputFile.close();
        std::cout << "Data logged successfully" << std::endl;
        ros::shutdown();
    }
}
    

int main(int argc, char** argv){

    ros::init(argc, argv, "get_wheel_status");
    ros::NodeHandle n;

    ros::Subscriber sub_motor = n.subscribe("/robot_status_example/diablo_ros_LEGMOTORS_b", 10, motor_callback);
    ros::spin();
    return 0;
}