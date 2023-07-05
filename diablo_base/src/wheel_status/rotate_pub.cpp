#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>


int counter = 0; 

int main(int argc, char** argv){

    ros::init(argc, argv, "rotate_commander");
    ros::NodeHandle n;
    ros::Publisher commander = n.advertise<std_msgs::String>("/move", 1);
    ros::Rate loop_rate(10);
    std::cout << "Starting" << std::endl; 
    while(ros::ok()){
        std_msgs::String message;
        message.data = "s";
        commander.publish(message);
        // counter++;
        // if(counter == -1){
        //     std::cout << "Completed: " << counter << std::endl;
        //     ros::shutdown();
        // }
        loop_rate.sleep();
    }

    return 0;
}