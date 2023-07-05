#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

int x,y,z = 0;

void teleop_ctrl(const std_msgs::String::ConstPtr& msg){
    for(const char& c : msg->data)
    {
        switch(c)
        {
            case 'w':
                x++;   
                break;
            case 's':
                x--;
                break;
            case 'a':
                y++;
                break;
            case 'd':
                y--;
                break;
            default:
                break;
        }
    }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  ros::Subscriber sub = n.subscribe("/DJ_teleop", 1, teleop_ctrl);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(x, y, z)),
        ros::Time::now(),"odom", "base_link"));
    ros::spinOnce();
    r.sleep();
  }
}