#include "ros/ros.h"
#include "std_msgs/Int8.h"

void msgCallback(const std_msgs::Int8::ConstPtr& msg)
{
    ROS_INFO("receive msg = %d", msg -> data);
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "testListener");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("test_topic",100,msgCallback);
    ros::spin(); //어떤 값이 들어오기 전까지 대기 (다시 위로 올라감)
    return 0;
}
