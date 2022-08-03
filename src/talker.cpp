//C++
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <vector>

#include <math.h>

// ROS消息
#include "ros/ros.h"
#include<jakarobot/a.h>
#include<std_msgs/Float32.h>
//#include<std_msgs/Time.h>

const float PI = 3.1415926535;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"velocity_talker");//节点初始化
    ros::NodeHandle h;//创建节点句柄
    jakarobot::a msg;
    msg.vel_1 = 0;
    msg.vel_2 = 0;
    msg.vel_3 = 0;
    msg.vel_4 = 0;
    msg.vel_5 = 0;
    
    ros::Publisher chatter_pub = h.advertise<jakarobot::a>("velocity_talker",50); //消息缓存1000
    ros::Rate looprate(25);//循环频率25Hz
    unsigned int i = 0;
 
    while (ros::ok())
    {
        msg.time_stamp = (float)(ros::Time::now().toSec());//时间戳，预留，暂时无用
        msg.vel_6 = 1*sin(2*PI*i/64);
        //ROS_INFO("%s",msg.vel_2.c_str());
        chatter_pub.publish(msg);//发布消息
        i++;
        ros::spinOnce();//等待回调函数
        looprate.sleep();//按照之前设定的进行循环

    }
    
}
