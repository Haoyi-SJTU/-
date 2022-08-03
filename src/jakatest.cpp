//C++
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <vector>


//jaka
#include <JAKAZuRobot.h>
#include <jkerr.h>
#include <jktypes.h>

// ROS消息
#include "ros/ros.h"
#include<jakarobot/a.h>
#include<std_msgs/Float32.h>
#include<std_msgs/Float64.h>

//#define ROBOT_SHUTDOWN //关闭机器人

// 机器人对象 全局变量
JAKAZuRobot jaka_robot;//extern 

//double real_joint=0;
//double desire_joint=0;

std_msgs::Float64 real_joint;
std_msgs::Float64 desire_joint;

// 连接JAKA机械臂
// 输入:JAKA机器人对象,机器人IP
// 输出：true-成功 false-失败
bool Connect_Robot(JAKAZuRobot& robot, std::string IP)
{
    if(!robot.login_in(IP.c_str()))
    {
        if(!robot.power_on())
        {
            if(!robot.enable_robot())
            {
                sleep(2);
                std::cout << "Robot Connect!" << std::endl;
                return true;
            }
            else
            {std::cout << "Cannot Enable Robot!" << std::endl;}
        }
        else
        {std::cout << "Cannot Power On!" << std::endl;}
        
    }
    else
    {std::cout << "Cannot Login In!" << std::endl;}
    return false;    
}

void vel_run(const jakarobot::a::ConstPtr &msg)//
// bool velocity_servo(JAKAZuRobot& jaka_robot, double *velocity)
{
    //std::cout << "11111"  << std::endl;
    double delta_t=0.04;//单步的时间长度8ms
    JointValue joint_pos;
    //JointValue real_joint_pos;
    //JointValue origin_joint_pos;
    //jaka_robot.get_joint_position(&origin_joint_pos);// 记录初始关节角

    RobotStatus status;
    JointValue origin_joint_pos;
    JointValue after_joint_pos;
    jaka_robot.get_robot_status(&status);
    origin_joint_pos.jVal[5] = status.joint_position[5];

    joint_pos.jVal[0] = msg->vel_1 *delta_t;
    joint_pos.jVal[1] = msg->vel_2 *delta_t;
    joint_pos.jVal[2] = msg->vel_3 *delta_t;
    joint_pos.jVal[3] = msg->vel_4 *delta_t;
    joint_pos.jVal[4] = msg->vel_5 *delta_t;
    joint_pos.jVal[5] = msg->vel_6 *delta_t;
    // joint_pos.jVal[5] = velocity[5]*delta_t;
    unsigned int i = 0; 
    //for(i = 0; i<5; i++)
    //{
        if(jaka_robot.servo_j(&joint_pos, INCR, 5))//INCR相对运动;ABS绝对运动
        {
             std::cout << "servo error!"<<std::endl;
        }    
    //}
    jaka_robot.get_robot_status(&status);
    after_joint_pos.jVal[5] = status.joint_position[5];
    //jaka_robot.get_joint_position(&real_joint_pos);// 记录实际关节角
    //std::cout << "desire joint:" << joint_pos.jVal[5] << std::endl;
    //std::cout << "real joint:" << after_joint_pos.jVal[5] - origin_joint_pos.jVal[5] << std::endl;
    real_joint.data = (after_joint_pos.jVal[5] - origin_joint_pos.jVal[5]);//(float)
    desire_joint.data = joint_pos.jVal[5];
    //return true;
}

//断开连接JAKA机械臂
void Disconnect_Robot(JAKAZuRobot& robot)
{
    if(!robot.disable_robot())
    {
        if(!robot.power_off())
        {
            if(!robot.login_out())
            {sleep(1);
             std::cout << "Robot Shutdown!" << std::endl;}
            else
            {std::cout << "Cannot Login Out!" << std::endl;}
        }
        else
        {std::cout << "Cannot Power Off!" << std::endl;}
        
    }
    else
    {
        std::cout << "Cannot Disable Robot!" << std::endl;
    }
}


int main(int argc, char** argv)
{ 
    //JAKAZuRobot jaka_robot;
    std::string robot_IP = "10.5.5.100";    
    if(!Connect_Robot(jaka_robot, robot_IP)) //连接机器人
    {
        std::cout << "Fail to connect JAKA!" << std::endl;
        return -1;
    }
    sleep(1);

    if(!jaka_robot.servo_move_enable(TRUE))
    {std::cout <<"进入伺服模式"<< std::endl;}

    ros::init(argc,argv,"velocity_listener");
    ros::NodeHandle h;//创建节点句柄
    //创建subscriber 接收消息 响应chatterCallBack
    ros::Subscriber listener_sub = h.subscribe("velocity_talker", 50, vel_run);
    ros::Publisher desire_pub = h.advertise<std_msgs::Float64>("desire",200);
    ros::Publisher real_pub = h.advertise<std_msgs::Float64>("real",200);
    ros::Rate looprate(25);
    //循环等待回调函数
    //ros::spin();
    //ros::Rate loop_rate(25);
    while(ros::ok())
    {
        
        //loop_rate.sleep();
        desire_pub.publish(desire_joint);
        real_pub.publish(real_joint);
        ros::spinOnce();
    }


    //读取数据测试
    // RobotStatus status;
    // CartesianPose robotpose;
    // JointValue robotjoints;
    // jaka_robot.get_robot_status(&status);
    // robotpose.tran.x = status.cartesiantran_position[0];
    // robotpose.rpy.rx = status.cartesiantran_position[3];        
    // robotjoints.jVal[0] = status.joint_position[0];
    // std::cout << "Joint1: " << robotjoints.jVal[0] << std::endl;
    // std::cout << "x: " << robotpose.tran.x << std::endl;
    // std::cout << "rx: " << robotpose.rpy.rx << std::endl;

#ifdef ROBOT_SHUTDOWN
    Disconnect_Robot(jaka_robot);
#endif

    return 0;
}
