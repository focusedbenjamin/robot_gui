#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#include <ros/ros.h>


#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <std_srvs/Trigger.h>


#include <opencv2/opencv.hpp>

class RobotGUI {
public:
    RobotGUI();     
    void spin();   

private:
    
    ros::NodeHandle nh_;

    ros::Publisher cmd_pub_;
    ros::Subscriber robot_info_sub_;
    ros::Subscriber odom_sub_;
    ros::ServiceClient distance_client_;

    
    geometry_msgs::Twist cmd_msg_;

    double linear_x_;
    double angular_z_;

    double pos_x_;
    double pos_y_;
    double pos_z_;

    std::string robot_info_text_;
    std::string distance_text_;

    
    void robotInfoCallback(
        const robotinfo_msgs::RobotInfo10Fields::ConstPtr& msg);

    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg);

    
    void drawGUI(cv::Mat& frame);

    
    void publishVelocity();
};

#endif