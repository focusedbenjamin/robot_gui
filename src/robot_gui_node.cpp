#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <std_srvs/Trigger.h>

#include <opencv2/opencv.hpp>
#define CVUI_IMPLEMENTATION
#include "cvui.h"

#define WINDOW_NAME "Robot GUI"

class RobotGUI {
public:
    RobotGUI() {
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        robot_info_sub_ = nh_.subscribe("/robot_info", 10,
            &RobotGUI::robotInfoCallback, this);

        odom_sub_ = nh_.subscribe("/odom", 10,
            &RobotGUI::odomCallback, this);

        distance_client_ = nh_.serviceClient<std_srvs::Trigger>("/get_distance");
        distance_client_.waitForExistence();

        // Initialize values
        linear_x_ = 0.0;
        angular_z_ = 0.0;
        pos_x_ = pos_y_ = pos_z_ = 0.0;
        robot_info_text_ = "";
        distance_text_ = "";

        cv::namedWindow(WINDOW_NAME);
        cvui::init(WINDOW_NAME);
    }

    void spin() {
        cv::Mat frame = cv::Mat(600, 800, CV_8UC3);

        while (ros::ok()) {
            frame = cv::Scalar(49, 52, 49);

            drawGUI(frame);

            cvui::update();
            cv::imshow(WINDOW_NAME, frame);

            publishVelocity();

            ros::spinOnce();

            if (cv::waitKey(20) == 27) break;
        }
    }

private:
    ros::NodeHandle nh_;

    ros::Publisher cmd_pub_;
    ros::Subscriber robot_info_sub_;
    ros::Subscriber odom_sub_;
    ros::ServiceClient distance_client_;

    geometry_msgs::Twist cmd_msg_;

    double linear_x_, angular_z_;
    double pos_x_, pos_y_, pos_z_;

    std::string robot_info_text_;
    std::string distance_text_;

    // -------- CALLLLLBACKS --------
    void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& msg) {
        robot_info_text_ = msg->data_field_01;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        pos_x_ = msg->pose.pose.position.x;
        pos_y_ = msg->pose.pose.position.y;
        pos_z_ = msg->pose.pose.position.z;
    }

    // -------- GUI --------
    void drawGUI(cv::Mat& frame) {

        // INFO
        cvui::text(frame, 20, 20, "Robot Info:");
        cvui::text(frame, 20, 50, robot_info_text_);

        // TELEOPOPERATIONS
        cvui::text(frame, 20, 100, "Teleoperation");

        if (cvui::button(frame, 20, 130, "Forward")) linear_x_ += 0.1;
        if (cvui::button(frame, 120, 130, "Backward")) linear_x_ -= 0.1;
        if (cvui::button(frame, 20, 180, "Left")) angular_z_ += 0.1;
        if (cvui::button(frame, 120, 180, "Right")) angular_z_ -= 0.1;

        if (cvui::button(frame, 70, 230, "STOP")) {
            linear_x_ = 0.0;
            angular_z_ = 0.0;
        }

        // VELOCITY DISPLAY
        cvui::text(frame, 20, 280, "Linear Velocity: " + std::to_string(linear_x_));
        cvui::text(frame, 20, 310, "Angular Velocity: " + std::to_string(angular_z_));

        // ODOMETRY
        cvui::text(frame, 20, 360, "Position:");
        cvui::text(frame, 20, 390, "X: " + std::to_string(pos_x_));
        cvui::text(frame, 20, 420, "Y: " + std::to_string(pos_y_));
        cvui::text(frame, 20, 450, "Z: " + std::to_string(pos_z_));

        // SERVICE
        if (cvui::button(frame, 20, 500, "Get Distance")) {
            std_srvs::Trigger srv;
            if (distance_client_.call(srv)) {
                distance_text_ = srv.response.message;
            }
        }

        cvui::text(frame, 20, 540, "Distance: " + distance_text_);
    }

    // -------- PUBLISH --------
    void publishVelocity() {
        cmd_msg_.linear.x = linear_x_;
        cmd_msg_.angular.z = angular_z_;
        cmd_pub_.publish(cmd_msg_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_gui_node");

    RobotGUI gui;
    gui.spin();

    return 0;
}