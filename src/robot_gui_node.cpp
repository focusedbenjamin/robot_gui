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
        reset_client_ = nh_.serviceClient<std_srvs::Trigger>("/reset_distance");
        distance_client_.waitForExistence();

        // Initialize thvalues
        linear_x_ = 0.0;
        angular_z_ = 0.0;
        pos_x_ = pos_y_ = pos_z_ = 0.0;
        robot_info_text_ = "";
        distance_text_ = "";

        cv::namedWindow(WINDOW_NAME);
        cvui::init(WINDOW_NAME);
    }

    void spin() {
        cv::Mat frame = cv::Mat(850, 600, CV_8UC3);

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
    ros::ServiceClient reset_client_;

    geometry_msgs::Twist cmd_msg_;

    double linear_x_, angular_z_;
    double pos_x_, pos_y_, pos_z_;

    std::string robot_info_text_;
    std::string distance_text_;

    // CALLLLLBACKS --------
    void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& msg) {
    robot_info_text_ =
        msg->data_field_01 + "\n" +
        msg->data_field_02 + "\n" +
        msg->data_field_03 + "\n" +
        msg->data_field_04 + "\n" +
        msg->data_field_05 + "\n" +
        msg->data_field_06 + "\n" +
        msg->data_field_07 + "\n" +
        msg->data_field_08 + "\n" +
        msg->data_field_09 + "\n" +
        msg->data_field_10;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
            pos_x_ = msg->pose.pose.position.x;
            pos_y_ = msg->pose.pose.position.y;
            pos_z_ = msg->pose.pose.position.z;
    }

        // -------- GUI --------
    void drawGUI(cv::Mat& frame) {

        //----INFO PANELLL============
        cvui::window(frame, 10, 10, 580, 200, "Info");

        int y = 40;
        std::stringstream ss(robot_info_text_);
        std::string line;

        while (std::getline(ss, line)) {
            cvui::text(frame, 30, y, line);
            y += 18;
        }

        // TELEOPERA PANEL =====================
        cvui::window(frame, 10, 220, 580, 250, "Teleoperation");

        // Forward(Mbele)
        if (cvui::button(frame, 250, 260, 100, 50, "Forward"))
            linear_x_ += 0.1;

        // Middle (Katikati) row
        if (cvui::button(frame, 150, 320, 100, 50, "Left"))
            angular_z_ += 0.1;

        if (cvui::button(frame, 250, 320, 100, 50, "Stop")) {
            linear_x_ = 0.0;
            angular_z_ = 0.0;
        }

        if (cvui::button(frame, 350, 320, 100, 50, "Right"))
            angular_z_ -= 0.1;

        // Backward(Nyuma)
        if (cvui::button(frame, 250, 380, 100, 50, "Backward"))
            linear_x_ -= 0.1;

        // ===================== VELOCITY ===========
        cvui::text(frame, 20, 490, "Linear velocity:");
        cvui::printf(frame, 20, 520, 0.7, 0xff0000, "%.2f m/sec", linear_x_);

        cvui::text(frame, 320, 490, "Angular velocity:");
        cvui::printf(frame, 320, 520, 0.7, 0xff0000, "%.2f rad/sec", angular_z_);

        // === ODOMETRY PANEL =====================
        cvui::text(frame, 20, 550, "Estimated robot position based off odometry");

        // X
        cvui::window(frame, 10, 580, 180, 120, "X");
        cvui::printf(frame, 50, 640, 1.0, 0xffffff, "%.0f", pos_x_);

        // Y
        cvui::window(frame, 210, 580, 180, 120, "Y");
        cvui::printf(frame, 250, 640, 1.0, 0xffffff, "%.0f", pos_y_);

        // Z
        cvui::window(frame, 410, 580, 180, 120, "Z");
        cvui::printf(frame, 450, 640, 1.0, 0xffffff, "%.0f", pos_z_);

        // ==------------------------------------------- DISTANCE SECTION =========
        cvui::window(frame, 10, 710, 280, 120, "Distance Travelled");

        if (cvui::button(frame, 30, 750, 100, 50, "Call")) {
            std_srvs::Trigger srv;
            if (distance_client_.call(srv)) {
                distance_text_ = srv.response.message;
            }
        }

        cvui::window(frame, 310, 710, 280, 120, "Distance in meters");
        cvui::printf(frame, 400, 770, 1.0, 0xffffff, "%s", distance_text_.c_str());
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