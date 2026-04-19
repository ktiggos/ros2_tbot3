#include "tbot3_webots/tbot3_driver.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cstdio>
#include <functional>
#include <cmath>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>
#include <webots/supervisor.h>

#include <pluginlib/class_list_macros.hpp>

// Values taken from URDF
#define WHEEL_HALF_DIS 0.144
#define WHEEL_RAD 0.033

#define LINEAR_COEFF 0.33/10.0
#define ANGULAR_COEFF 2.29/10.0

#define USE_SPEED_NORM true
#define FAKE_LOCALIZATION true

PLUGINLIB_EXPORT_CLASS(tb3_driver::Tb3Driver, webots_ros2_driver::PluginInterface);

void tb3_driver::Tb3Driver::cmdVelCB(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vel_msg.linear = msg->linear;
    cmd_vel_msg.angular = msg->angular;
}

void tb3_driver::Tb3Driver::init(webots_ros2_driver::WebotsNode *node,
                                 std::unordered_map<std::string, std::string> &parameters)
{
    this->node_ = node;
    self_node = wb_supervisor_node_get_self();

    RCLCPP_INFO(node_->get_logger(),"Initializing differential driver for Turtlebot...");
    
    left_motor = wb_robot_get_device("wheel_left_joint");
    right_motor = wb_robot_get_device("wheel_right_joint");
    lm_sensor = wb_robot_get_device("wheel_left_joint_sensor");
    rm_sensor = wb_robot_get_device("wheel_right_joint_sensor");

    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);

    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(right_motor, 0.0);

    wb_position_sensor_enable(lm_sensor, 10);
    wb_position_sensor_enable(rm_sensor, 10);

    RCLCPP_INFO(node_->get_logger(), "Initializing odometry measurements...");
    odom_msg.pose.pose.position = geometry_msgs::msg::Point(
        MessageInit::ZERO
    );
    odom_msg.pose.pose.orientation = geometry_msgs::msg::Quaternion(
        MessageInit::ZERO
    );
    odom_msg.pose.pose.orientation.w = 1.0;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    cb_time = node_->get_clock()->now();
    driver_time = node_->get_clock()->now();
    dtime_last = node_->get_clock()->now();

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", 10
    );

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::SensorDataQoS().reliable(),
        [this](geometry_msgs::msg::Twist::SharedPtr msg) -> void {
            this->cmdVelCB(msg);
            cb_time = node_->get_clock()->now();
        }
    );
}

void tb3_driver::Tb3Driver::step(){
    rclcpp::Time now = node_->get_clock()->now();

    driver_time = now;
    
    double dt = (now - dtime_last).seconds();
    this->dtime_last = now;

    bool stale{ (driver_time - cb_time) > 
        rclcpp::Duration::from_seconds(timeout)
    };

    double fwd_speed = stale ? 0.0 : cmd_vel_msg.linear.x;
    double ang_speed = stale ? 0.0 : cmd_vel_msg.angular.z;

    if(USE_SPEED_NORM){
        fwd_speed = fwd_speed * LINEAR_COEFF;
        ang_speed = ang_speed * ANGULAR_COEFF;
    }

    double lm_cmd{ (fwd_speed - ang_speed * WHEEL_HALF_DIS) / WHEEL_RAD };
    double rm_cmd{ (fwd_speed + ang_speed * WHEEL_HALF_DIS) / WHEEL_RAD };

    lm_cmd = std::abs(lm_cmd) < 10.0 ? lm_cmd : (lm_cmd/std::abs(lm_cmd))*10.0;
    rm_cmd = std::abs(rm_cmd) < 10.0 ? rm_cmd : (rm_cmd/std::abs(rm_cmd))*10.0;

    wb_motor_set_velocity(left_motor, lm_cmd);
    wb_motor_set_velocity(right_motor, rm_cmd);

    double ldis{ wb_position_sensor_get_value(lm_sensor) * WHEEL_RAD };
    double rdis{ wb_position_sensor_get_value(rm_sensor) * WHEEL_RAD };

    double dldis{ ldis - ldis_last };
    double drdis{ rdis - rdis_last };
    ldis_last = ldis;
    rdis_last = rdis;

    double dis_avg{ (dldis + drdis) / 2.0 };
    double dtheta{ (drdis - dldis) / (WHEEL_HALF_DIS * 2.0) };

    theta += dtheta;

    double lin{0.0}, ang{0.0};
    if(FAKE_LOCALIZATION){
        const double *p = wb_supervisor_node_get_position(self_node);
        const double *R = wb_supervisor_node_get_orientation(self_node);

        double x = p[0];
        double y = p[1];
        double z = p[2];

        double yaw = std::atan2(R[3], R[0]);

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        if (dt > 1e-6 && !gt_first) {
            double vx_world = (x - gt_x_last) / dt;
            double vy_world = (y - gt_y_last) / dt;
            double dyaw = yaw - gt_yaw_last;

            while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
            while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

            lin = std::cos(yaw) * vx_world + std::sin(yaw) * vy_world;
            ang = dyaw / dt;
        } else {
            lin = 0.0;
            ang = 0.0;
        }

        gt_x_last = x;
        gt_y_last = y;
        gt_yaw_last = yaw;
        gt_first = false;
    } else {
        odom_msg.pose.pose.position.x += dis_avg * std::cos(theta + 0.5 * dtheta);
        odom_msg.pose.pose.position.y += dis_avg * std::sin(theta + 0.5 * dtheta);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        if (dt > 1e-6) {
            lin = dis_avg / dt;
            ang = dtheta / dt;
        } else {
            lin = 0.0;
            ang = 0.0;
        }
    }

    if(USE_SPEED_NORM){
        lin = lin / LINEAR_COEFF;
        ang = ang / ANGULAR_COEFF;
    }

    odom_msg.twist.twist.linear.x = lin;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = ang;

    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    
    tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    odom_msg.header.stamp = now;
    tf_msg.header.stamp = odom_msg.header.stamp;
    odom_pub_->publish(odom_msg);
    tf_broadcaster_->sendTransform(tf_msg);
}