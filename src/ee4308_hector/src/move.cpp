#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <signal.h>
#include "common.hpp"
#define NaN std::numeric_limits<double>::quiet_NaN()

ros::ServiceClient en_mtrs;
void disable_motors(int sig)
{
    ROS_INFO(" HMOVE : Disabling motors...");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv); 
}

double target_x = NaN, target_y = NaN, target_z = NaN;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target_x = msg->point.x;
    target_y = msg->point.y;
    target_z = msg->point.z;
}

double x = NaN, y = NaN, z = NaN, a = NaN;
void cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    auto &p = msg->pose.pose.position;
    x = p.x;
    y = p.y;
    z = p.z;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    a = atan2(siny_cosp, cosy_cosp);
}

bool rotate = false;
void cbRotate(const std_msgs::Bool::ConstPtr &msg)
{
    rotate = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // --------- parse parameters ----------
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" HMOVE : Param enable_move not found, set to true");
    if (!enable_move)
        return 0;
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" HMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" HMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    double Kp_z;
    if (!nh.param("Kp_z", Kp_z, 1.0))
        ROS_WARN(" HMOVE : Param Kp_z not found, set to 1.0");
    double Ki_z;
    if (!nh.param("Ki_z", Ki_z, 0.0))
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    double Kd_z;
    if (!nh.param("Kd_z", Kd_z, 0.0))
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    double yaw_rate;
    if (!nh.param("yaw_rate", yaw_rate, 0.5))
        ROS_WARN(" HMOVE : Param yaw_rate not found, set to 0.5");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 2.0))
        ROS_WARN(" HMOVE : Param max_lin_vel not found, set to 2");
    double max_z_vel;
    if (!nh.param("max_z_vel", max_z_vel, 0.5))
        ROS_WARN(" HMOVE : Param max_z_vel not found, set to 0.5");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" HMOVE : Param move_iter_rate not found, set to 25");
    
    // --------- Enable Motors ----------
    ROS_INFO(" HMOVE : Enabling motors...");
    en_mtrs = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = true;
    if (en_mtrs.call(en_mtrs_srv))
        ROS_INFO(" HMOVE : Motors enabled!");
    else
        ROS_WARN(" HMOVE : Cannot enable motors!");
    signal(SIGINT, disable_motors);

    // --------- Subscribers ----------
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);
    ros::Subscriber sub_rotate = nh.subscribe("rotate", 1, &cbRotate);

    // --------- Publishers ----------
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // --------- Wait for Topics ----------
    ROS_INFO(" HMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && (std::isnan(target_x) || std::isnan(x))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce(); // update the topics

    // --------- Begin Controller ----------
    ROS_INFO(" HMOVE : ===== BEGIN =====");
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic
    double cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z, cmd_lin_vel_a;
    double dt;
    double prev_time = ros::Time::now().toSec();

    double x_pos_error, y_pos_error, z_pos_error;
    double past_x_pos_error = 0, past_y_pos_error = 0, past_z_pos_error = 0;
    double total_x = 0, total_y = 0, total_z = 0;
    double x_portional, x_integal, x_derviate;
    double y_portional, y_integal, y_derviate;
    double z_portional, z_integal, z_derviate;
    
    double pos_error, total_error = 0;
    double total_portional, total_integal, total_derviate;
    double past_pos_error = 0;
    double cmd_total_lin_vel;

    double c_theta, s_theta;

    // main loop
    while (ros::ok() && nh.param("run", true))
    {
        // update all topics
        ros::spinOnce();

        dt = ros::Time::now().toSec() - prev_time;
        if (dt == 0) // ros doesn't tick the time fast enough
            continue;
        prev_time += dt;

        // publish speeds
        msg_cmd.linear.x = cmd_lin_vel_x;
        msg_cmd.linear.y = cmd_lin_vel_y;
        msg_cmd.linear.z = cmd_lin_vel_z;
        msg_cmd.angular.z = cmd_lin_vel_a;
        pub_cmd.publish(msg_cmd);

        //// IMPLEMENT /////
        
        // For tuning purposes
        //target_x = 4.0;
        //target_y = 4.0;
        
        // Frame Transformation (from absolute to drone frame)
        c_theta = cos(-a);
        s_theta = sin(-a);

        // Find position error between target and current coordinates
        x_pos_error = c_theta * (target_x - x) - s_theta * (target_y - y);
        y_pos_error = s_theta * (target_x - x) + c_theta * (target_y - y);
        z_pos_error = target_z - z;
        pos_error = dist_euc(x, y, target_x, target_y);

        // Sumation of position errors
        total_x += x_pos_error;
        total_y += y_pos_error;
        total_z += z_pos_error;
        total_error += pos_error;

        // PID for total component
        total_portional = Kp_lin * pos_error;
        total_integal = Ki_lin * total_error;
        total_derviate = Kd_lin * ((pos_error - past_pos_error)/dt);

        // PID for x component
        x_portional = Kp_lin * x_pos_error;
        x_integal = Ki_lin * total_x;
        x_derviate = Kd_lin * ((x_pos_error - past_x_pos_error)/dt);

        // PID for y component
        y_portional = Kp_lin * y_pos_error;
        y_integal = Ki_lin * total_y;
        y_derviate = Kd_lin * ((y_pos_error - past_y_pos_error)/dt);

        // PID for z component
        z_portional = Kp_z * z_pos_error;
        z_integal = Ki_z * total_z;
        z_derviate = Kd_z * ((z_pos_error - past_z_pos_error)/dt);

        // Store pervious position errors
        past_x_pos_error = x_pos_error;
        past_y_pos_error = y_pos_error;
        past_z_pos_error = z_pos_error;
        past_pos_error = pos_error;
        
        // Calculate final control signal
        cmd_lin_vel_x = x_portional + x_integal + x_derviate;
        cmd_lin_vel_y = y_portional + y_integal + y_derviate;
        cmd_lin_vel_z = z_portional + z_integal + z_derviate;
        cmd_total_lin_vel = total_derviate + total_integal + total_portional;

        // State Conditional Check 
        if (rotate) 
        {                              
            cmd_lin_vel_a = yaw_rate;
            if (cmd_total_lin_vel > max_lin_vel) 
            {
                cmd_lin_vel_x *= max_lin_vel/cmd_total_lin_vel;
                cmd_lin_vel_y *= max_lin_vel/cmd_total_lin_vel;
                cmd_lin_vel_z = sat(cmd_lin_vel_z, max_z_vel);    
            } else if (cmd_total_lin_vel < - max_lin_vel) 
            {
                cmd_lin_vel_x *= -max_lin_vel/cmd_total_lin_vel;
                cmd_lin_vel_y *= -max_lin_vel/cmd_total_lin_vel;
                cmd_lin_vel_z = sat(cmd_lin_vel_z, max_z_vel);  
            } else 
            {
                cmd_lin_vel_z = sat(cmd_lin_vel_z, max_z_vel);      
            }
        } else 
        {
            cmd_lin_vel_x = 0;
            cmd_lin_vel_y = 0;
            cmd_lin_vel_z = sat(cmd_lin_vel_z, max_z_vel); 
            cmd_lin_vel_a = 0;
        } 

        // verbose
        if (verbose)
        {
            ROS_INFO(" HMOVE : Target(%6.3f, %6.3f, %6.3f) VX(%6.3f) VY(%6.3f) VZ(%7.3f)", target_x, target_y, target_z, cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z);
            ROS_INFO(" Z_POSE_ERROR: %6.3f", z_pos_error); 
            ROS_INFO(" X_POSE %6.3f , Y_POSE: %6.3f , Z_POSE: %6.3f" , x, y, z);
        }

        // wait for rate
        rate.sleep();
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.linear.y = 0;
    msg_cmd.linear.z = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    // disable motors
    ROS_INFO(" HMOVE : Motors Disabled");
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv);

    ROS_INFO(" HMOVE : ===== END =====");

    return 0;
}