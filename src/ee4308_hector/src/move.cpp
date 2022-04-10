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
#include <std_msgs/Float64.h>
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

    std::string dir;
    nh.getParam("dir", dir);

    std::ofstream data_file;
    data_file.open(dir + "/XandY.txt");

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

    double x_error = 0;
    double x_prev_error = 0;
    double x_error_total = 0;

    double y_error = 0;
    double y_prev_error = 0;
    double y_error_total = 0;

    double z_error = 0;
    double z_prev_error = 0;
    double z_error_total = 0;

    double combined_vel;
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

        // For tuning 
        //target_x = 4;
        //target_y = 4;
        
        // Frame transformation
        c_theta = cos(-a);
        s_theta = sin(-a);

        // X axis
        double x_p = 0, x_i = 0, x_d = 0;
        double temp_signal_x = 0, acc_x = 0;

        x_prev_error = x_error;
        x_error = c_theta * (target_x - x) - s_theta * (target_y - y);
        x_p = Kp_lin * x_error;

        x_error_total += (x_error * dt);
        x_i = Ki_lin * x_error_total;

        x_d = Kd_lin * ((x_error - x_prev_error)/dt);

        temp_signal_x = x_p + x_i +x_d;

        acc_x = (temp_signal_x - cmd_lin_vel_x)/dt;
        cmd_lin_vel_x += (acc_x * dt);

        // Y axis
        double y_p = 0, y_i = 0, y_d = 0;
        double temp_signal_y = 0, acc_y = 0;
        
        y_prev_error = y_error;
        y_error = s_theta * (target_x - x) + c_theta * (target_y - y);
        y_p = Kp_lin * y_error;

        y_error_total += (y_error * dt);
        y_i = Ki_lin * y_error_total;

        y_d = Kd_lin * ((y_error - y_prev_error)/dt);
        
        temp_signal_y = y_p + y_i + y_d;

        acc_y = (temp_signal_y - cmd_lin_vel_y)/dt;
        cmd_lin_vel_y += (acc_y * dt);
        

        // Z axis
        double z_p = 0, z_i = 0, z_d = 0;
        double temp_signal_z = 0, acc_z = 0;
        
        z_prev_error = z_error;
        z_error = target_z - z;
        z_p = Kp_z * z_error;

        z_error_total += (z_error * dt);
        z_i = Ki_z * z_error_total;

        z_d = Kd_z * ((z_error - z_prev_error)/dt);
        
        temp_signal_z = z_p + z_i + z_d;

        acc_z = (temp_signal_z - cmd_lin_vel_z)/dt;
        cmd_lin_vel_z += (acc_z * dt);

        cmd_lin_vel_z = sat(cmd_lin_vel_z + (acc_z * dt), max_z_vel);


        // Velocity Check
        combined_vel = sqrt(pow(cmd_lin_vel_x, 2.0) + pow(cmd_lin_vel_y, 2));
        if (combined_vel > max_lin_vel) 
        {
            cmd_lin_vel_x = cmd_lin_vel_x * max_lin_vel / combined_vel;
            cmd_lin_vel_y = cmd_lin_vel_y * max_lin_vel / combined_vel;
        }

        /*
        // Rotate check
        if(rotate) {
            cmd_lin_vel_a = yaw_rate;
        } else {
            cmd_lin_vel_a = 0;
        }
        */

        //data_file << ros::Time::now().toSec() << "\t" << x_pos_error << "\t" << y_pos_error << std::endl;

        // verbose
        if (verbose)
        {
            ROS_INFO(" HMOVE : Target(%6.3f, %6.3f, %6.3f) VX(%6.3f) VY(%6.3f) VZ(%7.3f)", target_x, target_y, target_z, cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z);
            //ROS_INFO(" X POSE ERROR: %6.3f , Y POSE ERROR: %6.3f , Z POSE ERROR: %6.3f " , x_pos_error, y_pos_error, z_pos_error);
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

    data_file.close();
    
    return 0;

}