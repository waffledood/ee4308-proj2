#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <fstream>
#include "common.hpp"

bool target_changed = false;
Position target;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target.x = msg->point.x;
    target.y = msg->point.y;
}

Position pos_rbt(0, 0);
double ang_rbt = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    pos_rbt.x = p.x;
    pos_rbt.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ang_rbt = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    std::string dir;
    nh.getParam("dir", dir);

    std::ofstream data_file;
    data_file.open("/home/selva/team08/data.text");
    data_file.open(dir + "/data.txt");
    
    // Get ROS Parameters
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" TMOVE : Param enable_move not found, set to true");
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" TMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" TMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" TMOVE : Param Kd_lin not found, set to 0");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 0.22))
        ROS_WARN(" TMOVE : Param max_lin_vel not found, set to 0.22");
    double max_lin_acc;
    if (!nh.param("max_lin_acc", max_lin_acc, 1.0))
        ROS_WARN(" TMOVE : Param max_lin_acc not found, set to 1");
    double Kp_ang;
    if (!nh.param("Kp_ang", Kp_ang, 1.0))
        ROS_WARN(" TMOVE : Param Kp_ang not found, set to 1.0");
    double Ki_ang;
    if (!nh.param("Ki_ang", Ki_ang, 0.0))
        ROS_WARN(" TMOVE : Param Ki_ang not found, set to 0");
    double Kd_ang;
    if (!nh.param("Kd_ang", Kd_ang, 0.0))
        ROS_WARN(" TMOVE : Param Kd_ang not found, set to 0");
    double max_ang_vel;
    if (!nh.param("max_ang_vel", max_ang_vel, 2.84))
        ROS_WARN(" TMOVE : Param max_ang_vel not found, set to 2.84");
    double max_ang_acc;
    if (!nh.param("max_ang_acc", max_ang_acc, 4.0))
        ROS_WARN(" TMOVE : Param max_ang_acc not found, set to 4");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" TMOVE : Param move_iter_rate not found, set to 25");

    // Subscribers
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // prepare published messages
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // Setup rate
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic

    // wait for other nodes to load
    ROS_INFO(" TMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ang_rbt == 10) // not dependent on main.cpp, but on motion.cpp
    {
        rate.sleep();
        ros::spinOnce(); //update the topics
    }

    // Setup variables
    double cmd_lin_vel = 0, cmd_ang_vel = 0;
    double dt;
    double prev_time = ros::Time::now().toSec();

    ////////////////// DECLARE VARIABLES HERE //////////////////
    double pos_error = 0, ang_error = 0, pos_error_prev = 0, ang_error_prev = 0;
    double P_lin, I_lin, D_lin;
    double P_ang, I_ang, D_ang;
    double lin_acc, constrained_lin_acc;
    double cmd_lin_vel_prev = 0;
    double ang_acc, constrained_ang_acc;
    double cmd_ang_vel_prev = 0;
    Position target_alt;
    double direction , prop;

    ROS_INFO(" TMOVE : ===== BEGIN =====");

    // main loop
    if (enable_move)
    {
        while (ros::ok() && nh.param("run", true))
        {
            // update all topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION CONTROLLER HERE //////////////////
            
            // Computing PID for linear velocity //
            pos_error = dist_euc(pos_rbt, target);
            P_lin = Kp_lin * pos_error;
            I_lin += (Ki_lin * dt);
            D_lin = Kd_lin * (pos_error - pos_error_prev) / dt;
            cmd_lin_vel = P_lin + I_lin + D_lin;

            // Computing PID for angular velocity //
            ang_error = limit_angle(heading(pos_rbt, target) - ang_rbt);
            if (ang_error < (-0.5 * M_PI)) {
                ang_error += M_PI;
                direction = -1.0;
            } else if (ang_error > (0.5 * M_PI)){
                ang_error -= M_PI;
                direction = -1.0;
            } else {
                direction = 1.0;
            }

            P_ang = Kp_ang * ang_error;
            I_ang += (Ki_ang * dt);
            D_ang = Kd_ang * (ang_error - ang_error_prev) / dt;
            cmd_ang_vel = P_ang + I_ang + D_ang;

            cmd_lin_vel *= (1.0 - abs(ang_error/M_PI))*direction;

            // Constraint for linear velocity //
            lin_acc = (cmd_lin_vel - cmd_lin_vel_prev) / dt;
            constrained_lin_acc = sat(lin_acc, max_lin_acc);
            cmd_lin_vel = sat(cmd_lin_vel + constrained_lin_acc * dt, max_lin_vel);              

            // Constraint for angular velocity //
            ang_acc = (cmd_ang_vel - cmd_ang_vel_prev) / dt;
            constrained_ang_acc = sat(ang_acc, max_ang_acc);
                cmd_ang_vel = sat(cmd_ang_vel + constrained_ang_acc * dt, max_lin_acc);

                // Coupling linear velocity with angular error //

                //  if (ang_error < M_PI/4 && ang_error > -M_PI/4) {
                    
                //     prop = 1;
                //     cmd_lin_vel *= prop;

                // } else {

                //     prop = ang_error / M_PI;
                //     if (ang_error > 0) {
                //         cmd_lin_vel *= 1-prop;
                //     } else {
                //         cmd_lin_vel *= -(1+prop);
                //     }
                //     ROS_INFO("BACKWARD MOVEMENT"); 
                // }

                // Updating variables tracking variables' previous occurrences //
                pos_error_prev = pos_error;
                ang_error_prev = ang_error;
                cmd_lin_vel_prev = cmd_lin_vel;

                // publish speeds //
                msg_cmd.linear.x = cmd_lin_vel;
                msg_cmd.angular.z = cmd_ang_vel;
                pub_cmd.publish(msg_cmd);

                // write to file
                data_file << ros::Time::now().toSec() << "\t" << pos_error << "\t" << ang_error << "\t" << cmd_lin_vel << "\t" << cmd_ang_vel << "\t" << prop << std::endl;               
            // }

            // verbose
            if (verbose)
            {
                ROS_INFO(" TMOVE : FV(%2.3f) AV(%2.3f)", cmd_lin_vel, cmd_ang_vel);
                ROS_INFO(" pos_error: (%2.3f), ang_error: (%2.3f)", pos_error, ang_error);
                ROS_INFO(" Target_alt: (%3.3f, %3.3f)", target_alt.x, target_alt.y);
                ROS_INFO(" Target: (%3.3f, %3.3f)", target.x, target.y);
                ROS_INFO(" Pose_rbt: (%3.3f, %3.3f)", pos_rbt.x, pos_rbt.y);
            }

            // wait for rate
            rate.sleep();
        }
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    ROS_INFO(" TMOVE : ===== END =====");
    return 0;
}
