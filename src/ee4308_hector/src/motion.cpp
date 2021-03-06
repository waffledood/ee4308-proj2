#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> // publish to pose topic
#include <geometry_msgs/Vector3Stamped.h>            // subscribe to magnetic topic
#include <sensor_msgs/Imu.h>                         // subscribe to imu topic
#include <sensor_msgs/NavSatFix.h>                   // subscribe to GPS
#include <hector_uav_msgs/Altimeter.h>               // subscribe to barometer
#include <sensor_msgs/Range.h>                       // subscribe to sonar
#include <nav_msgs/Odometry.h>                       // subscribe to ground truth topic
#include <std_srvs/Empty.h>                          // Service to calrbrate motors
#include <opencv2/core/core.hpp>
#include "common.hpp"
#include <iostream>
#include <vector>
#include <fstream>
using namespace std;

#define NaN std::numeric_limits<double>::quiet_NaN()

// global parameters to be read from ROS PARAMs
bool verbose, use_ground_truth, enable_baro, enable_magnet, enable_sonar, enable_gps;
bool data_collection;

// others
bool ready = false; // signal to topics to begin

// --------- PREDICTION WITH IMU ----------
const double G = 9.8;
double prev_imu_t = 0;
cv::Matx21d X = {0, 0}, Y = {0, 0}; // see intellisense. This is equivalent to cv::Matx<double, 2, 1>
cv::Matx21d A = {0, 0};
cv::Matx31d Z = {0.178, 0, 0};
cv::Matx22d P_x = cv::Matx22d::ones(), P_y = cv::Matx22d::zeros();
cv::Matx22d P_a = cv::Matx22d::ones();
cv::Matx33d P_z = cv::Matx33d::ones();
double ua = NaN, ux = NaN, uy = NaN, uz = NaN;
double qa, qx, qy, qz;
double Xb = NaN, Yb = NaN, Zb = NaN, Ab = NaN;
bool bias = true;
vector<double> UX;
vector<double> UY;
vector<double> UZ;
vector<double> UA;
// see https://docs.opencv.org/3.4/de/de1/classcv_1_1Matx.html
void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (!ready)
    {
        prev_imu_t = msg->header.stamp.toSec();
        return;
    }

    // calculate time
    double imu_t = msg->header.stamp.toSec();
    double imu_dt = imu_t - prev_imu_t;
    prev_imu_t = imu_t;

    // read inputs
    ua = msg->angular_velocity.z;
    ux = msg->linear_acceleration.x;
    uy = msg->linear_acceleration.y;
    uz = msg->linear_acceleration.z;
    
    //// IMPLEMENT IMU ////

    //// additional variables ////
    // yaw
    double a = A(0,0);

    // covariance
    // UX.push_back(ux);
    // UY.push_back(uy);
    // UZ.push_back(uz);
    // UA.push_back(ua);
    // if (UX.size() > 100) {
    //     ROS_INFO("Variance: %1.5lf %1.5lf %1.5lf %1.5lf", calculate_var(UX), calculate_var(UY), calculate_var(UZ), calculate_var(UA));
    //     UX.erase(UX.begin());
    //     UY.erase(UY.begin());
    //     UZ.erase(UZ.begin());
    //     UA.erase(UA.begin());
    // }

    // x //
    cv::Matx22d Fx = {1, imu_dt, 0, 1};
    cv::Matx22d Wx = {-0.5 * pow(imu_dt, 2) * cos(a),  0.5 * pow(imu_dt, 2) * sin(a),
                                    -imu_dt * cos(a),                imu_dt * sin(a)};
    cv::Matx22d Qx = {qx, 0, 0, qy};
    cv::Matx21d Ux = {ux, uy};

    // y // 
    cv::Matx22d Wy = {-0.5*imu_dt*imu_dt*sin(a),  -0.5*imu_dt*imu_dt*cos(a),
                                 -imu_dt*sin(a),             -imu_dt*cos(a)};
    cv::Matx22d Qy = {qx, 0, 0, qy};
    cv::Matx21d Uy = {ux, uy};

    // z //
    cv::Matx31d Wz = {0.5 * pow(imu_dt, 2), imu_dt, 0};
    cv::Matx<double, 1,1> Qz = {qz};
    cv::Matx<double, 1, 1> Uz = {uz - G};
    cv::Matx33d Fz = {1, imu_dt, 0,
                      0, 1, 0,
                      0, 0, 1};

    // yaw //
    cv::Matx22d Fa = {1, 0, 0, 0};
    cv::Matx21d Wa = {imu_dt, 1};
    cv::Matx<double, 1,1> Qa = {qa};
    cv::Matx<double, 1,1> Ua = {ua};

    // EKF Prediction for State matrices //
    X = Fx * X + Wx * Ux;
    Y = Fx * Y + Wy * Uy;
    Z = Fz * Z + Wz * Uz;
    A = Fa * A + Wa * Ua;

    // EKF Prediction for Variance matrices //
    P_x = Fx * P_x * Fx.t() + Wx * Qx * Wx.t();
    P_y = Fx * P_y * Fx.t() + Wy * Qy * Wy.t();
    P_z = Fz * P_z * Fz.t() + Wz * Qz * Wz.t();
    P_a = Fa * P_a * Fa.t() + Wa * Qa * Wa.t();
}

// --------- GPS ----------
// https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html
cv::Matx31d GPS = {NaN, NaN, NaN};
cv::Matx31d initial_pos = {NaN, NaN, NaN}; // written below in main. no further action needed.
cv::Matx31d cov = {NaN, NaN, NaN};
const double DEG2RAD = M_PI / 180;
const double RAD_POLAR = 6356752.3;
const double RAD_EQUATOR = 6378137;
double r_gps_x, r_gps_y, r_gps_z;
double eSqr, pvrc; // e^2 and N(??)
cv::Matx31d initial_ECEF = {NaN, NaN, NaN};
cv::Matx31d ECEF = {NaN, NaN, NaN};
cv::Matx31d NED = {NaN, NaN, NaN};
void cbGps(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (!ready)
        return;

    //// IMPLEMENT GPS /////
    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;
    boost::array<double, 9> array_msg;

    lat *= DEG2RAD;
    lon *= DEG2RAD;

    eSqr = 1 - pow(RAD_POLAR,2) / pow(RAD_EQUATOR,2);
    pvrc = RAD_EQUATOR / sqrt(1 - eSqr * pow(sin(lat), 2));

    ECEF = {(pvrc + alt) * cos(lat) * cos(lon),
        (pvrc + alt) * cos(lat) * sin(lon), 
        (pow(RAD_POLAR,2) / pow(RAD_EQUATOR,2) * pvrc + alt) * sin(lat)};

    // for initial message -- you may need this:
    if (std::isnan(initial_ECEF(0)))
    {   // calculates initial ECEF and returns
        initial_ECEF = ECEF;
        return;
    }

    ECEF = {(ECEF(0) - initial_ECEF(0)), (ECEF(1) - initial_ECEF(1)), (ECEF(2) - initial_ECEF(2))};

    NED = {-sin(lat) * cos(lon) * ECEF(0) + -sin(lat) * sin(lon) * ECEF(1) + cos(lat) * ECEF(2),
        -sin(lon) * ECEF(0) + cos(lon) * ECEF(1),
        -cos(lat) * cos(lon) * ECEF(0) + -cos(lat) * sin(lon) * ECEF(1) + -sin(lon) * ECEF(2)};


    GPS = {NED(0) + initial_pos(0), -NED(1) + initial_pos(1), -NED(2) + initial_pos(2)};

    // EKF Correction for x, y & z
    double x_gps = GPS(0), y_gps = GPS(1), z_gps = GPS(2);
    cv::Matx21d Kx = {0, 0};
    cv::Matx21d Ky = {0, 0};
    cv::Matx31d Kz = {0, 0, 0};
    cv::Matx12d H = {1.0, 0};
    cv::Matx13d Hz = {1.0, 0, 0};
    cv::Matx<double,1,1> r_x = {r_gps_x};
    cv::Matx<double,1,1> r_y = {r_gps_y};
    cv::Matx<double,1,1> r_z = {r_gps_z};
    
    cv::Matx<double, 1, 1> M = (H*P_x*H.t() + r_x);
    Kx = P_x * H.t() * (1/M(0));
    X = X + Kx*(x_gps - X(0));
    P_x = P_x - Kx*H*P_x;    

    M = (H*P_y*H.t() + r_y);
    Ky = P_y * H.t() * (1/M(0));
    Y = Y + Ky*(y_gps - Y(0));
    P_y = P_y - Ky*H*P_y;

    // new Z state matrix & variance matrix
    M = (Hz*P_z*Hz.t() + r_x);
    Kz = P_z * Hz.t() * (1/M(0));
    Z = Z + Kz*(z_gps - Z(0));
    P_z = P_z - Kz*Hz*P_z;
}

// --------- Magnetic ----------
double a_mgn = NaN;
double r_mgn_a;
double init_mgn;
vector<double> magnetic;

void cbMagnet(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    if (!ready)
        return;
    
    // IMPLEMENT Magnetic ////
    double mx = msg->vector.x;
    double my = msg->vector.y;
    if (std::isnan(a_mgn)) {
        init_mgn = atan2(-my, mx);
    }
    a_mgn = atan2(-my, mx) - init_mgn;

    // Variables for EKF Correction (Magnetometer)
    cv::Matx21d kalman_gain_a = {0, 0};
    cv::Matx<double, 1, 2> H_a = {1.0, 0.0};
    cv::Matx<double, 1, 1> Y_a = {a_mgn};
    cv::Matx<double, 1,1> R_a = {r_mgn_a};

    // EKF Correction for a (yaw) state 
    cv::Matx<double, 1, 1> M = H_a*P_a*H_a.t() + R_a;
    kalman_gain_a = P_a*H_a.t()*(1/M(0));
    A = A + kalman_gain_a*(a_mgn - A(0));
    P_a = P_a - kalman_gain_a*H_a*P_a;

    // covariance
    if (data_collection) {
        magnetic.push_back(a_mgn);
        if (magnetic.size() > 100) {
            ROS_INFO("Magnetic Variance: %7.3lf", calculate_var(magnetic));
            magnetic.erase(magnetic.begin());
        }
    }
}

// --------- Baro ----------
double z_bar = NaN;
double r_bar_z;
double baroBias = NaN;
vector<double> baroCorrectedValues;
cv::Matx31d Z_new = {Z(0), Z(1), z_bar};

void cbBaro(const hector_uav_msgs::Altimeter::ConstPtr &msg)
{
    if (!ready)
        return;

    //// IMPLEMENT BARO ////
    z_bar = msg->altitude;

    // determine the value of the barometer bias
    if (std::isnan(baroBias)) {
        baroBias = z_bar - Z(0);
    }
    z_bar = z_bar - baroBias;
    Z(2) = baroBias;

    // Variables for EKF Correction (Baro)
    cv::Matx<double, 1, 3> H_z = {1.0, 0.0, 0.0};
    cv::Matx31d kalman_gain_z = {0, 0, 0};
    cv::Matx<double, 1, 1> r_z = {r_bar_z};

    // EKF Correction for z state
    cv::Matx<double, 1, 1> Mz = (H_z*P_z*H_z.t() + r_z);
    kalman_gain_z = P_z*H_z.t()* (1/Mz(0));
    Z = Z + kalman_gain_z*(z_bar-Z(0));
    P_z = P_z - kalman_gain_z*H_z*P_z;

    // debug print out the values of Z_new
    ROS_INFO("Z(0): %3.3lf, Z(1): %3.3lf, Z(2): %3.3lf", Z(0), Z(1), Z(2));

    // variance of Barometer
    if (data_collection) {
        baroCorrectedValues.push_back(z_bar);
        if (baroCorrectedValues.size() > 100) {
            ROS_INFO("Baro Variance: %lf", calculate_var_baro(baroCorrectedValues));
            baroCorrectedValues.erase(baroCorrectedValues.begin());
        }
    }

}

// --------- Sonar ----------
/* important note: sonar has a range of 3m & is affected 
   by heights of ground obstacles 
*/
double z_snr = NaN, z_snr_prev = NaN;
double r_snr_z;
vector<double> sonar;
bool takeoffDone = false;
void cbSonar(const sensor_msgs::Range::ConstPtr &msg)
{
    if (!ready)
        return;

    //// IMPLEMENT SONAR ////
    z_snr = msg->range;

    // Variables for EKF Correction (Sonar)
    cv::Matx31d kalman_gain_z = {0, 0, 0};
    cv::Matx<double, 1, 3> H_z = {1.0, 0.0, 0.0};
    cv::Matx<double, 1, 1> R_z = {r_snr_z};
    cv::Matx<double, 1, 1> Y_z = {z_snr};

    // EKF Correction for z
    cv::Matx<double, 1, 1> Mz = (H_z*P_z*H_z.t() + R_z);
    kalman_gain_z = P_z*H_z.t()* (1/Mz(0));
    Z = Z + kalman_gain_z*(z_snr- Z(0));
    P_z = P_z - kalman_gain_z*H_z*P_z;

    z_snr_prev = z_snr;

    // variance of Sonar
    if (data_collection) {
        sonar.push_back(z_snr);
        if (sonar.size() > 100) {
            ROS_INFO("Sonar Variance: %7.3lf", calculate_var(sonar));
            sonar.erase(sonar.begin());
        }
    }
}

nav_msgs::Odometry msg_true;
// --------- GROUND TRUTH ----------
void cbTrue(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_true = *msg;
}

// --------- MEASUREMENT UPDATE WITH GROUND TRUTH ----------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_motion");
    ros::NodeHandle nh;

    std::string dir;
    nh.getParam("dir", dir);
    std::ofstream data_file;
    data_file.open(dir + "/data.txt");

    // debug info for A
    ROS_INFO("[inside main] ua: %6.3lf, A(0): %6.3lf, A(1): %6.3lf", ua, A(0), A(1));

    // --------- parse parameters ----------
    double motion_iter_rate;
    if (!nh.param("motion_iter_rate", motion_iter_rate, 50.0))
        ROS_WARN("HMOTION: Param motion_iter_rate not found, set to 50.0");
    if (!nh.param("verbose_motion", verbose, false))
        ROS_WARN("HMOTION: Param verbose_motion not found, set to false");
    if (!nh.param("initial_x", X(0), 0.0))
        ROS_WARN("HMOTION: Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", Y(0), 0.0))
        ROS_WARN("HMOTION: Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", Z(0), 0.178))
        ROS_WARN("HMOTION: Param initial_z not found, set initial_z to 0.178");
    initial_pos = {X(0), Y(0), Z(0)};
    if (!nh.param("use_ground_truth", use_ground_truth, true))
        ROS_WARN("HMOTION: Param use_ground_truth not found, set use_ground_truth to true");
    if (!nh.param("r_gps_x", r_gps_x, 1.0))
        ROS_WARN("HMOTION: Param r_gps_x not found, set to 1.0");
    if (!nh.param("r_gps_y", r_gps_y, 1.0))
        ROS_WARN("HMOTION: Param r_gps_y not found, set to 1.0");
    if (!nh.param("r_gps_z", r_gps_z, 1.0))
        ROS_WARN("HMOTION: Param r_gps_z not found, set to 1.0");
    if (!nh.param("r_mgn_a", r_mgn_a, 1.0))
        ROS_WARN("HMOTION: Param r_mgn_a not found, set to 1.0");
    if (!nh.param("r_bar_z", r_bar_z, 1.0))
        ROS_WARN("HMOTION: Param r_bar_z not found, set to 1.0");
    if (!nh.param("r_snr_z", r_snr_z, 1.0))
        ROS_WARN("HMOTION: Param r_snr_z not found, set to 1.0");
    if (!nh.param("qa", qa, 1.0))
        ROS_WARN("HMOTION: Param qa not found, set to 1.0");
    if (!nh.param("qx", qx, 1.0))
        ROS_WARN("HMOTION: Param qx not found, set to 1.0");
    if (!nh.param("qy", qy, 1.0))
        ROS_WARN("HMOTION: Param qy not found, set to 1.0");
    if (!nh.param("qz", qz, 1.0))
        ROS_WARN("HMOTION: Param qz not found, set to 1.0");
    if (!nh.param("enable_baro", enable_baro, true))
        ROS_WARN("HMOTION: Param enable_baro not found, set to true");
    if (!nh.param("enable_magnet", enable_magnet, true))
        ROS_WARN("HMOTION: Param enable_magnet not found, set to true");
    if (!nh.param("enable_sonar", enable_sonar, true))
        ROS_WARN("HMOTION: Param enable_sonar not found, set to true");
    if (!nh.param("enable_gps", enable_gps, true))
        ROS_WARN("HMOTION: Param enable_gps not found, set to true");
    if (!nh.param("data_collection", data_collection, true))
        ROS_WARN("HMOTION: Param data_collection not found, set to true");

    // --------- Subscribers ----------
    ros::Subscriber sub_true = nh.subscribe<nav_msgs::Odometry>("ground_truth/state", 1, &cbTrue);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("raw_imu", 1, &cbImu);
    ros::Subscriber sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("fix", 1, &cbGps);
    if (!enable_gps)
        sub_gps.shutdown();
    ros::Subscriber sub_magnet = nh.subscribe<geometry_msgs::Vector3Stamped>("magnetic", 1, &cbMagnet);
    if (!enable_magnet)
        sub_magnet.shutdown();
    ros::Subscriber sub_baro = nh.subscribe<hector_uav_msgs::Altimeter>("altimeter", 1, &cbBaro);
    if (!enable_baro)
        sub_baro.shutdown();
    ros::Subscriber sub_sonar = nh.subscribe<sensor_msgs::Range>("sonar_height", 1, &cbSonar);
    if (!enable_sonar)
        sub_sonar.shutdown();

    // --------- Publishers ----------
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1, true);
    geometry_msgs::PoseWithCovarianceStamped msg_pose;
    msg_pose.header.frame_id = "world";   // for rviz
    msg_pose.pose.pose.orientation.x = 0; // no roll
    msg_pose.pose.pose.orientation.y = 0; // no pitch
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("velocity", 1, true); // publish velocity
    geometry_msgs::Twist msg_vel;

    // --------- Wait for Topics ----------
    ROS_INFO("HMOTION: Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ((std::isnan(ux) && msg_true.header.seq == 0))) // wait for imu and truth only
        ros::spinOnce(); // update subscribers

    if (!ros::ok())
    { // ROS shutdown
        ROS_INFO("HMOTION: ===== END =====");
        return 0;
    }

    // --------- Calibrate Gyro service ----------
    ROS_INFO("HMOTION: Calibrating Gyro...");
    ros::ServiceClient calibrate_gyro = nh.serviceClient<std_srvs::Empty>("raw_imu/calibrate");
    std_srvs::Empty calibrate_gyro_srv;
    if (calibrate_gyro.call(calibrate_gyro_srv))
        ROS_INFO("HMOTION: Calibrated Gyro");
    else
        ROS_WARN("HMOTION: Gyro cannot be calibrated!");

    // --------- Main loop ----------

    ros::Rate rate(motion_iter_rate);
    ROS_INFO("HMOTION: ===== BEGIN =====");
    ready = true;
    while (ros::ok() && nh.param("run", true))
    {
        ros::spinOnce(); // update topics

        // Verbose
        if (verbose)
        {
            auto & tp = msg_true.pose.pose.position;
            auto &q = msg_true.pose.pose.orientation;
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            ROS_INFO("[HM] ---------X-------Y-------Z-------A------");
            ROS_INFO("[HM]  TRUE(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", tp.x, tp.y, tp.z, atan2(siny_cosp, cosy_cosp));
            ROS_INFO("[HM] STATE(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", X(0), Y(0), Z(0), A(0));
            ROS_INFO("[HM]   GPS(%7.3lf,%7.3lf,%7.3lf, ---- )", GPS(0), GPS(1), GPS(2));
            ROS_INFO("[HM] MAGNT( ----- , ----- , ----- ,%6.3lf)", a_mgn);
            ROS_INFO("[HM]  BARO( ----- , ----- ,%7.3lf, ---- )", z_bar);
            ROS_INFO("[HM] BAROB( ----- , ----- ,%7.3lf, ---- )", Z(2));
            ROS_INFO("[HM] SONAR( ----- , ----- ,%7.3lf, ---- )", z_snr);

            if (data_collection) {
                data_file << "baro bias var: " << calculate_var_baro(baroCorrectedValues) << ", "
                          << "z_snr var: " << calculate_var(sonar) << ", "
                          << "a_mgn var: " << calculate_var(magnetic)
                          << std::endl;
            }
        }

        //  Publish pose and vel
        if (use_ground_truth)
        {
            msg_pose.header.stamp = ros::Time::now();
            msg_pose.pose.pose.position = msg_true.pose.pose.position;
            msg_pose.pose.pose.orientation = msg_true.pose.pose.orientation;
            msg_vel = msg_true.twist.twist;
        }
        else
        {
            msg_pose.header.stamp = ros::Time::now();
            msg_pose.pose.pose.position.x = X(0);
            msg_pose.pose.pose.position.y = Y(0);
            msg_pose.pose.pose.position.z = Z(0);
            msg_pose.pose.covariance[0] = P_x(0, 0);  // x cov
            msg_pose.pose.covariance[7] = P_y(0, 0);  // y cov
            msg_pose.pose.covariance[14] = P_z(0, 0); // z cov
            msg_pose.pose.covariance[35] = P_a(0, 0); // a cov
            msg_pose.pose.pose.orientation.w = cos(A(0) / 2);
            msg_pose.pose.pose.orientation.z = sin(A(0) / 2);
            msg_vel.linear.x = X(1);
            msg_vel.linear.y = Y(1);
            msg_vel.linear.z = Z(1);
            msg_vel.angular.z = A(1);
        }
        pub_pose.publish(msg_pose);
        pub_vel.publish(msg_vel);

        rate.sleep();
    }

    ROS_INFO("HMOTION: ===== END =====");
    return 0;
}