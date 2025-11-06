# include <iostream>
# include <string>
# include <fstream>
# include <iomanip>
# include "find_path/function.hpp"
# include <ros/ros.h>
# include <morai_msgs/GPSMessage.h>
# include <morai_msgs/CtrlCmd.h>
# include <sensor_msgs/Imu.h> //imu 데이터값 들어있는 메세지 헤더파일 include
# include <std_msgs/Float64.h>
# include <cmath>
# include <ros/package.h>
# define _USE_MATH_DEFINES

using namespace std;

ros::Publisher pub;


void imuCallback (const sensor_msgs::Imu::ConstPtr& msg) {
    getYaw (msg, current_yaw);
}

void enuCallback (const morai_msgs::GPSMessage::ConstPtr& msg) {
    gpsTf (msg, current_e, current_n);

    int closest_idx = findClosestPoint(path);
    int coner_idx = findConerIdx(path);
    egoPath bestPoint = detectFront(path);
    double d_err = getSignedDistanceErr(path, closest_idx);
    double y_err = getYawErr(path);

    double steer_angle;
    double v_mps;

    if (closest_idx > coner_idx-50 && closest_idx < coner_idx) {
        double k = 5.0;
        double v_low = 10.0;


        steer_angle = y_err + atan2(k * d_err, v_low);
        v_mps = v_low;
    }
    else {
        double k = 0.4;
        double v_high = 30.0;

        steer_angle = y_err + atan2(k * d_err, v_high);
        v_mps = v_high;
    }

    // double steer_angle_min = -0.52;
    // double steer_angle_max = 0.52;
    // double steer_angle = y_err + atan2(k * d_err, v_mps);
    // double steer_angle = max(steer_angle_min, min(y_err + atan2(k * d_err, v_mps), steer_angle_max));
    cout << coner_idx << endl;
    cout << closest_idx << endl;


    morai_msgs::CtrlCmd cmd_msg;
    cmd_msg.longlCmdType = 2;
    cmd_msg.velocity = v_mps;
    cmd_msg.steering = steer_angle;

    pub.publish(cmd_msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "auto_steer");
    ros::NodeHandle nh;

    if (!loadPath()) {
        ROS_ERROR("Failed to load path file. Shutting down.");
        return 1; // 오류 코드로 종료
    }
    cout << "Path loaded successfully with %lu points.", path.size();

    pub = nh.advertise <morai_msgs::CtrlCmd>("/ctrl_cmd", 10);

    ros::Subscriber sub_imu = nh.subscribe("/imu", 10, imuCallback);
    ros::Subscriber sub_gps = nh.subscribe("/gps", 10, enuCallback);
    

        ros::spin();

    return 0;
}