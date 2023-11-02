#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
using namespace std;

// 双移线构造的参数
const float shape = 2.4;
const float dx1 = 25.0, dx2 = 21.95;
const float dy1 = 4.05, dy2 = 5.7;
const float Xs1 = 27.19, Xs2 = 56.46;
// 参考路径在 X 方向长度以及参考点的步长
const float length = 160.0;
const float step = 0.1;

inline float calculate_reference_y(const float ref_x)
{
    float z1 = shape / dx1 * (ref_x - Xs1) - shape / 2.0;
    float z2 = shape / dx2 * (ref_x - Xs2) - shape / 2.0;
    return dy1 / 2.0 * (1 + tanh(z1)) - dy2 / 2.0 * (1 + tanh(z2));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "DoubleLane");

    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("double_lane", 100, true);

    nav_msgs::Path reference_path;
    reference_path.header.frame_id = "world";
    reference_path.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";

    int points_size = length / step;
    for (int i = 0; i < points_size; ++i)
    {
        float ref_x = i * step;
        float ref_y = calculate_reference_y(ref_x);
        cout << "ref_x: " << ref_x << "; ref_y: " << ref_y << endl;
        pose.pose.position.x = ref_x;
        pose.pose.position.y = ref_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 0.0;
        reference_path.poses.emplace_back(pose);
    }

    ros::Rate loop(10);
    while (ros::ok())
    {
        path_pub.publish(reference_path);
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}