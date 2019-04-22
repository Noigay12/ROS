#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <iostream>
#include <algorithm>
using namespace std;

const float PI = 3.14159265;
float rate = 50;
turtlesim::Pose current_pose;
turtlesim::Pose current_pose_2;

geometry_msgs::Twist getMessage(double linear_x, double angular_z)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    return msg;
}

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    current_pose = *msg;
}

void poseCallback2(const turtlesim::Pose::ConstPtr& msg)
{
    current_pose_2 = *msg;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "myturtle_control");
    ros::NodeHandle h;
    ros::Publisher pub = h.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1000);
    ros::Publisher pub1 = h.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::Subscriber sub =
        h.subscribe("/turtle1/pose", 1000, poseCallback2);
    ros::Subscriber sub1 = h.subscribe("/turtle2/pose", 1000, poseCallback);
    ros::Rate loopRate(rate);

    double x0, y0, x1, y1;
    const double tolerance = 1e-5;

    while (ros::ok())
    {
        x0 = rand() % 10;
        y0 = rand() % 10;
        while (ros::ok()) {
            loopRate.sleep();
            ros::spinOnce();
            cout << current_pose_2.x << " " << current_pose_2.y << " " << current_pose_2.theta << endl;

            // tính khoảng cách đến đích
            double distance = sqrt( pow(x0-current_pose_2.x, 2) + pow(y0-current_pose_2.y, 2) );
            if (distance < tolerance) { // dừng robot khi đến đủ gần
                pub1.publish(getMessage(0,0));
                break;
            }
            // double alpha = atan2( y0-current_pose.y, x0-current_pose.x ),a_z;
            // tính các vector đến đích và hướng hiện thời
            double dx = x0 - current_pose_2.x, dy = y0 - current_pose_2.y, theta = current_pose_2.theta;
            // sử dụng tích có hướng của vector để tính sin của góc lệch (sau đó dùng arcsin)
            double dalpha = asin ((cos(theta)*dy-sin(theta)*dx) / distance);
            
            // tạo message bằng phương pháp điều khiển PID (có giới hạn vận tốc)
            // Ở đây dùng hệ số K_p = 1.0, K_i = K_d = 0.0
            geometry_msgs::Twist msg = getMessage(
                min(1*distance, 4.0),
                4*dalpha
            );

            pub1.publish(msg);

            x1 = current_pose_2.x;
            y1 = current_pose_2.y;
            while (ros::ok()) {
                loopRate.sleep();
                ros::spinOnce();
                cout << current_pose.x << " " << current_pose.y << " " << current_pose.theta << endl;

                // tính khoảng cách đến đích
                double distance1 = sqrt( pow(x1-current_pose.x, 2) + pow(y1-current_pose.y, 2) );
                // double alpha = atan2( y0-current_pose.y, x0-current_pose.x ),a_z;
                // tính các vector đến đích và hướng hiện thời
                double dx1 = x1 - current_pose.x, dy1 = y1 - current_pose.y, theta1 = current_pose.theta;
                // sử dụng tích có hướng của vector để tính sin của góc lệch (sau đó dùng arcsin)
                double dalpha1 = asin ((cos(theta1)*dy-sin(theta1)*dx) / distance);

                // tạo message bằng phương pháp điều khiển PID (có giới hạn vận tốc)
                // Ở đây dùng hệ số K_p = 1.0, K_i = K_d = 0.0
                geometry_msgs::Twist msg = getMessage(
                    min(1*distance1, 4.0),
                    4*dalpha1
                );

                pub.publish(msg);
            }
        }
    }
    return 0;
}