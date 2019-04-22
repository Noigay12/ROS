#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <cmath>
#include <iostream>
#include <algorithm>
using namespace std;

ros::Publisher pub;
const float PI = 3.14159265;
float rate = 100;
turtlesim::Pose current_pose;

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "myturtle_control");
    cout << "argc =" << argc << endl;
    for (int i=0; i<argc; ++i)
    cout << "argv[" << i << "]= " << argv[i] << endl;
    ros::NodeHandle h;
    ros::Publisher pub = h.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::Subscriber sub = h.subscribe("/turtle1/pose", 1000, poseCallback);
    ros::Rate loopRate(rate);

    double x0, y0, x1 = 5.544445, y1 = 5.544445, alpha;
    const double tolerance = 1e-2;

    while (ros::ok())
    {
        x0 = (rand()%20)/2;
        y0 = (rand()%20)/2;
        while (ros::ok()) {
            loopRate.sleep();
            ros::spinOnce();

            // tính khoảng cách đến đích
            double distance = sqrt( pow(x0-current_pose.x, 2) + pow(y0-current_pose.y, 2) );
            // double alpha = atan2( y0-current_pose.y, x0-current_pose.x ),a_z;
            // tính các vector đến đích và hướng hiện thời
            double dx = x0 - current_pose.x, dy = y0 - current_pose.y, theta = current_pose.theta;
            // sử dụng tích có hướng của vector để tính sin của góc lệch (sau đó dùng arcsin)
            double dalpha = asin ((cos(theta)*dy-sin(theta)*dx) / distance);
            cout << "distance= " << distance << "dalpha= " << dalpha << endl;

            //int state = 0;
            //break;
            // tạo message bằng phương pháp điều khiển PID (có giới hạn vận tốc)
            // Ở đây dùng hệ số K_p = 1.0, K_i = K_d = 0.0
            if ((dx > 0) && (dy > 0) && (x0 != x1))
            {
                geometry_msgs::Twist msg = getMessage(
                    min(4*distance, 6.5),
                    20*dalpha
                );
                pub.publish(msg);
                if ((distance < tolerance) && (current_pose.linear_velocity < tolerance))
                {
                    pub.publish(getMessage(0,0));
                    break;
                }
            }
            else if ((dx > 0) && (dy < 0) && (x0 != x1))
            {
                geometry_msgs::Twist msg = getMessage(
                    min(4*distance, 6.5),
                    20*dalpha
                );
                pub.publish(msg);
                if ((distance < tolerance) && (current_pose.linear_velocity < tolerance))
                {
                    pub.publish(getMessage(0,0));
                        break;
                }
            }
            else if ((dx < 0) && (dy < 0) && (x0 != x1))
            {
                dalpha = asin ((cos(theta + PI)*dy-sin(theta + PI)*dx) / distance);
                geometry_msgs::Twist msg = getMessage(
                    -min(4*distance, 6.5),
                    20*dalpha
                );
                pub.publish(msg);
                if ((distance < tolerance) && (current_pose.linear_velocity < tolerance))
                {
                    pub.publish(getMessage(0,0));
                    break;
                }
            }
            else if ((dx < 0) && (dy > 0) && (x0 != x1))
            {
                dalpha = asin ((cos(theta + PI)*dy-sin(theta + PI)*dx) / distance);
                geometry_msgs::Twist msg = getMessage(
                    -min(4*distance, 6.5),
                    20*dalpha
                );
                pub.publish(msg);
                if ((distance < tolerance) && (current_pose.linear_velocity < tolerance))
                {
                    pub.publish(getMessage(0,0));
                    break;
                }
            }
            else if ((x0 == x1) && (y0 < y1))
            {
                geometry_msgs::Twist msg = getMessage(
                    -min(4*distance, 6.5),
                    -20*dalpha
                );
                pub.publish(msg);
                if ((distance < tolerance) && (current_pose.linear_velocity < tolerance))
                {
                    pub.publish(getMessage(0,0));
                    break;
                }
            }
            else if ((x0 == x1) && (y0 > y1))
            {
                geometry_msgs::Twist msg = getMessage(
                    min(4*distance, 6.5),
                    20*dalpha
                );
                pub.publish(msg);
                if ((distance < tolerance) && (current_pose.linear_velocity < tolerance))
                {
                    pub.publish(getMessage(0,0));
                    break;
                }
            }
            else
            {
                dalpha = asin ((cos(theta + PI)*dy-sin(theta + PI)*dx) / distance);
                geometry_msgs::Twist msg = getMessage(
                    min(4*distance, 6.5),
                    20*dalpha
                );
                pub.publish(msg);
                if ((distance < tolerance) && (current_pose.linear_velocity < tolerance))
                {
                    pub.publish(getMessage(0,0));
                    break;
                }
            }
            cout << dx << " " << dy << endl;
        }
        x1 = x0; y1 = y0;
        alpha = atan2(x0, y0);
        cout << x0 << " " << y0 << " " << alpha << endl;
        cout << current_pose.x << " " << current_pose.y << " " << current_pose.theta << endl;
    }
    return 0;
}