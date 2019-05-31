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
double x[50], y[50];

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

void filter(double a, double b, int n, int s)
{
    double d[50], thaytam;
    for(int i = s;i <= n;i++)
        d[i] = sqrt(pow(x[i] - a, 2) + pow(y[i] - b, 2));
    for(int i = s;i <= n;i++)
        for (int k = s+1; k <= n; k++)
            if (d[i] > d[k])
            {
                thaytam = d[i];
                d[i] = d[k];
                d[k] = thaytam;
                thaytam = x[i];
                x[i] = x[k];
                x[k] = thaytam;
                thaytam = y[i];
                y[i] = y[k];
                y[k] = thaytam;
            }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "myturtle_control");
    cout << "argc =" << argc << endl;
    for (int i=1; i<argc; ++i)
    cout << "argv[" << i << "]= " << argv[i] << endl;
    int n = (argc - 1) / 2;
    ros::NodeHandle h;
    ros::Publisher pub = h.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::Subscriber sub = h.subscribe("/turtle1/pose", 1000, poseCallback);
    ros::Rate loopRate(rate);
    double distance;
    double x0, y0, x1 = 5.544445, y1 = 5.544445, alpha;
    const double tolerance = 1e-1;
    int j = 1, s = 1;
    x[0] = x1;
    y[0] = y1;
    for (int i = 1;i <= (argc - 1) / 2;i++)
        if(j < argc - 1)
        {
            x[i] = atof(argv[j++]);
            y[i] = atof(argv[j++]);
        }
    while (ros::ok())
    {
        if (s <= (argc - 1) / 2)
        {
            x0 = x[s];
            y0 = y[s];
            s++;
        }
        else
            break;
        for (int i = 1;i <= (argc - 1) / 2;i++)
            filter(x0, y0, (argc - 1) / 2, s);
        while (ros::ok()) {
            loopRate.sleep();
            ros::spinOnce();

            // tính khoảng cách đến đích
            distance = sqrt(pow(x0-current_pose.x, 2) + pow(y0-current_pose.y, 2) );
            // double alpha = atan2( y0-current_pose.y, x0-current_pose.x ),a_z;
            // tính các vector đến đích và hướng hiện thời
            double dx = x0 - current_pose.x, dy = y0 - current_pose.y, theta = current_pose.theta;
            // sử dụng tích có hướng của vector để tính sin của góc lệch (sau đó dùng arcsin)
            double dalpha = asin ((cos(theta)*dy-sin(theta)*dx) / distance);

            //int state = 0;
            //break;
            // tạo message bằng phương pháp điều khiển PID (có giới hạn vận tốc)
            // Ở đây dùng hệ số K_p = 1.0, K_i = K_d = 0.0
            if ((dx > 0) && (dy > 0) && (x0 != x1))
            {
                geometry_msgs::Twist msg = getMessage(
                    min(5*distance, 9.0),
                    10*dalpha
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
                    min(4*distance, 9.0),
                    10*dalpha
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
                    -min(4*distance, 9.0),
                    10*dalpha
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
                    -min(4*distance, 9.0),
                    10*dalpha
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
                    -min(4*distance, 9.0),
                    -10*dalpha
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
                    min(4*distance, 9.0),
                    10*dalpha
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
                    min(4*distance, 9.0),
                    10*dalpha
                );
                pub.publish(msg);
                if ((distance < tolerance) && (current_pose.linear_velocity < tolerance))
                {
                    pub.publish(getMessage(0,0));
                    break;
                }
            }
        }
        x1 = x0; y1 = y0;
        alpha = atan2(x0, y0);
        cout << "turtle go to: " << x0 << "; " << y0 << endl;
    }
    return 0;
}