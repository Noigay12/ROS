#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

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
    ros::NodeHandle h;
    pub = h.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::Subscriber sub =
        h.subscribe("/turtle1/pose", 1000, poseCallback);
    ros::Rate loopRate(rate);

while (ros::ok())
{
    double x0= (rand()%20)/2  , y0= (rand()%20)/2;
    const double tolerance = 1e-2;

 
        while (ros::ok()) {
           // loopRate.sleep();
            ros::spinOnce();
            cout << current_pose.x << " " << current_pose.y << " " << current_pose.theta << endl;

            // tính khoảng cách đến đích
            double distance = sqrt( pow(x0-current_pose.x, 2) + pow(y0-current_pose.y, 2) );
            if (distance < tolerance) { // dừng robot khi đến đủ gần
               //pub.publish(getMessage(0,0));
                break;
            }
            // double alpha = atan2( y0-current_pose.y, x0-current_pose.x ),a_z;
            // tính các vector đến đích và hướng hiện thời
            double dx = x0 - current_pose.x, dy = y0 - current_pose.y, theta = current_pose.theta;
            // sử dụng tích có hướng của vector để tính sin của góc lệch (sau đó dùng arcsin)
            double dalpha = asin ((cos(theta)*dy-sin(theta)*dx) / distance);
            
            // tạo message bằng phương pháp điều khiển PID (có giới hạn vận tốc)
            // Ở đây dùng hệ số K_p = 1.0, K_i = K_d = 0.0


if ((cos(theta)>0)&&(cos(theta)<1))
{
    if ((sin(theta)>0)&&(sin(theta)<1))
    {
        if ((x0 >= current_pose.x) && (y0 >= current_pose.y))
            {
                geometry_msgs::Twist msg = getMessage(min(2*distance+0.8,8.0),2*dalpha);
                pub.publish(msg);
            }
        else
            {
                geometry_msgs::Twist msg = getMessage(-1 * min(2*distance+0.8,8.0),-8*dalpha);
                pub.publish(msg);
            }
    }
    else
    {
        if ((x0 >= current_pose.x) && (y0 <= current_pose.y))
            {
                geometry_msgs::Twist msg = getMessage(min(2*distance+0.8,8.0),8*dalpha);
                pub.publish(msg);
            }
        else    
            {    
                geometry_msgs::Twist msg = getMessage(-1 * min(2*distance+0.8,8.0),-8 *dalpha);
                pub.publish(msg);
            }
    }
}
else
{
    if ((sin(theta)>0)&&(sin(theta)<1))
    {
        if ((x0 <= current_pose.x) && (y0 >= current_pose.y))
            {
                geometry_msgs::Twist msg = getMessage(min(2*distance+0.8,8.0),8*dalpha);
                pub.publish(msg);
            }
        else
            {
                geometry_msgs::Twist msg = getMessage(-1 * min(2*distance+0.8,8.0),-8 *dalpha);
                pub.publish(msg);
            }
    }
    else
    {
        if ((x0 <= current_pose.x) && (y0 <= current_pose.y))
            {
                geometry_msgs::Twist msg = getMessage(min(2*distance+0.8,8.0),8*dalpha);
                pub.publish(msg);
            }
        else    
            {
                geometry_msgs::Twist msg = getMessage(-1 * min(2*distance+0.8,8.0),-8 *dalpha);
                pub.publish(msg);
            }
    }
}

}

}
    return 0;
}