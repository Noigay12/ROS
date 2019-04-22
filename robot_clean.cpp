#include<ros/ros.h>
#include<turtlesim/Pose.h>
#include<geometry_msgs/Twist.h>
#include<cmath>
#include<iostream>
using namespace std;

ros::Publisher pub;
ros::Subscriber sub;
turtlesim::Pose turtle_pose;
geometry_msgs::Twist msg;

#define PI 3.141592

void callback(const turtlesim::Pose::ConstPtr& msg)
{
    turtle_pose.x= msg->x;
    turtle_pose.y= msg->y;
    turtle_pose.theta= msg->theta;
}
double dist(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x2-x1), 2)+pow((y2-y1), 2));
}
void gotogoal(turtlesim::Pose goal_pose,double error)
{
    
    ros::Rate looprate(10);
    do
    {
        msg.linear.x = 0.5*dist(turtle_pose.x, turtle_pose.y, goal_pose.x, goal_pose.y);
        double theta0 = atan((goal_pose.y - turtle_pose.y) / (goal_pose.x - turtle_pose.x));
        if ((goal_pose.x < turtle_pose.x) && (goal_pose.y > turtle_pose.y))
          theta0 = PI - atan((goal_pose.y - turtle_pose.y) / fabsf(goal_pose.x - turtle_pose.x));
        if ((goal_pose.x < turtle_pose.x) && (goal_pose.y < turtle_pose.y))
          theta0 = -(PI / 2 + atan((goal_pose.x - turtle_pose.x) / (goal_pose.y - turtle_pose.y)));
        if ((goal_pose.x > turtle_pose.x) && (goal_pose.y < turtle_pose.y))
          theta0 = -(atan(fabsf(goal_pose.y - turtle_pose.y) / (goal_pose.x - turtle_pose.x)));
        msg.angular.z = 4*(theta0-turtle_pose.theta);
        pub.publish(msg);
        ros::spinOnce();
        looprate.sleep();
        cout << turtle_pose.x << " " << turtle_pose.y << " " << theta0 << endl;
    } while (dist(turtle_pose.x, turtle_pose.y, goal_pose.x, goal_pose.y) > error);
    
    do{
      msg.linear.x = 0;
      msg.angular.z = 4;
      pub.publish(msg);
    } while(turtle_pose.theta > 0.1);

    msg.linear.x = 0;
    msg.angular.z = 0;
    pub.publish(msg);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "myturtle_control");
    ros::NodeHandle node;
    pub= node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
    sub= node.subscribe("/turtle1/pose", 10, callback);
    ros::Rate looprate(10);

    turtlesim::Pose goal_pose;
 
    do{
      cout<<"Nhap toa do x, y: ";
      cin>>goal_pose.x>>goal_pose.y;
      gotogoal(goal_pose, 0.01);
      looprate.sleep();
    } while(msg.linear.x==0 && msg.angular.z == 0);
    ros::spin();
    
    return 0;
}