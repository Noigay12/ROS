#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <turtlesim/Spawn.h>
#include <cstdlib>
#include <ctime>
using namespace std;

turtlesim::Pose current_pose[50];

const float PI = 3.14159265;
const double pl = 0.01;

class Posecallback {
public:
    int turtle_idx;
    ros::Subscriber sub;
    void callback(const turtlesim::PoseConstPtr &pose)
    {
        current_pose[turtle_idx] = *pose;
    }
};

double x_y[50][2];
int ipt[50];

Posecallback sub[50];
ros::Publisher pub[50];
double distance(double x0, double y0, double goal_x, double goal_y)
{
    double dx = x0 - goal_x, dy = y0 - goal_y;
    double d = sqrt(pow(goal_x - x0, 2) + pow(goal_y - y0, 2));
    if(d < pl) d = 0.0 ;
    return d ;
}
double dalpha(double x0, double y0, double goal_x, double goal_y, double z0)
{
    double alfa;
    if(distance(x0, y0, goal_x, goal_y)< pl) alfa = 0.0;
    else
        alfa =  asin ((cos(z0)*(goal_y-y0)-sin(z0)*(goal_x-x0)) / distance(x0, y0, goal_x, goal_y));
    return alfa;
}
void filter_tp(double x0, double y0, int n, int j)
{
    double d[50], thaytam;
    for(int i = j;i <= n;i++)
        d[i] = sqrt(pow(x_y[i][0] - x0, 2) + pow(x_y[i][1] - y0, 2));
    for(int i = j;i <= n;i++)
        for (int k = j+1; k <= n; k++)
            if (d[i] > d[k])
            {
                thaytam = d[i];
                d[i] = d[k];
                d[k] = thaytam;
                thaytam = x_y[i][0];
                x_y[i][0] = x_y[k][0];
                x_y[k][0] = thaytam;
                thaytam = x_y[i][1];
                x_y[i][1] = x_y[k][1];
                x_y[k][1] = thaytam;
            }
}
// void filter_pt(int n, int arg)
// {
//     double d[50][50];
//     for(int j = 1+n; j <= (arg - 1) / 2; j++)
//         for(int i = 1; i <= n; i++)
//             d[j][i] = sqrt(pow(x_y[j][0] - x_y[i][0] , 2) + pow(x_y[j][1] - x_y[i][0] , 2));
//     double min = d[1+n][1];
//     for (int j = 1+n; j <= (arg - 1) / 2; j++)
//         for(int i = 1; i <= n; i++)
//         {
//             if (min > d[j][i])
//             {
//                 min = d[j][i];
//                 ipt[j] = i;
//             }
//             cout << min << " " << ipt[j] << endl;
//         }
// }
geometry_msgs::Twist getMessage(double x0, double y0, double goal_x, double goal_y, double z0)
{
    geometry_msgs::Twist msg;
    msg.linear.x = min(4*distance(x0, y0, goal_x, goal_y), 6.5);
    msg.angular.z = 10*dalpha(x0, y0, goal_x, goal_y, z0);
    return msg;
}

int main(int argc, char **argv)
{
    srand(time(NULL));
    ros::init(argc, argv, "pl_socute");
    ros::NodeHandle node;
    int n = atof(argv[1]) ;
    for(int i = 1;i < n;i++)
    {
        ros::service::waitForService("spawn");
        ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn");
        turtlesim::Spawn turtle;
        turtle.request.x = (rand() % 20)/2;
        turtle.request.y = (rand() % 20)/2;
        turtle.request.theta = 1.04;
        spawner.call(turtle);
    }
    for(int i = 1;i <= n;i++)
    {
        stringstream s;
        s << "turtle" << i;
        string name = s.str();
        sub[i].turtle_idx = i;
        sub[i].sub = node.subscribe(name+"/pose", 100, &Posecallback::callback, &sub[i]);
        pub[i] = node.advertise<geometry_msgs::Twist>(name+"/cmd_vel", 100);
    }
    const double tolerance = 1e-2;
    int j = 2;
    ros::Rate rate(100);
    for(int i = 1;i <= (argc - 1) / 2;i++)
        if(j < argc - 1)
        {
            x_y[i][0] = atof(argv[j++]);
            x_y[i][1] = atof(argv[j++]);
        }
    static bool test_idx = false;
    int s = 0;
    while(ros::ok())
    {
        for(int i = 1;i <= n;i++)
        {
            if (test_idx == false)
            {
                filter_tp(current_pose[i].x, current_pose[i].y, (argc - 1) / 2, i);
                pub[i].publish(getMessage(current_pose[i].x, current_pose[i].y, x_y[i][0], x_y[i][1], current_pose[i].theta));
                if ((distance(current_pose[i].x, current_pose[i].y, x_y[i][0], x_y[i][1]) == 0) && (test_idx == false))
                    s++; 
                if (s == n)
                    test_idx = true;
            }
            if (test_idx == true)
            {
                if (i + n <= (argc - 1) / 2)
                {
                    filter_tp(current_pose[i].x, current_pose[i].y, (argc - 1) / 2, i+n);
                    pub[i].publish(getMessage(current_pose[i].x, current_pose[i].y, x_y[i+n][0], x_y[i+n][1], current_pose[i].theta));
                }
            }
            // if (test_idx == true)
            // {
            //     if (i + n <= (argc - 1) / 2)
            //     {    
            //         filter_pt(n, argc);
            //         pub[ipt[i+n]].publish(getMessage(current_pose[ipt[i+n]].x, current_pose[ipt[i+n]].y, x_y[i+n][0], x_y[i+n][1], current_pose[ipt[i+n]].theta));
            //     }
            // }    
        }
        rate.sleep();
        ros::spinOnce();
    }
    for(int i = 1;i <= n;i++)
        cout << "turtle " << i << " go to: " << x_y[i][0] << "," << x_y[i][1] << endl;
    for(int i = 1;i <= n;i++)
        cout << "turtle " << i << " go to: " << x_y[i+n][0] << "," << x_y[i+n][1] << endl;
    return 0;
}