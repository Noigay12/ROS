#include <boost/bind.hpp>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

turtlesim::PoseConstPtr g_pose;
turtlesim::Pose g_goal;
turtlesim::Pose goal;
turtlesim::Pose Test;

enum State
{
  FORWARD,
  TURN,
  STOP_TURN,
  END,
};

State g_state = TURN;
State g_last_state = END;
bool g_first_goal_set = false;

#define PI 3.141592

void poseCallback1(const turtlesim::PoseConstPtr& pose)
{
  g_pose = pose;
}

void poseCallback2(const turtlesim::PoseConstPtr& pose)
{
  goal.x = pose->x;
  goal.y = pose->y;
  goal.theta = atan((goal.y - g_pose->y) / (goal.x - g_pose->x));
  if ((goal.x < g_pose->x) && (goal.y > g_pose->y))
    goal.theta = PI - atan((goal.y - g_pose->y) / fabsf(goal.x - g_pose->x));
  if ((goal.x < g_pose->x) && (goal.y < g_pose->y))
    goal.theta = -(PI / 2 + atan((goal.x - g_pose->x) / (goal.y - g_pose->y)));
  if ((goal.x > g_pose->x) && (goal.y < g_pose->y))
    goal.theta = -(atan(fabsf(goal.y - g_pose->y) / (goal.x - g_pose->x)));
}

bool hasReachedGoal()
{
  return fabsf(g_pose->x - g_goal.x) < 0.01 && fabsf(g_pose->y - g_goal.y) < 0.01 && fabsf(g_pose->theta - goal.theta) < 0.01;
}

bool hasStopped()
{
  return g_pose->angular_velocity < 0.0001 && g_pose->linear_velocity < 0.0001;
}

bool hasReachedEnd()
{
  return fabsf(g_pose->x - goal.x) > 0.01 && fabsf(g_pose->y - goal.y) > 0.01;
}

void printGoal()
{
  ROS_INFO("New Goal [%f %f, %f]", g_goal.x, g_goal.y, g_goal.theta);
}

void commandTurtle(ros::Publisher twist_pub, float linear, float angular)
{
  geometry_msgs::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  twist_pub.publish(twist);
}

void stopTurn(ros::Publisher twist_pub)
{
  if (hasStopped())
  {
    ROS_INFO("Reached Goal");
    g_state = FORWARD;
    g_goal.x = goal.x;
    g_goal.y = goal.y;
    printGoal();
  }
  else
  {
    commandTurtle(twist_pub, 0, 0);
  }
}


void forward(ros::Publisher twist_pub)
{
  if (hasReachedEnd())
  {
    commandTurtle(twist_pub, 1.0, 0.0);
  }
  else
  {
    commandTurtle(twist_pub, 1.0, 0.0);
    ROS_INFO("Reached End!");
    g_state = END;
    commandTurtle(twist_pub, 0, 0);
  }
}

void turn(ros::Publisher twist_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_TURN;
    commandTurtle(twist_pub, 0, 0);
  }
  else
  {
    commandTurtle(twist_pub, 0.0, 0.4);
  }
}

void end(ros::Publisher twist_pub)
{
  commandTurtle(twist_pub, 0, 0);
}
void timerCallback(const ros::TimerEvent&, ros::Publisher twist_pub)
{
  if (!g_pose)
  {
    return;
  }
  if (!g_first_goal_set)
  {
    g_first_goal_set = true;
    g_state = TURN;
    g_goal.x = g_pose->x;
    g_goal.y = g_pose->y;
    g_goal.theta = goal.theta;
    printGoal();
  }
  if (g_state == TURN)
  {
    turn(twist_pub);
  }
  else if (g_state == STOP_TURN)
  {
    stopTurn(twist_pub);
  }
  else if (g_state == FORWARD)
  {
    forward(twist_pub);
  }
  else if (g_state == END)
  {
    end(twist_pub);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "myturtle_control");
  ros::NodeHandle nh;
  ros::Subscriber pose_sub1 = nh.subscribe("turtle1/pose", 1, poseCallback1);
  ros::Subscriber pose_sub2 = nh.subscribe("turtle2/pose", 1, poseCallback2);
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
  ros::Timer timer = nh.createTimer(ros::Duration(0.016), boost::bind(timerCallback, _1, twist_pub));

  ros::spin();
}