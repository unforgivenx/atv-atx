#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>

double x;
double y;
double th;
double beta;
double delta_x;
double delta_y;
double delta_th;

double lr = 0.55; // distance of rear axle to c.g.
double lf = 0.65; // distance of front axle to c.g.

double v; // m/s
double vx;
double vy;
double vehicle_x = 0;
double vth;
double deltaLeft;
double deltaRight;

double vl, vr; // m/s velocity of right and left hubs

double steering_angle, st_pot;
int thrust_dir=0;
double wheel_dia = 0.5; //meters
double DistancePerCount = (3.14159265 * 0.50) / 128.0;
float rad_cur;
ros::Time last_time_r, last_time_l;

void rspcallback(const std_msgs::UInt16::ConstPtr& msg)
{
  last_time_r = ros::Time::now();
  vr=(msg->data)*wheel_dia*M_PI/(128.0);
  vehicle_x = DistancePerCount;
}

void lspcallback(const std_msgs::UInt16::ConstPtr& msg)
{
  last_time_l = ros::Time::now();
  vl=(msg->data)*wheel_dia*M_PI/(128.0);
  vehicle_x = DistancePerCount;
}

void stcallback(const std_msgs::Int16::ConstPtr& msg)
{
  steering_angle=-1*((msg->data)-100)*60/100.0;
  beta = atan(lr*tan(steering_angle * M_PI / 180.0)/(lf+lr));
  rad_cur = (lf+lr)/(steering_angle*M_PI/180.0);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber vr_sub = n.subscribe("r_speed", 1, rspcallback);
  ros::Subscriber vl_sub = n.subscribe("l_speed", 1, lspcallback);
  ros::Subscriber st_sub = n.subscribe("steering_perc", 1, stcallback);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time, current_time_r, current_time_l;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    current_time_r = ros::Time::now();
    current_time_l = ros::Time::now();
    //compute odometry in a typical way given the velocities of the robot
    if ((current_time_r-last_time_r).toSec() > 0.5)
	vr=0;

    if ((current_time_l-last_time_l).toSec() > 0.5)
        vl=0;

    v = (vl + vr)/2.0;
    //v = vr;
    if (thrust_dir == -1)
        v = -1 * v;

    vx = v*cos(th+beta);
    vy = v*sin(th+beta);
    vth = v*cos(beta)*tan(steering_angle * M_PI / 180.0)/(lf+lr);

    double dt = (current_time - last_time).toSec();
    double delta_x = vx * dt;
    double delta_y = vy * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;
    ROS_INFO("st_ang: %lf, beta: %lf, th: %lf", steering_angle, beta, th);
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
