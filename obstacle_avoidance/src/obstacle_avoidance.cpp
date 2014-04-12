/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#define PI 3.14159265358979323846

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"

int nRaysLaser = 0;
double thresholdLow = 10.0;	//Lowest Safety radius for the laser in [m]
double thresholdHigh = 10.0;	//Highest Safety radius for the laser in [m]
double thresholdEmergency = 100;	//Emergency range, the vehicle stops
double rangePotFieldLow = 50.0;		//The range of the potential field magnitude between [-rangePotField, rangePotField]
double rangePotFieldHigh = 5.0;         //The range of the potential field magn
double slopeRepulsive = 0;			//Repulsive force slope
double slopeAttractive = 0; 		//Attractive force slope
bool isFirstScan = true;		//If it is the first scan or not
//If the resolution of SICK LMS is set to 1 deg, then whole data is sent at one jerk. If the resolution is set to 0.5 deg, then the whole data is sent at two. And if the resolution is set to 0.25 deg, then the whole data is sent at four jerks.
int countScan = 0;			
int countScanMax = 2; 
double sumMagX = 0.0;
double sumMagY = 0.0;
double sumWeightedTheta = 0.0;

ros::Publisher pub_steering_angle;
ros::Publisher pub_raw_angle;
ros::Publisher pub_throttle;

std_msgs::Float32 msg_steering_angle;
std_msgs::Float32 msg_raw_angle;
std_msgs::Float32 msg_throttle;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%

//Calculates the slopes of the pot field function (both slopes are positive)
void InitiateSettings(float rangeMin, float rangeMax, float angleMin, float angleMax, float angleInc)
{
	slopeRepulsive = rangePotFieldLow/(thresholdLow - rangeMin);	
  	slopeAttractive = rangePotFieldHigh/(rangeMax - thresholdHigh);
	nRaysLaser = (angleMax - angleMin) / (angleInc) + 1;
}

// Calculates the potential field function value for a particular range scan reading (can be positive or negative)
double CalculatePotFieldWeight(float reading)
{
	double value = 0;
   	if (reading < thresholdLow)
		value = -rangePotFieldLow + reading * slopeRepulsive;
	else if (reading > thresholdHigh)
		value = reading * slopeAttractive;
   	return value;	
}

// Sick LMS Callback Function
void sickLMSCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// If this is your first scan, you have to fight!
	if (isFirstScan)
    	{
		InitiateSettings(msg->range_min, msg->range_max,  msg->angle_min, msg->angle_max, msg->angle_increment);
		isFirstScan = !isFirstScan;
    	}	
	if (countScan >= countScanMax)
	{
		countScan = 0;
		sumMagX = 0;
		sumMagY = 0;
		sumWeightedTheta = 0;
	}
  	/*
	for (int i = 0; i < nRaysLaser; i++)
	{
		double tempRange = msg->ranges[i];
		if (!isnan(tempRange))
		{
			//if (tempRange <= thresholdEmergency)
			//	ROS_INFO("Stop!!!!!");  
			sumWeights += fabs(CalculatePotFieldWeight(tempRange));
		}
		//ROS_INFO("Range [%d]: [%f]",i , msg->ranges[i]);
	}
	*/
	for (int i = 0; i < nRaysLaser; i++)
	{
		double tempRange = msg->ranges[i];
		if (!isnan(tempRange))
		{
			double angle = msg->angle_min + i * msg->angle_increment;
			double magnitude = CalculatePotFieldWeight(tempRange);
			angle = (magnitude < 0) ? angle + 180 : angle;
			sumMagX += fabs(magnitude) * cos(angle * PI / 180.0);
			sumMagY += fabs(magnitude) * sin(angle * PI / 180.0);
			//ROS_INFO("Angle: [%f]", angle*180.0/PI);
			//ROS_INFO("Weight: [%f]", weight);
			//sumWeightedTheta += fabs(weight)*((1-(weight > 0))*PI + angle);
			//ROS_INFO("sumWeightedTheta: [%f]", sumWeightedTheta);
		}
	}
  
	countScan++;
	if (countScan == countScanMax)
	{  	
		//if (sumWeights == 0)
		//{
		//	msg_steering_angle.data = 0.0f;
		//	msg_throttle.data = 0.0f;
		//	pub_steering_angle.publish(msg_steering_angle);
			//pub_throttle.publish(msg_throttle);
		//	ROS_INFO("Steering Angle: [%f]", 0.0);
		//	ROS_INFO("Throttle: [%f]", 0.0);			
		//}
		//else
		//{
			msg_steering_angle.data = atan2(sumMagY, sumMagX) * 180.0 / PI;
 			ROS_INFO("Raw Angle: [%f]", msg_steering_angle.data);
			pub_raw_angle.publish(msg_steering_angle);
			if (msg_steering_angle.data <= -90.0)
				msg_steering_angle.data = 180.0;
			if (msg_steering_angle.data <= 0.0 && msg_steering_angle.data > -90.0)
				msg_steering_angle.data = 0.0;			
//msg_steering_angle.data = (msg_steering_angle.data >= 180) ? 180.0 : msg_steering_angle.data;
			msg_throttle.data = slopeRepulsive;
			pub_throttle.publish(msg_throttle);
			pub_steering_angle.publish(msg_steering_angle);
			msg_throttle.data = slopeAttractive;
			pub_throttle.publish(msg_throttle);
			ROS_INFO("Steering Angle: [%f]", msg_steering_angle.data);
			ROS_INFO("Throttle: [%f]", msg_throttle.data);
		//}
		//ROS_INFO("CS: [%d]", countScan);
  		//ROS_INFO("Ranges 0: [%f]", msg->angle_min);
	}
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "obstacle_avoidance");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("scan", 1000, sickLMSCallback);
  pub_steering_angle = n.advertise<std_msgs::Float32>("steering_angle", 1000);
  pub_raw_angle = n.advertise<std_msgs::Float32>("raw_angle", 1000);
  pub_throttle = n.advertise<std_msgs::Float32>("throttle", 1000);
// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
