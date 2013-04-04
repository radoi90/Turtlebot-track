#ifndef MOVEMENT_H
#define	MOVEMENT_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class BotMover {
	private:
		ros::Publisher vel_pub;
		double cruiseSpeed;
		double boostSpeed;
		double last_speed;
	public:
		BotMover(ros::NodeHandle nh, double turn_speed, double fast_speed, double init_speed);
		void move(double alpha, int distFromObject);
};
#endif
