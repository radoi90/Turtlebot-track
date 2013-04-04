#include "Movement.h"

//constructor, initialise the speeds
BotMover::BotMover(ros::NodeHandle nh, double turn_speed, double fast_speed, double init_speed) {
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
	cruiseSpeed = turn_speed;
	boostSpeed = fast_speed;
	last_speed = init_speed;
}

//send the movement command to the Turtlebot based on road curvature and collission distance
void BotMover::move(double alpha, int distFromObject) {
	geometry_msgs::Twist vel; //the movement message
	double topSpeed;	//the maximum speed based on the situation

	//if collission iminent STOP!
	if (distFromObject < 28) {
		vel_pub.publish(vel);
		return;
	}
	
	//if the line is valid see which way to go
	if(alpha != -1000 && alpha != -1) {
		//+/-alpha = right/left
		//+/-z	   = left/right
		//</>img   = left/right
	
		//if we're on a straight and empty strech
		if (std::abs(alpha) < 12 && distFromObject > 105) {
			topSpeed = boostSpeed;	//go faster than normal
			
			//accelarate faster at the beginning
			if (last_speed < 0.15) last_speed = std::min(last_speed * 1.9, topSpeed);
			//get to top speed
			else last_speed = last_speed + (topSpeed - last_speed)/3;
		}
		//if we are turning or going towards an object
		else {
			topSpeed = cruiseSpeed; //use a safe speed limit
			if (distFromObject < 45) topSpeed /= 3; //if the object is really close break hard
			else if (distFromObject < 65) topSpeed /= 2.1;	//if the object gets closer slow down more
			else if (distFromObject < 85) topSpeed /= 1.5;	//if there is an object in the distance slow down a bit
			//deccelerate to the appropriate speed
			if (last_speed > topSpeed) last_speed = last_speed - ((last_speed - topSpeed) / 2.1) - 0.01;
			//maintain cruise speed in turn
			else last_speed = std::min(last_speed * 1.5, topSpeed);
		}
	}
	//in case the line is lost decelerate
	else last_speed = std::max(last_speed * 0.65, 0.01);

	vel.linear.x = last_speed; //set the linear speed
	//adapt the turning speed to the linear speed
	vel.angular.z = (-0.02 * alpha * (last_speed/cruiseSpeed) * (last_speed/cruiseSpeed));
	//debug
	std::cout << alpha << " " << vel.angular.z << " " << vel.linear.x << std::endl;	
	vel_pub.publish(vel); //move the robot
}
