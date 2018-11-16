/*
sample_publisher is an example that publishes a vector on a topic where sample_listener.cpp listens to the message
*/
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

float Arr[90];
void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "arraySubscriber");

	ros::NodeHandle n;	

	ros::Subscriber sub3 = n.subscribe("Topic_Array", 100, arrayCallback);
	while (ros::ok())
	{
		for(int j = 0; j < 90; j++)
		{
			printf("%g, ", Arr[j]);
		}

		printf("\n");

		ros::spinOnce();
		//Added a delay so not to spam
		sleep(2);
	}

	return 0;
}

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

	int i = 0;
	// print all the remaining numbers
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}
	return;
}