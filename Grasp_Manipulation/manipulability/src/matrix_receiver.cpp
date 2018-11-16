#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

void matrixcb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	float dstride0 = msg->layout.dim[0].stride;
	float dstride1 = msg->layout.dim[1].stride;
	float h = msg->layout.dim[0].size;
	float w = msg->layout.dim[1].size;
	ROS_INFO("mat(0,0) = %f",msg->data[0 + dstride1*0]);
	ROS_INFO("mat(0,1) = %f",msg->data[0 + dstride1*1]);
	ROS_INFO("mat(1,1) = %f\r\n",msg->data[1 + dstride1*1]);

	// Below are a few basic Eigen demos:
	std::vector<float> data = msg->data;
	Eigen::Map<Eigen::MatrixXf> mat(data.data(), h, w);
	std::cout << "I received = " << std::endl << mat << std::endl;
	std::cout << "Its inverse is = " << std::endl << mat.inverse() << std::endl;
	std::cout << "Multiplying by itself  = " << std::endl << mat*mat << std::endl;
	Eigen::EigenSolver<Eigen::MatrixXf> es(mat);
	std::cout << "Its eigenvalues are  = " << std::endl << es.eigenvalues() << std::endl;

	Eigen::MatrixXf newmat = mat;
	// Now let's do the nested for loop computation:
	for (int i=0; i<h-1; i++) 
		for (int j=0; j<w; j++) 
			newmat(i,j) = mat(i+1,j)+2;
	std::cout << "newmat = " << std::endl << newmat << std::endl;
	
	return;
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "subscriber");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("sent_matrix", 1, matrixcb);
	ros::spin();
    return 0;
}


//  source :  https://gist.github.com/jarvisschultz/7a886ed2714fac9f5226