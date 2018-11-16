#include "ros/ros.h"
#include "manipulability/Man_service.h"
#include <cstdlib>
#include <geometry_msgs/Pose.h>
#include <manipulability/AddTwoInt.h>
#include "manipulability/AddTwoInt.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  //ros::ServiceClient client = n.serviceClient<manipulability::AddTwoInt>("add_two_ints");
  ros::ServiceClient client = n.serviceClient<manipulability::Man_service>("add_two_ints");

  manipulability::AddTwoInt srv;
  std::cout<<"\n"<<std::endl;
  srv.request.a = 1.3;//atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  std::cout<<"A = ="<<srv.request.a<<"; \n B="<<srv.request.b<<"; \n This is the computed value: \n"<<srv.response.sum<<std::endl;
   
  //std::cout<<"\n Test \n"<<std::endl;
   
   printf ("Characters: %c %c \n", 'a', 65);
   printf ("Decimals: %d %ld\n", 1977, 650000);
   printf ("Preceding with blanks: %10d \n", 1977);
   printf ("Preceding with zeros: %010d \n", 1977);
   printf ("Some different radixes: %d %x %o %#x %#o \n", 100, 100, 100, 100, 100);
   printf ("floats: %4.2f %+.0e %E \n", 3.1416, 3.1416, 3.1416);
   printf ("Width trick: %*d \n", 5, 10);
   printf ("%s \n", "A string");
   


  //ros::NodeHandle n;
  manipulability::Man_service srv1;
  
  srv1.request.pose.position.x = 1;  
  srv1.request.pose.position.y = 1;  
  srv1.request.pose.position.z = 1; 
  srv1.request.pose.orientation.x = 0;  
  srv1.request.pose.orientation.y = 0;  
  srv1.request.pose.orientation.z = 0;   
  srv1.request.pose.orientation.w = 1; 


  if (client.call(srv1))
  {
    ROS_INFO("Sum: ", srv1.response.pose_back.position.x);
    std::cout<<"A="<<"This is the computed value:\n"<<srv1.response.pose_back<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service!!!");
    return 1;
  }

  return 0;
}



/*
float32 a
float32 b
---
float32 sum

*/



/*
#include "ros/ros.h"
#include "manipulability/AddTwoInt.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<manipulability::AddTwoInt>("add_two_ints");
  manipulability::AddTwoInt srv;
  srv.request.a = 1.2;//atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  std::cout<<"A="<<srv.request.a<<"; B="<<srv.request.b<<"; This is the computed value:"<<srv.response.sum<<std::endl;
   
   printf ("Characters: %c %c \n", 'a', 65);
   printf ("Decimals: %d %ld\n", 1977, 650000);
   printf ("Preceding with blanks: %10d \n", 1977);
   printf ("Preceding with zeros: %010d \n", 1977);
   printf ("Some different radixes: %d %x %o %#x %#o \n", 100, 100, 100, 100, 100);
   printf ("floats: %4.2f %+.0e %E \n", 3.1416, 3.1416, 3.1416);
   printf ("Width trick: %*d \n", 5, 10);
   printf ("%s \n", "A string");
   
  if (client.call(srv))
  {
    ROS_INFO("Sum: %4.2f", srv.response.sum);
    //std::cout<<"A="<<"This is the computed value:"<<srv.response.sum<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_int");
    return 1;
  }

  return 0;
}

*/