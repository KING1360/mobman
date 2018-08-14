#include <ros/ros.h>
#include "std_msgs/String.h"

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <Eigen/LU>
#include <Eigen/Dense>

#include <sstream>

#include <iostream>
using namespace std;
using namespace KDL;
using namespace Eigen;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "manip");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher joint_msg_pub = n.advertise<sensor_msgs::JointState>("leg_joints_states", 1);

  ros::Rate loop_rate(10);
 
  std::string robot_desc_string;
  sensor_msgs::JointState joint_msg;
  //n.param("robot_description", robot_desc_string, std::string());



  if (n.getParam("robot_description", robot_desc_string))
  {
    //ROS_INFO("Got param: %s", robot_desc_string.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get param 'my_param'");
  }

  KDL::Tree my_tree;
  
  if (!kdl_parser::treeFromString(robot_desc_string, my_tree) ){
     ROS_ERROR("Failed to construct kdl tree!");
     return false;
   }
  else
  {
    ROS_INFO("Got the tree!");
  }
  KDL::Chain chain;
  float myinput;
  scanf ("%e",&myinput);

//=======================================================
  cout << "What's your name?\n ";
//  kdl_parser::treeFromString(robot_desc_string, my_tree);
  cout << "Schunk?\n ";
  //KDL::Chain chain;
  my_tree.getChain("arm_base_link", "arm_ee_link", chain);
  my_tree.getChain("arm_ee_link", "arm_base_link", chain_ee_b);


  // Create solver based on kinematic chain
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

  // Create joint array
  int nj = chain.getNrOfJoints();
  KDL::JntArray jointpositions = JntArray(nj);

  printf("%s \n","Number of Joints==");
  cout<< nj ;
  printf("\n %s \n","This was the Number of Joint");
  // Assign some values to the joint positions
  for(unsigned int i=0;i<nj;i++){
      float myinput;
      printf ("\nEnter the position of joint %i: ",i);
      //scanf ("%e",&myinput);
      //jointpositions(i)=(double)myinput;
      jointpositions(i)=1.0;
  }

  // Create the frame that will contain the results
  KDL::Frame cartpos;    
  // Calculate forward position kinematics
  bool kinematics_status;
  kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
  if(kinematics_status>=0){
      std::cout << "\n This is joint position= \n"<<cartpos <<std::endl;
      printf("%s \n","Succes, thanks KDL!");
  }else{
      printf("%s \n","Error: could not calculate forward kinematics :(");
  }
  int segmentNR;

  KDL::Jacobian J;
  J.resize(chain.getNrOfJoints());

  KDL::ChainJntToJacSolver jsolver(chain);
  jsolver.JntToJac(jointpositions, J);
  std::cout << "\n No. of rows of Jacobian = \n"<<J.rows() << "\n No. of columns of Jacobian = \n"<<J.columns() << ";\n \n"<<std::endl;
   

   




  


  // Eigen::MatrixXf J_T_(7,6), J_T__(7,6), JJ_T(6,7);
  // //J_T_ = J_.data; //J__(6,chain.getNrOfJoints());
 

  // for (unsigned int i = 0 ; i < chain.getNrOfJoints() ; i++)
  //   {
  //   for (unsigned int j = 0 ; j < 6 ; j++)
  //   //J__(i,j) = J_.data(j,i) ;
  //   //  a = J_.data(j,i);
  //   //J__(j,i) = 
  //   //J_T_(i,j) = 0;
  // if (i == 0){
  //   J_T_(i,j) = 1; 
  // }
  //   else 
  //   {
  //   J_T_(i,j) = 0;
  //   }
  // }
  // J_T__ =   J_T_.transpose();
  // std::cout << "\n Original matrix J_T_  = \n"<< J_T_<< std::endl;
  // std::cout << "\n Tanspose of J_T_  = \n"<< J_T__<< std::endl;


  // MatrixXf JJ_T; 

  // JJ_T = J * J_T;


  // std::cout << "\n  J J'  = \n"<< JJ_T<<std::endl;
  // std::cout << "\n Transpose Jacobian  = \n"<< J.data.transpose()<<std::endl;


  // MatrixXd J_T_ = ;

  MatrixXd JJ_T =   J.data * J.data.transpose();

  EigenSolver<MatrixXd> es(JJ_T);

  std::cout << "\n  J J'  = \n"<< JJ_T<<std::endl;
  std::cout << "\n EigenValue  = \n"<< es.eigenvectors() <<std::endl;
  std::cout << "\n EigenValue  = \n"<< es.eigenvalues() <<std::endl;







  



 //  // KDL class and functions
 //  KDL::Vector v1(1,2,1);
 //  KDL::Vector v2(v1);
 //  KDL::Vector v3= KDL::Vector::Zero();
 //  cout<< v1.x()<<"\n Here you go!\n";
 //  v2 = 2*v1;
 //  //v1 = v1/2;
 //  double a=dot(v1,v2);

 //  cout<<"\n v1.x ="<< v1.x()<<"; v1.y ="<< v1.y()<<" v1.z ="<< v1.z();
 //  cout<<"\n v2.x ="<< v2.x()<<"; v2.y ="<< v2.y()<<" v2.z ="<< v2.z();
 //  cout<<"\n A= !\n"<< a;

 //  cout<< v2.x()<<"\n Here you go2222222222!\n"<<"\n";
 //  KDL::Rotation r1 = KDL::Rotation::Identity();
 //  cout<<"\n r1 ="<< r1(1,1);
 //  KDL::Rotation r3=KDL::Rotation::RotX(.2);
 //  KDL::Rotation r6=KDL::Rotation::RPY(.2,1,1);
 //  r1.SetInverse();
 //  KDL::Rotation r7 = r1.Inverse();
 //  KDL::Vector v5 = r1*v1;

 //  cout<<"\n V511 ="<< v5(0);cout<<"; r12 ="<< v2(1);cout<<"; r13 ="<< v2(2);

 //  KDL::Rotation r5 = r1*r3;
 //  KDL::Rotation r2 = r5.Inverse();
 //  cout<<"\n r11 ="<< r2(0,0);cout<<"; r12 ="<< r2(1,0);cout<<"; r13 ="<< r2(2,0);
 //  cout<<"\n r21 ="<< r2(0,1);cout<<"; r22 ="<< r2(1,1);cout<<"; r23 ="<< r2(2,1);
 //  cout<<"\n r31 ="<< r2(0,2);cout<<"; r32 ="<< r2(1,2);cout<<"; r33 ="<< r2(2,2)<<"\n";


 //  //FRAMES
 //  KDL::Frame f1;
 //  f1 = KDL::Frame::Identity();
 //  KDL::Frame f2(r5,v1);
 //  f2 = f1.Inverse();

 //  cout<<"\n f11 ="<< f2(0,0);cout<<"; r12 ="<< f2(1,0);cout<<"; f13 ="<< f2(2,0);cout<<"; f14 ="<< f2(3,0);
 //  cout<<"\n f21 ="<< f2(0,1);cout<<"; r22 ="<< f2(1,1);cout<<"; f23 ="<< f2(2,1);cout<<"; f23 ="<< f2(3,1);
 //  cout<<"\n f31 ="<< f2(0,2);cout<<"; r32 ="<< f2(1,2);cout<<"; f33 ="<< f2(2,2);cout<<"; f33 ="<< f2(3,2);
 //  cout<<"\n f31 ="<< f2(0,3);cout<<"; r32 ="<< f2(1,3);cout<<"; f33 ="<< f2(2,3);cout<<"; f33 ="<< f2(3,3)<<"\n";
 // // cout<<"\n f31 ="<< f2(0,3);cout<<"; r32 ="<< f2(1,3);cout<<"; f33 ="<< f2(2,3);cout<<"; f33 ="<< f2(3,3);<<"\n";
 //  cout<< "\nF2.M = "<<f2.M(1,1);cout<< "\nF2.P = "<<f2.p(1); 
 //  Frame f3 = f1*f2;

 //  Twist t1;
 //  Twist t2(v1,v2);
 //  Twist t3 = Twist::Zero();

 //  double vx = t2.vel.x();
 //  cout<< "\n VX = "<< vx <<"\n";



 //  Wrench w1(v1,v2);
 //  Wrench w2 = Wrench::Zero();
  
 //  Wrench w3 = w1+w2;

 //  double wx = w3.force.x();
 //  cout<< "\n WX = "<< wx <<"\n";



 //  std::string s1;
 //  n.param<std::string>("my_Pa", s1, "Hey this is it!");
 //  ROS_INFO("Got it! %s", s1.c_str());
 //  std::string s;
  
  /*
  if (n.getParam("my_Pa", s) ) {
    ROS_INFO("Got it! %s", s.c_str());
  }
  else{
    ROS_ERROR("Failed haha!!!");
  }*/
  int count = 0;

  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world "<<count;
    //msg.data = ss.str();
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    //joint_msg_pub.publish(joint_pos);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
