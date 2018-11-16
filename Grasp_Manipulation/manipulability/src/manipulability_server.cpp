
#include "ros/ros.h"
#include "manipulability/Man_service.h"
#include <geometry_msgs/Pose.h>
#include "manipulability/AddTwoInt.h"


#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>


#include <kdl/chainfksolver.hpp>
#include <kdl/frames_io.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <stdio.h>
#include <iostream>
#include <Eigen/LU>
#include <Eigen/Dense>

#include <math.h>

using namespace std;
using namespace KDL;
using namespace Eigen;





KDL::Chain chain;
KDL::Tree my_tree;

class PG_obj {
    KDL::Jacobian Jacob;
  public:
    float manip(KDL::Jacobian);
};


float PG_obj::manip (KDL::Jacobian J){
  // std::cout << "\n No. of rows of Jacobian IS= \n"<<J.rows() << "\n No. of columns of Jacobian IS= \n"<<J.columns() << ";\n \n"<<std::endl;
  MatrixXd JJ_T =   J.data * J.data.transpose();
  EigenSolver<MatrixXd> es(JJ_T);




  VectorXd eval = es.eigenvalues().col(0).real();
  float evp = eval.transpose()*eval;
  // std::cout << "\n EIGEN PRODUCT  = \n"<< eval.size() <<std::endl;
  
  float m = 0;
  Eigen::VectorXd ones(6);
  ones[0] = 1;
  ones[1] = 1;
  ones(2) = 1;
  ones(3) = 1;
  ones(4) = 1;
  ones(5) = 1;
  // std::cout << "\n ONES  = \n"<<  ones <<std::endl;
  for(unsigned int i=0;i<eval.size();i++){
    m =    eval(i)  * abs( ones.transpose() * es.eigenvectors().col(i).real() );

  }
  return m;
}

bool manip1(manipulability::Man_service::Request  &request,
            manipulability::Man_service::Response &res)
{
  res.pose_back.position.x   =request.pose.position.x +0.25;
  res.pose_back.position.y   =request.pose.position.y+ 0.25;
  res.pose_back.position.z   =request.pose.position.z;
  res.pose_back.orientation.x=request.pose.orientation.x;
  res.pose_back.orientation.y=request.pose.orientation.y;
  res.pose_back.orientation.z=request.pose.orientation.z;
  res.pose_back.orientation.w=request.pose.orientation.w;
  //std::cout<<"A="<<req.a<<"; B="<<req.b<<"; This is the computed value:"<<res.sum<<std::endl;
  //ROS_INFO("request: x=%ld, y=%ld", (int)req.a, (int)req.b);
  //ROS_INFO("SSending back response: [%ld]", (float)res.sum);

  my_tree.getChain("arm_base_link", "arm_ee_link", chain);
  JntArray q(chain.getNrOfJoints());
  KDL::Jacobian Jac;
  KDL::ChainJntToJacSolver jsolver(chain);
  PG_obj obj1;

  jsolver.JntToJac(q, Jac);
  float m = obj1.manip(Jac);
  std::cout << "\n This is MANIPULABILITY = \n"<<m << std::endl;  
  return true;
}




bool add(manipulability::AddTwoInt::Request  &req,
         manipulability::AddTwoInt::Response &res)
{
  res.sum = 1;//req.a * req.b ;
  std::cout<<"A="<<req.a<<"; B="<<req.b<<"; This is the computed value:"<<res.sum<<std::endl;
  ROS_INFO("request: x=%ld, y=%ld", (int)req.a, (int)req.b);
  //ROS_INFO("SSending back response: [%ld]", (float)res.sum);
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;





  std::string robot_desc_string;



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
  KDL::Chain chain, chain_ee_b;
  // float myinput;
  // scanf ("%e",&myinput);



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
   



  PG_obj obj1;
  obj1.manip(J);
  cout << "Manipuilability: " << obj1.manip(J) << endl;



  MatrixXd JJ_T =   J.data * J.data.transpose();

  EigenSolver<MatrixXd> es(JJ_T);


KDL::ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
KDL::ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
ChainIkSolverPos_NR iksolver1(chain,fksolver1,iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
 
 //Creation of jntarrays:
JntArray q(chain.getNrOfJoints());
JntArray q_init(chain.getNrOfJoints());


//Set destination frame
Rotation r = Rotation::Identity();

for(unsigned int i=0;i<nj;i++){
  float x =0.2 + (i * 0.10);
  Vector v( x,    0.275989,    0.398982);
  Frame F_dest(r, v);
  int ret = iksolver1.CartToJnt(q_init,F_dest,q);
  std::cout << "\n This is EE = \n"<<  v << std::endl;
  std::cout << "\n This is Joint Positions = \n" <<  q.operator()(0) << ","<<  q.operator()(1) << ","<<  q.operator()(2) << ","<<  q.operator()(3) << ","<<  q.operator()(4) << "," <<  q.operator()(5) << ","<<  q.operator()(6) <<";\n \n"<<std::endl; 
  jsolver.JntToJac(q, J);
  float m = obj1.manip(J);
  std::cout << "\n This is MANIPULABILITY = \n"<<m << std::endl;
}


KDL::Frame position1;
fksolver1.JntToCart(jointpositions,position1);
std::cout << "\n position1  = \n"<< position1<<std::endl;
std::cout << "\n q  = \n"<< q(0,3) <<std::endl;





  ros::ServiceServer service = n.advertiseService("add_two_ints", manip1);
  ROS_INFO("Ready to add two numbers.");
  ros::spin();

  return 0;
}





/*

#include "ros/ros.h"
#include "manipulability/AddTwoInt.h"

bool add(manipulability::AddTwoInt::Request  &req,
         manipulability::AddTwoInt::Response &res)
{
  res.sum = req.a * req.b;
  std::cout<<"A="<<req.a<<"; B="<<req.b<<"; This is the computed value:"<<res.sum<<std::endl;
  //ROS_INFO("request: x=%ld, y=%ld", (int)req.a, (int)req.b);
  //ROS_INFO("SSending back response: [%ld]", (float)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two numbers.");
  ros::spin();

  return 0;
}

*/