#include <ros/ros.h>
#include "std_msgs/String.h"
#include <math.h>

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

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <stdio.h>
#include <iostream>
#include <Eigen/LU>
#include <Eigen/Dense>

#include <sstream>

#include <iostream>
using namespace std;
using namespace KDL;
using namespace Eigen;
#include <trac_ik/trac_ik.hpp>



class PG_obj {
    KDL::Jacobian Jacob;
  public:
    float manip(KDL::Jacobian);
};


float PG_obj::manip (KDL::Jacobian J){
  // std::cout << "\n No. of rows of Jacobian IS= \n"<<J.rows() << "\n No. of columns of Jacobian IS= \n"<<J.columns() << ";\n \n"<<std::endl;
  MatrixXd JJ_T =   J.data * J.data.transpose();
  EigenSolver<MatrixXd> es(JJ_T);

  /*
  std::cout << "\n  J J'  = \n"<< JJ_T<<std::endl;
  std::cout << "\n EigenValue  = \n"<< es.eigenvectors().col(0) <<std::endl;
  std::cout << "\n EigenValue  = \n"<< es.eigenvalues().col(0)(0).real() <<std::endl;
  std::cout << "\n EigenValue  = \n"<< es.eigenvalues().real() <<std::endl;
  */


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
    /*
    std::cout << "\n EIGEN PRODUCT  = \n"<<  es.eigenvectors().col(i).real().transpose() * es.eigenvectors().col(i).real() <<std::endl;
    std::cout << "\n ABS EIGEN SUM  = \n"<<  abs( ones.transpose() * es.eigenvectors().col(i).real() ) <<std::endl;
    std::cout << "\n EIGEN SUM  = \n"<<  ( ones.transpose() * es.eigenvectors().col(i).real() ) <<std::endl;
    std::cout << "\n Manipuilability  = \n"<<  m <<std::endl;
    */
  }
  return m;
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "manip");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
 
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

  // int m = 20; 
  // MatrixXf jointpos(nj, m);

  // for (unsigned int i = 0 ; i < nj ; i++)
  //   {
  //     for (unsigned int j = 0 ; j < m ; j++){
  //       jointpos(i,j) = -M_PI/2 + j * M_PI/m;
  //     }
  //   }

  // for (unsigned int i = 0 ; i < nj ; i++)
  //   {
  //     for (unsigned int j = 0 ; j < m ; j++){
  //       for (unsigned int j = 0 ; j < m ; j++){
  //       for (unsigned int j = 0 ; j < m ; j++){
  //       for (unsigned int j = 0 ; j < m ; j++){
  //       for (unsigned int j = 0 ; j < m ; j++){
  //       for (unsigned int j = 0 ; j < m ; j++){
  //       for (unsigned int j = 0 ; j < m ; j++){
  //         jointpos(i,j) = -M_PI/2 + j * M_PI/m;
  //       }}}}}}}
  //   }
  // std::cout << "\n  jointpositions  = \n"<< jointpos <<std::endl;


  MatrixXd JJ_T =   J.data * J.data.transpose();

  EigenSolver<MatrixXd> es(JJ_T);

  /*
  std::cout << "\n  J J'  = \n"<< JJ_T<<std::endl;
  std::cout << "\n EigenValue  = \n"<< es.eigenvectors() <<std::endl;
  std::cout << "\n EigenValue  = \n"<< es.eigenvalues() <<std::endl;
  */


// Create solver based on kinematic chain
// ChainFkSolverPos_recursive fk_solver = ChainFkSolverPos_recursive(chain);
//Creation of the solvers:
//Chain chain1;
ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
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


// KDL::ChainIkSolverPos_NR_JL ik_solver(KDL::Chain chain, KDL::JntArray lower_joint_limits, KDL::JntArray upper_joint_limits, fk_solver, vik_solver, int num_iterations, double error);

// TRAC_IK::TRAC_IK ik_solver(Chain chain1, KDL::JntArray lower_joint_limits, KDL::JntArray upper_joint_limits, double timeout_in_secs=0.005, double error=1e-5, TRAC_IK::SolveType type=TRAC_IK::Speed);  

//TRAC_IK::TRAC_IK ik_solver(string arm_base_link, string arm_ee_link, string URDF_param="robot_desc_string", double timeout_in_secs=0.005, double error=1e-5, TRAC_IK::SolveType type=TRAC_IK::Speed);
  // ik_solver.SetSolveType (SolveType _type)
// int rc = ik_solver.CartToJnt(KDL::JntArray joint_seed = jointpositions, KDL::Frame desired_end_effector_pose = [.5 0 0 0 0 0 1], KDL::JntArray& return_joints);
  KDL::Frame position1;
  fksolver1.JntToCart(jointpositions,position1);
  std::cout << "\n position1  = \n"<< position1<<std::endl;
  std::cout << "\n q  = \n"<< q(0,3) <<std::endl;

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
