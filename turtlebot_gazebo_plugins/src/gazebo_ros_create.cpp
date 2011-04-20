//#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <turtlebot_node/TurtlebotSensorState.h>
#include <turtlebot_plugins/gazebo_ros_create.h>

#include <gazebo/Joint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/Entity.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/XMLConfig.hh>

#include <ros/time.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_create", GazeboRosCreate);

enum {LEFT= 0, RIGHT=1, FRONT=2, REAR=3};

GazeboRosCreate::GazeboRosCreate( Entity *parent )
  : Controller(parent)
{
  ros::MultiThreadedSpinner s(1);
  boost::thread spinner_thread( boost::bind( &ros::spin, s) );

  my_parent_ = dynamic_cast<Model*>(parent);

  if (!my_parent_)
    gzthrow("Gazebo_ROS_Create controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  node_namespaceP_ = new ParamT<std::string>("node_namespace","",0);
  left_wheel_joint_nameP_ = new ParamT<std::string>("left_wheel_joint","left_wheel_joint",1);
  right_wheel_joint_nameP_ = new ParamT<std::string>("right_wheel_joint","right_wheel_joint",1);
  front_castor_joint_nameP_ = new ParamT<std::string>("front_castor_joint","front_castor_joint",1);
  rear_castor_joint_nameP_ = new ParamT<std::string>("rear_castor_joint","rear_castor_joint",1);
  wheel_sepP_ = new ParamT<float>("wheel_separation", 0.34,1);
  wheel_diamP_ = new ParamT<float>("wheel_diameter", 0.15,1);
  torqueP_ = new ParamT<float>("torque", 10.0, 1);
  Param::End();

  wheel_speed_ = new float[2];
  wheel_speed_[LEFT] = 0.0;
  wheel_speed_[RIGHT] = 0.0;

  joints_[0] = NULL;
  joints_[1] = NULL;
  joints_[2] = NULL;
  joints_[3] = NULL;
}

GazeboRosCreate::~GazeboRosCreate()
{
  delete [] wheel_speed_;
  delete wheel_diamP_;
  delete wheel_sepP_;
  delete torqueP_;
  delete node_namespaceP_;
  delete left_wheel_joint_nameP_;
  delete right_wheel_joint_nameP_;
  delete front_castor_joint_nameP_;
  delete rear_castor_joint_nameP_;
  delete rosnode_;
}
    
void GazeboRosCreate::LoadChild( XMLConfigNode *node )
{
  node_namespaceP_->Load(node);
  left_wheel_joint_nameP_->Load(node);
  right_wheel_joint_nameP_->Load(node);
  front_castor_joint_nameP_->Load(node);
  rear_castor_joint_nameP_->Load(node);
  wheel_sepP_->Load(node);
  wheel_diamP_->Load(node);
  torqueP_->Load(node);

  joints_[LEFT] = my_parent_->GetJoint(**left_wheel_joint_nameP_);
  joints_[RIGHT] = my_parent_->GetJoint(**right_wheel_joint_nameP_);
  joints_[FRONT] = my_parent_->GetJoint(**front_castor_joint_nameP_);
  joints_[REAR] = my_parent_->GetJoint(**rear_castor_joint_nameP_);

  if (!joints_[LEFT])
    gzthrow("The controller couldn't get left hinge joint");

  if (!joints_[RIGHT])
    gzthrow("The controller couldn't get right hinge joint");

  if (!joints_[FRONT])
    gzthrow("The controller couldn't get front castor joint");

  if (!joints_[REAR])
    gzthrow("The controller couldn't get rear castor joint");

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_turtlebot", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle( **node_namespaceP_ );

  cmd_vel_sub_ = rosnode_->subscribe("cmd_vel", 1, &GazeboRosCreate::OnCmdVel, this );

  sensor_state_pub_ = rosnode_->advertise<turtlebot_node::TurtlebotSensorState>("sensor_state", 1);
  odom_pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);

  joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 1);

  js_.name.push_back( **left_wheel_joint_nameP_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( **right_wheel_joint_nameP_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( **front_castor_joint_nameP_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( **rear_castor_joint_nameP_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);
}

void GazeboRosCreate::InitChild()
{
}

void GazeboRosCreate::FiniChild()
{
  rosnode_->shutdown();
}

void GazeboRosCreate::UpdateChild()
{
  double wd, ws;
  double d1, d2;
  double dr, da;
  Time step_time;

  wd = **(wheel_diamP_);
  ws = **(wheel_sepP_);

  d1 = d2 = 0;
  dr = da = 0;

  step_time = Simulator::Instance()->GetSimTime() - prev_update_time_;
  prev_update_time_ = Simulator::Instance()->GetSimTime();

  // Distance travelled by front wheels
  d1 = step_time.Double() * (wd / 2) * joints_[LEFT]->GetVelocity(0);
  d2 = step_time.Double() * (wd / 2) * joints_[RIGHT]->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / ws;

  // Compute odometric pose
  odom_pose_[0] += dr * cos( odom_pose_[2] );
  odom_pose_[1] += dr * sin( odom_pose_[2] );
  odom_pose_[2] += da;

  // Compute odometric instantaneous velocity
  odom_vel_[0] = dr / step_time.Double();
  odom_vel_[1] = 0.0;
  odom_vel_[2] = da / step_time.Double();

  joints_[LEFT]->SetVelocity( 0, wheel_speed_[LEFT] / (wd/2.0) );
  joints_[RIGHT]->SetVelocity( 0, wheel_speed_[RIGHT] / (wd / 2.0) );

  joints_[LEFT]->SetMaxForce( 0, **(torqueP_) );
  joints_[RIGHT]->SetMaxForce( 0, **(torqueP_) );


  /*btQuaternion qt;
  //qt.setRPY(this->posIface->data->pose.roll, this->posIface->data->pose.pitch, this->posIface->data->pose.yaw);
  qt.setRPY(0,0,0);
  btVector3 vt(odom_pose_[0], odom_pose_[1], odom_pose_[2]);
  tf::Transform base_link_to_odom(qt, vt);
 transform_broadcaster_.sendTransform( tf::StampedTransform( base_link_to_odom, ros::Time::now(), "/turtlebot_node/odom", "turtlebot_node/base_footprint"));
 */

  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "/turtlebot_node/odom";
  odom.child_frame_id = "/turtlebot_node/base_footprint";
  odom.pose.pose.position.x = odom_pose_[0];
  odom.pose.pose.position.y = odom_pose_[1];
  odom.pose.pose.position.z = 0;

  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = 0;
  odom.pose.pose.orientation.w = 1;

  double pose_cov[36] = { 1e-3, 0, 0, 0, 0, 0,
                          0, 1e-3, 0, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e3};

  memcpy( &odom.pose.covariance[0], pose_cov, sizeof(double)*36 );
  memcpy( &odom.twist.covariance[0], pose_cov, sizeof(double)*36 );

  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;

  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;

  odom_pub_.publish( odom ); 

  js_.position[0] = joints_[LEFT]->GetAngle(0).GetAsRadian();
  js_.velocity[0] = joints_[LEFT]->GetVelocity(0);

  js_.position[1] = joints_[RIGHT]->GetAngle(0).GetAsRadian();
  js_.velocity[1] = joints_[RIGHT]->GetVelocity(0);

  js_.position[2] = joints_[FRONT]->GetAngle(0).GetAsRadian();
  js_.velocity[2] = joints_[FRONT]->GetVelocity(0);

  js_.position[3] = joints_[REAR]->GetAngle(0).GetAsRadian();
  js_.velocity[3] = joints_[REAR]->GetVelocity(0);

  joint_state_pub_.publish( js_ );
}

void GazeboRosCreate::OnCmdVel( const geometry_msgs::TwistConstPtr &msg)
{
  double vr, va;
  vr = msg->linear.x;
  va = msg->angular.z;

  wheel_speed_[LEFT] = vr - va * **(wheel_sepP_) / 2;
  wheel_speed_[RIGHT] = vr + va * **(wheel_sepP_) / 2;
}
