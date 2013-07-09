/*
 * Copyright (c) 2010, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 /** \author Benjamin Cohen */

#include <pr2_collision_checker/sbpl_arm_model.h>
#include <tf_conversions/tf_kdl.h>
#include <pr2_arm_kinematics/pr2_arm_kinematics_utils.h>

using namespace std;

namespace sbpl_arm_planner {

SBPLArmModel::SBPLArmModel(FILE* arm_file) : fk_solver_(NULL), pr2_arm_ik_solver_(NULL)
{
  name_="right_arm"; //right_arm or left_arm
  num_joints_ = 7;
  num_links_ = 3;
  chain_root_name_ = "base_link";
  chain_tip_name_ = "r_wrist_roll_link";
  planning_joint_name_ = "r_wrist_roll_link";
  reference_frame_ = "base_link";
    
  joints_.resize(num_joints_);
  links_.resize(num_links_);

  for(unsigned int i = 0; i < joints_.size(); ++i)
    joints_[i].continuous = true;

  //parse arm description file
  if(!getArmDescription(arm_file))
    SBPL_WARN("[arm] Parsing of sbpl arm description file failed. Will attempt to use defaults.");
 
  debug_log_ = "arm";
  max_radius_ = 0;
  resolution_ = 0.02;
  ik_search_inc_ = 0.02;
}

SBPLArmModel::~SBPLArmModel()
{
  if(fk_solver_ != NULL)
    delete fk_solver_;
  if(pr2_arm_ik_solver_ != NULL)
    delete pr2_arm_ik_solver_;
}

void SBPLArmModel::setDebugLogName(std::string stream_name)
{
  debug_log_ = stream_name;
}

bool SBPLArmModel::getArmDescription(FILE* fCfg)
{
  if(fCfg == NULL)
    return false;

  char sTemp[1024];
  int i;

  if(fscanf(fCfg,"%s",sTemp) < 1)
    SBPL_WARN("Parsed string has length < 1.");
  while(!feof(fCfg) && strlen(sTemp) != 0)
  {
    if(strcmp(sTemp, "name:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      name_ = sTemp;
    }
    else if(strcmp(sTemp, "number_of_joints:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      num_joints_ = atoi(sTemp);
      joints_.resize(num_joints_);
      if(num_joints_ < 6)
        ROS_ERROR("Are you sure there are %d joints in this arm?", num_joints_);
    }
    else if(strcmp(sTemp, "number_of_links:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      num_links_ = atoi(sTemp);
      links_.resize(num_links_);
      if(num_links_ < 2)
        ROS_ERROR("Are you sure there are %d links in this arm?", num_links_);
    }
    else if(strcmp(sTemp, "index_of_planning_joint:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      planning_joint_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "kdl_chain_root_name:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      chain_root_name_.assign(sTemp);
    }
    else if(strcmp(sTemp, "kdl_chain_tip_name:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      chain_tip_name_.assign(sTemp);
    }
    else if(strcmp(sTemp, "planning_joint_name:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      planning_joint_name_.assign(sTemp);
    }
    else if(strcmp(sTemp, "min_joint_limits:") == 0)
    {
      for(i = 0; i < num_joints_; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1.");
        joints_[i].min = atof(sTemp);

        if(joints_[i].min != 0.0)
          joints_[i].continuous = false;
      }
    }
    else if(strcmp(sTemp, "max_joint_limits:") == 0)
    {
      for(i = 0; i < num_joints_; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1.");
        joints_[i].max = atof(sTemp);
        
        if(joints_[i].max != 0.0)
          joints_[i].continuous = false;
      }
    }
    else if(strcmp(sTemp, "radius_of_links(meters):") == 0)
    {
      for(i = 0; i < num_links_; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1.");
        links_[i].radius = atof(sTemp);
      }
    }
    else if(strcmp(sTemp, "length_of_links(meters):") == 0)
    {
      for(i = 0; i < num_links_; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1.");
        links_[i].length = atof(sTemp);
      }
    }
    else if(strcmp(sTemp, "ik_search_discretization(radians):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_WARN("Parsed string has length < 1.");
      ik_search_inc_ = atof(sTemp);
    }
    else if(strcmp(sTemp, "collision_cuboids:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1.");
     
      num_collision_cuboids_  = atoi(sTemp);
      collision_cuboids_.resize(num_collision_cuboids_);
      for(i = 0; i < num_collision_cuboids_; i++)
      {
        collision_cuboids_[i].resize(6);
        for(int j = 0; j < 6; j++)
        {
          if(fscanf(fCfg,"%s",sTemp) < 1)
            SBPL_WARN("Parsed string has length < 1.");
          collision_cuboids_[i][j] = atof(sTemp);
        }
      }
      SBPL_DEBUG("Number of collision cuboids: %d", num_collision_cuboids_);
    }
    else if(strcmp(sTemp, "kdl_indeces_of_link_tips:") == 0)
    {
      joint_indeces_.resize(num_links_+1);
      for(i = 0; i < num_links_ + 1; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1.");

        joint_indeces_[i] = atoi(sTemp);
      }
    }
    
    if(fscanf(fCfg,"%s",sTemp) < 1)
      SBPL_DEBUG("Parsed string has length < 1.");
  }

  if(name_.compare("left_arm") == 0 || name_.compare("left") == 0)
  {
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_LEFT_SHOULDER_PAN_ANGLE);
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_LEFT_SHOULDER_PITCH_ANGLE);
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_LEFT_UPPER_ARM_ROLL_ANGLE);
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_LEFT_ELBOW_FLEX_ANGLE);
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_LEFT_FOREARM_ROLL_ANGLE);
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_LEFT_WRIST_PITCH_ANGLE);
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_LEFT_WRIST_ROLL_ANGLE);
  }
  else
  {
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_RIGHT_SHOULDER_PAN_ANGLE);
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_RIGHT_SHOULDER_PITCH_ANGLE);
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_RIGHT_UPPER_ARM_ROLL_ANGLE);
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_RIGHT_ELBOW_FLEX_ANGLE);
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_RIGHT_FOREARM_ROLL_ANGLE);
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_RIGHT_WRIST_PITCH_ANGLE);
    debug_code_types_.push_back(sbpl_arm_planner::INVALID_RIGHT_WRIST_ROLL_ANGLE);
  }

  SBPL_DEBUG("[arm] Finished parsing arm file.");
  return true;
}

void SBPLArmModel::setResolution(double resolution)
{
  resolution_ = resolution;

  //store discretized link radii
  for(int i = 0; i < num_links_; i++)
  {
    links_[i].radius_c = (short unsigned int)((links_[i].radius / resolution_) + 0.5);
    links_[i].length_c = (short unsigned int)((links_[i].length / resolution_) + 0.5);

    if(links_[i].radius_c < 1 || links_[i].length_c < 1)
      ROS_ERROR("[arm] The radius or length of link %d is practically zero. Something must be wrong in the arm description file.", i);
    if(links_[i].radius_c > 100 || links_[i].length_c > 100)
      ROS_ERROR("[arm] The radius or length of link %d is over 100 cells. Is that right? Check your arm description file.", i);
    if(links_[i].radius_c > max_radius_)
      max_radius_ = links_[i].radius_c;
  }
}

bool SBPLArmModel::initKDLChainFromParamServer()
{
  ros::NodeHandle nh("~");
  std::string robot_description;
  std::string robot_param;

  nh.searchParam("robot_description",robot_param);
  nh.param<std::string>(robot_param,robot_description,"");

  if(robot_description.empty())
  {
    SBPL_ERROR("[arm] Unable to get robot_description from rosparam server.");
    return false;
  }

  return initKDLChain(robot_description);
}

bool SBPLArmModel::initKDLChain(const std::string &fKDL)
{
  if (!kdl_parser::treeFromString(fKDL, kdl_tree_))
  {
    SBPL_ERROR("[arm] Failed to parse tree from manipulator description file.");
    return false;;
  }

  if (!kdl_tree_.getChain(chain_root_name_, chain_tip_name_, chain_))
  {
    SBPL_ERROR("[arm] Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }

  fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_); 
  jnt_pos_in_.resize(chain_.getNrOfJoints());
  jnt_pos_out_.resize(chain_.getNrOfJoints());

  ROS_INFO("[arm] The FK chain has %d segments with %d joints.", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  ROS_INFO("[arm] root: %s tip: %s.", chain_root_name_.c_str(), chain_tip_name_.c_str());
  
  // initialize PR2ArmIKSolver
  robot_model_.initString(fKDL);
  pr2_arm_ik_solver_ = new pr2_arm_kinematics::PR2ArmIKSolver(robot_model_, "torso_lift_link", planning_joint_name_, ik_search_inc_, 2); 

  if(!pr2_arm_ik_solver_->active_)
  {
    SBPL_ERROR("[arm] Error: the IK solver is NOT active.");
    return false;
  }

  kinematics_msgs::KinematicSolverInfo ik_info;
  pr2_arm_ik_solver_->getSolverInfo(ik_info);

  ROS_DEBUG_NAMED(debug_log_,"[%s] PR2ArmIKSolver Info:", name_.c_str());
  for(size_t i = 0; i < ik_info.limits.size(); ++i)
    ROS_DEBUG_NAMED(debug_log_, "[arm] %s: {%f %f})", ik_info.limits[i].joint_name.c_str(), ik_info.limits[i].min_position,ik_info.limits[i].max_position);

  ik_jnt_pos_in_.resize(num_joints_);
  ik_jnt_pos_out_.resize(num_joints_);

  // added for Cartesian Planner
  if(!pr2_arm_ik_.init(robot_model_, "torso_lift_link", planning_joint_name_))
    SBPL_ERROR("[arm] Failed to initialize pr2_arm_ik.");

  pr2_arm_ik_.getSolverInfo(ik_info);
  ROS_DEBUG_NAMED(debug_log_, "[%s] PR2ArmIK Info:", name_.c_str());
  for(size_t i = 0; i < ik_info.limits.size(); ++i)
    ROS_DEBUG_NAMED(debug_log_, "[arm] %s: {%f %f})", ik_info.limits[i].joint_name.c_str(), ik_info.limits[i].min_position,ik_info.limits[i].max_position);

  ROS_DEBUG_NAMED(debug_log_, "[arm] Initialized pr2_arm_ik for cartesian planner for %s.", planning_joint_name_.c_str());
  
  return true;
}

void SBPLArmModel::parseKDLTree()
{
  num_kdl_joints_ = kdl_tree_.getNrOfJoints();

  SBPL_DEBUG("%d KDL joints:",num_kdl_joints_);

  // create the joint_segment_mapping, which used to be created by the URDF -> KDL parser
  // but not any more, but the rest of the code depends on it, so we simply generate the mapping here:
  KDL::SegmentMap segment_map = kdl_tree_.getSegments();

  for (KDL::SegmentMap::const_iterator it = segment_map.begin(); it != segment_map.end(); ++it)
  {
    if (it->second.segment.getJoint().getType() != KDL::Joint::None)
    {
      std::string joint_name = it->second.segment.getJoint().getName();
      SBPL_DEBUG("%s",joint_name.c_str());
      std::string segment_name = it->first;
      joint_segment_mapping_.insert(make_pair(joint_name, segment_name));
    }
  }

  kdl_number_to_urdf_name_.resize(num_kdl_joints_);
  // Create the inverse mapping - KDL segment to joint name
  // (at the same time) Create a mapping from KDL numbers to URDF joint names and vice versa
  for (map<string, string>::iterator it = joint_segment_mapping_.begin(); it!= joint_segment_mapping_.end(); ++it)
  {
    std::string joint_name = it->first;
    std::string segment_name = it->second;
  
    std::cout << joint_name << " -> " << segment_name << std::endl;
  
    segment_joint_mapping_.insert(make_pair(segment_name, joint_name));
    int kdl_number = kdl_tree_.getSegment(segment_name)->second.q_nr;
    if (kdl_tree_.getSegment(segment_name)->second.segment.getJoint().getType() != KDL::Joint::None)
    {
      std::cout << "KDL number is: " << kdl_number << std::endl;
   
      kdl_number_to_urdf_name_[kdl_number] = joint_name;
      urdf_name_to_kdl_number_.insert(make_pair(joint_name, kdl_number));
    }
  }
}

KDL::Frame SBPLArmModel::computeBodyFK(const BodyPose pose){
  /*
  {
    KDL::Rotation r1;
    r1.DoRotZ(1.57);
    KDL::Vector t1(10,5,0.0);
    KDL::Frame base_footprint_in_map(r1,t1);

    KDL::Rotation r2;
    KDL::Vector t2(-0.05,0.0,0.803+0);
    KDL::Frame torso_lift_in_base_footprint(r2,t2);

    KDL::Frame f = base_footprint_in_map * torso_lift_in_base_footprint;

    printf("map_base %f %f %f\n",base_footprint_in_map.p.x(),base_footprint_in_map.p.y(),base_footprint_in_map.p.z());
    printf("torso_base %f %f %f\n",torso_lift_in_base_footprint.p.x(),torso_lift_in_base_footprint.p.y(),torso_lift_in_base_footprint.p.z());
    printf("f %f %f %f\n",f.p.x(),f.p.y(),f.p.z());
  }
  */



  KDL::Rotation r1;
  r1.DoRotZ(pose.theta);
  KDL::Vector t1(pose.x,pose.y,0.0);
  KDL::Frame base_footprint_in_map(r1,t1);

  KDL::Rotation r2;
  //TODO: confirm this bad boy
  KDL::Vector t2(-0.05,0.0,0.803+pose.z);
  KDL::Frame torso_lift_in_base_footprint(r2,t2);

  KDL::Frame f = base_footprint_in_map * torso_lift_in_base_footprint;


  //double wr,wp,wy;
  //base_footprint_in_map.M.GetRPY(wr,wp,wy);
  //printf("base in map: %f %f %f\n",wr,wp,wy);
  //torso_lift_in_base_footprint.M.GetRPY(wr,wp,wy);
  //printf("torso in base: %f %f %f\n",wr,wp,wy);
  //f.M.GetRPY(wr,wp,wy);
  //printf("torso in map: %f %f %f\n",wr,wp,wy);
  return f;
}

bool SBPLArmModel::computeArmFK(const std::vector<double> angles, int frame_num, KDL::Frame *frame_out){
  // frame_num pose in torso_lift_link frame

  //KDL::Frame fk_frame_out;
  for(int i = 0; i < num_joints_; ++i)
    jnt_pos_in_(i) = angles::normalize_angle(angles[i]);

  /*
  {
    fk_solver_->JntToCart(jnt_pos_in_, *frame_out, 10);
    printf("arm_torso %f %f %f\n",frame_out->p.x(),frame_out->p.y(),frame_out->p.z());
  }
  */


  if(fk_solver_->JntToCart(jnt_pos_in_, *frame_out, frame_num) < 0)
  {
    SBPL_ERROR("JntToCart returned < 0. Exiting.");
    return false;
  }

  /*
  double x,y,z;
  fk_frame_out.M.GetRPY(x,y,z);
  ROS_DEBUG("[arm] frame_out xyz: %0.2f %0.2f %0.2f rpy: %0.2f %0.2f %0.2f (verify it's in torso_lift_link)", fk_frame_out.p.x(),fk_frame_out.p.y() , fk_frame_out.p.z(),x,y,z);

  *frame_out = fk_frame_out;
  */
  return true;
}

bool SBPLArmModel::computeFK(const std::vector<double> angles, const BodyPose pose, int frame_num, KDL::Frame *frame_out)
{
  // frame_num pose in map frame
  KDL::Frame torso_to_link;
  if(!computeArmFK(angles, frame_num, &torso_to_link))
  {
    ROS_ERROR("[arm] Failed to compute forward kinematics for the arm.");
    return false;
  }

  KDL::Frame fk_body = computeBodyFK(pose);
  ROS_DEBUG("[arm] map->torso=(%f %f %f) torso->wrist=(%f %f %f)",fk_body.p.x(),fk_body.p.y(),fk_body.p.z(),torso_to_link.p.x(),torso_to_link.p.y(),torso_to_link.p.z());
  *frame_out = fk_body * torso_to_link;
  ROS_DEBUG("[arm] map->wrist=(%f %f %f)",frame_out->p.x(),frame_out->p.y(),frame_out->p.z());

  return true;
}

bool SBPLArmModel::computeFK(const std::vector<double> angles, const BodyPose pose, int frame_num, std::vector<double> &xyzrpy)
{
  KDL::Frame frame_out;
  xyzrpy.resize(6);
  if(computeFK(angles,pose,frame_num,&frame_out))
  {
    xyzrpy[0] = frame_out.p[0];
    xyzrpy[1] = frame_out.p[1];
    xyzrpy[2] = frame_out.p[2];

    frame_out.M.GetRPY(xyzrpy[3],xyzrpy[4],xyzrpy[5]);
    return true;
  }
  
  return false;
}

bool SBPLArmModel::computeFK(const std::vector<double> angles, const BodyPose pose, int frame_num, std::vector<double> &xyzrpy, int sol_num)
{
  KDL::Frame frame_out;
  xyzrpy.resize(6);
  if(computeFK(angles,pose,frame_num,&frame_out))
  {
    //convert to quaternion then back to rpy
    //so we can choose which rpy solution we want
    tf::Pose pose;
    tf::PoseKDLToTF(frame_out,pose);
    pose.getBasis().getRPY(xyzrpy[3],xyzrpy[4],xyzrpy[5],sol_num);

    xyzrpy[0] = frame_out.p[0];
    xyzrpy[1] = frame_out.p[1];
    xyzrpy[2] = frame_out.p[2];

    return true;
  }

  return false;
}

bool SBPLArmModel::computeEndEffPose(const std::vector<double> angles, const BodyPose pose, double  R_target[3][3], double* x, double* y, double* z, double* axis_angle)
{
  KDL::Frame f_endeff;

  if(computeFK(angles, pose, planning_joint_, &f_endeff))
  {
    *x = f_endeff.p[0];
    *y = f_endeff.p[1];
    *z = f_endeff.p[2];

    *axis_angle = frameToAxisAngle(f_endeff, R_target);
    return true;
  }
  return false;
}

double SBPLArmModel::frameToAxisAngle(const KDL::Frame frame, const double R_target[3][3])
{
  double R_endeff[3][3];

  //end effector rotation matrix
  for(unsigned int i = 0; i < 3; i++)
    for(unsigned int j = 0; j < 3; j++)
      R_endeff[i][j] = frame.M(i,j);

  //compute axis angle (temporary...switch to GetRotAngle())
  return getAxisAngle(R_endeff, R_target);
}

bool SBPLArmModel::computeIK(const std::vector<double> pose, const std::vector<double> start, std::vector<double> &solution)
{
  //pose: {x,y,z,r,p,y}  or {x,y,z,qx,qy,qz,qw}
  KDL::Frame frame_des;

  //fill the goal frame
  frame_des.p.x(pose[0]);
  frame_des.p.y(pose[1]);
  frame_des.p.z(pose[2]);
  
  //RPY
  if(pose.size() == 6)
    frame_des.M = KDL::Rotation::RPY(pose[3],pose[4],pose[5]);
  //Quaternion
  else
    frame_des.M = KDL::Rotation::Quaternion(pose[3],pose[4],pose[5],pose[6]);

  //transform from reference_frame_ to root_link_
  //frame_des.p = transform_inverse_ * frame_des.p;

  //fill the start configuration (to be used as a seed)
  for(unsigned int i = 0; i < start.size(); i++)
    ik_jnt_pos_in_(i) = angles::normalize_angle(start[i]); // must be normalized for CartToJntSearch

  //call IK solver
  if(pr2_arm_ik_solver_->CartToJntSearch(ik_jnt_pos_in_, frame_des, ik_jnt_pos_out_, 1.0) < 0)
    return false;

  solution.resize(start.size());
  for(size_t i = 0; i < solution.size(); ++i)
    solution[i] = ik_jnt_pos_out_(i);

  return true;
}

bool SBPLArmModel::computeIK(const KDL::Frame &frame, const std::vector<double> start, std::vector<double> &solution)
{
  KDL::Frame f = frame;

  //transform from reference_frame_ to root_link_
  //f.p = transform_inverse_ * frame.p;

  //fill the start configuration (to be used as a seed)
  for(unsigned int i = 0; i < start.size(); i++)
    ik_jnt_pos_in_(i) = angles::normalize_angle(start[i]); // must be normalized for CartToJntSearch

  //call IK solver
  if(pr2_arm_ik_solver_->CartToJntSearch(ik_jnt_pos_in_, f, ik_jnt_pos_out_, 1.0) < 0)
    return false;

  solution.resize(start.size());
  for(size_t i = 0; i < solution.size(); ++i)
    solution[i] = ik_jnt_pos_out_(i);

  return true;
}

bool SBPLArmModel::computeFastIK(const std::vector<double> pose, const std::vector<double> start, std::vector<double> &solution)
{
  //pose: {x,y,z,r,p,y}  or {x,y,z,qx,qy,qz,qw}
  KDL::Frame frame_des;

  //fill the goal frame
  frame_des.p.x(pose[0]);
  frame_des.p.y(pose[1]);
  frame_des.p.z(pose[2]);
  
  //RPY
  if(pose.size() == 6)
    frame_des.M = KDL::Rotation::RPY(pose[3],pose[4],pose[5]);
  //Quaternion
  else
    frame_des.M = KDL::Rotation::Quaternion(pose[3],pose[4],pose[5],pose[6]);

  //transform from reference_frame_ to root_link of the IK tree
  //frame_des.p = transform_inverse_ * frame_des.p;

  //fill the start configuration (to be used as a seed)
  for(unsigned int i = 0; i < start.size(); i++)
    ik_jnt_pos_in_(i) = angles::normalize_angle(start[i]); // must be normalized for CartToJnt

  ROS_DEBUG_NAMED(debug_log_, "[ik]  xyz: %0.2f %0.2f %0.2f  rpy: %0.2f %0.2f %0.2f", pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]);
  ROS_DEBUG_NAMED(debug_log_, "[ik] seed: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f",ik_jnt_pos_in_(0),ik_jnt_pos_in_(1),ik_jnt_pos_in_(2),ik_jnt_pos_in_(3),ik_jnt_pos_in_(4),ik_jnt_pos_in_(5),ik_jnt_pos_in_(6));

  //call IK solver
  if(pr2_arm_ik_solver_->CartToJnt(ik_jnt_pos_in_, frame_des, ik_jnt_pos_out_) < 0)
  {
    ROS_DEBUG_NAMED(debug_log_, "[ik] Failed with error code: %d", pr2_arm_ik_solver_->CartToJnt(ik_jnt_pos_in_, frame_des, ik_jnt_pos_out_));
    return false;
  }

  solution.resize(start.size());
  for(size_t i = 0; i < solution.size(); ++i)
    solution[i] = ik_jnt_pos_out_(i);

  ROS_DEBUG_NAMED(debug_log_, "[ik] sol.: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f",ik_jnt_pos_out_(0),ik_jnt_pos_out_(1),ik_jnt_pos_out_(2),ik_jnt_pos_out_(3),ik_jnt_pos_out_(4),ik_jnt_pos_out_(5),ik_jnt_pos_out_(6));
  
  return true;
}

bool SBPLArmModel::computeFastIK(const KDL::Frame &frame, const std::vector<double> start, std::vector<double> &solution)
{
  KDL::Frame f = frame;

  //transform from reference_frame_ to root_link of the IK tree
  //f.p = transform_inverse_ * frame.p;

  //fill the start configuration (to be used as a seed)
  for(unsigned int i = 0; i < start.size(); i++)
    ik_jnt_pos_in_(i) = angles::normalize_angle(start[i]); // must be normalized for CartToJnt

  ROS_DEBUG_NAMED(debug_log_, "[ik] seed: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f",ik_jnt_pos_in_(0),ik_jnt_pos_in_(1),ik_jnt_pos_in_(2),ik_jnt_pos_in_(3),ik_jnt_pos_in_(4),ik_jnt_pos_in_(5),ik_jnt_pos_in_(6));

  //call IK solver
  if(pr2_arm_ik_solver_->CartToJnt(ik_jnt_pos_in_, f, ik_jnt_pos_out_) < 0)
  {
    ROS_DEBUG_NAMED(debug_log_, "[ik] Failed with error code: %d", pr2_arm_ik_solver_->CartToJnt(ik_jnt_pos_in_, f, ik_jnt_pos_out_));
    return false;
  }

  solution.resize(start.size());
  for(size_t i = 0; i < solution.size(); ++i)
    solution[i] = ik_jnt_pos_out_(i);

  ROS_DEBUG_NAMED(debug_log_, "[ik] sol.: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f",ik_jnt_pos_out_(0),ik_jnt_pos_out_(1),ik_jnt_pos_out_(2),ik_jnt_pos_out_(3),ik_jnt_pos_out_(4),ik_jnt_pos_out_(5),ik_jnt_pos_out_(6));
  
  return true;
}

double SBPLArmModel::getAxisAngle(const double R1[3][3], const double R2[3][3])
{
  short unsigned int x,y,k;
  double sum, R3[3][3], R1_T[3][3];

  //R1_T = transpose R1
  for(x=0; x<3; x++)
    for(y=0; y<3; y++)
      R1_T[x][y] = R1[y][x];

  //R3= R1_T * R2
  for (x=0; x<3; x++)
  {
    for (y=0; y<3; y++)
    {
      sum = 0.0;
      for (k=0; k<3; k++)
        sum += (R1_T[x][k] * R2[k][y]);

      R3[x][y] = sum;
    }
  }

  return acos(((R3[0][0] + R3[1][1] + R3[2][2]) - 1.0) / 2.0);
}

bool SBPLArmModel::getJointPositions(const std::vector<double> angles, const BodyPose pose, std::vector<std::vector<double> > &links, KDL::Frame &f_out)
{
  KDL::Frame f;

  links.resize(joint_indeces_.size());

  for(int i = 0; i < int(joint_indeces_.size()); i++)
  {
    if(!computeFK(angles, pose, joint_indeces_[i], &f))
      return false;
    links[i].resize(3,0);
    links[i][0] = f.p.x();
    links[i][1] = f.p.y();
    links[i][2] = f.p.z();

    // TODO: changed from planning_joint_ to 2  (6/15/11)
    // TODO: changed from 2 to 3 (8/12/11) when I added in the top turret
    // TODO: changed it back temporarilu 8/23
    if(i == 2)
      f_out = f;
  }

  return true;
}

bool SBPLArmModel::getJointPositions(const std::vector<double> angles, const BodyPose pose, const double R_target[3][3], std::vector<std::vector<double> > &links, double *axis_angle)
{
  KDL::Frame f_link_tip;

  links.resize(joint_indeces_.size());

  for(unsigned int i = 0; i < joint_indeces_.size(); ++i)
  {
    //if(!computeFK(angles, links_[i].ind_chain, &f_link_tip))
    if(!computeFK(angles, pose, joint_indeces_[i], &f_link_tip))
      return false;

    links[i].resize(3,0);
    links[i][0] = f_link_tip.p.x();
    links[i][1] = f_link_tip.p.y();
    links[i][2] = f_link_tip.p.z();

    //if(links_[i].ind_chain == planning_joint_)
    if(joint_indeces_[i] == planning_joint_)
      *axis_angle = frameToAxisAngle(f_link_tip, R_target);
  }
  return true;
}

//returns end effector position in meters
bool SBPLArmModel::computePlanningJointPos(const std::vector<double> angles, const BodyPose pose, double* x, double* y, double* z)
{
  KDL::Frame planning_joint_frame;
  if(!computeFK(angles, pose, planning_joint_, &planning_joint_frame))
  {
    *x = planning_joint_frame.p.x();
    *y = planning_joint_frame.p.y();
    *z = planning_joint_frame.p.z();
    return true;
  }

  return false;
}

//returns end effector position in meters
bool SBPLArmModel::getPlanningJointPose(const std::vector<double> angles, const BodyPose body_pose, double R_target[3][3], std::vector<double> &pose, double *axis_angle)
{
  KDL::Frame planning_joint_frame;
  pose.resize(6);
  
  if(computeFK(angles, body_pose, planning_joint_, &planning_joint_frame))
  {
    pose[0] = planning_joint_frame.p.x();
    pose[1] = planning_joint_frame.p.y();
    pose[2] = planning_joint_frame.p.z();
    
    planning_joint_frame.M.GetRPY(pose[3],pose[4],pose[5]);

    *axis_angle = frameToAxisAngle(planning_joint_frame, R_target);

    return true;
  }

  return false;
}

bool SBPLArmModel::getPlanningJointPose(const std::vector<double> angles, const BodyPose body_pose, std::vector<double> &pose)
{
  KDL::Frame planning_joint_frame;
  if(pose.size() < 6)
    pose.resize(6);
 
  if(computeFK(angles, body_pose, planning_joint_, &planning_joint_frame))
  {
    pose[0] = planning_joint_frame.p.x();
    pose[1] = planning_joint_frame.p.y();
    pose[2] = planning_joint_frame.p.z();
  
    planning_joint_frame.M.GetRPY(pose[3],pose[4],pose[5]);
    //ROS_INFO("[arm] xyz: %0.12f %0.12f %0.12f rpy: %0.12f %0.12f %0.12f", planning_joint_frame.p.x(),planning_joint_frame.p.y() , planning_joint_frame.p.z(), pose[3],pose[4],pose[5]);
    return true;
  }

  return false;
}

void SBPLArmModel::RPY2Rot(double roll, double pitch, double yaw, double Rot[3][3])
{
  double cosr, cosp, cosy, sinr, sinp, siny;

  cosr = cos(roll);
  cosp = cos(pitch);
  cosy = cos(yaw);
  sinr = sin(roll);
  sinp = sin(pitch);
  siny = sin(yaw);

  Rot[0][0] = cosp*cosy;
  Rot[0][1] = cosy*sinp*sinr - siny*cosr;
  Rot[0][2] = cosy*sinp*cosr + siny*sinr;   

  Rot[1][0] = siny*cosp;
  Rot[1][1] = siny*sinp*sinr + cosy*cosr;
  Rot[1][2] = siny*sinp*cosr - cosy*sinr;

  Rot[2][0] = -sinp;
  Rot[2][1] = cosp*sinr;
  Rot[2][2] = cosp*cosr;
}

/** convert a rotation matrix into the RPY format (copied from getEulerZYX() in Bullet physics library) */
void SBPLArmModel::getRPY(double Rot[3][3], double* roll, double* pitch, double* yaw, int solution_number)
{
  double delta,rpy1[3],rpy2[3];

  // Check that pitch is not at a singularity
  if(fabs(Rot[0][2]) >= 1)
  {
    rpy1[2]  = 0;
    rpy2[2]  = 0;

    // From difference of angles formula
    delta = atan2(Rot[0][0], Rot[2][0]);
    if(Rot[0][2] > 0)   //gimbal locked up
    {
      rpy1[1] = M_PI / 2.0;
      rpy2[1] = M_PI / 2.0;
      rpy1[0] = rpy1[1] + delta;
      rpy2[0] = rpy2[1] + delta;
    }
    else // gimbal locked down
    {
      rpy1[1] = -M_PI / 2.0;
      rpy2[1] = -M_PI / 2.0;
      rpy1[0] = -rpy1[1] + delta;
      rpy2[0] = -rpy2[1] + delta;
    }
  }
  else
  {
    rpy1[1] = -asin(Rot[0][2]);
    rpy2[1] = M_PI - rpy1[1];


    rpy1[0] = atan2(Rot[1][2]/cos(rpy1[1]),
        Rot[2][2]/cos(rpy1[1]));

    rpy2[0] = atan2(Rot[1][2]/cos(rpy2[1]),
        Rot[2][2]/cos(rpy2[1]));

    rpy1[2] = atan2(Rot[0][1]/cos(rpy1[1]),
        Rot[0][0]/cos(rpy1[1]));

    rpy2[2] = atan2(Rot[0][1]/cos(rpy2[1]),
        Rot[0][0]/cos(rpy2[1]));
  }

  if (solution_number == 1)
  {
    *yaw = rpy1[2];
    *pitch = rpy1[1];
    *roll = rpy1[0];
  }
  else
  {
    *yaw = rpy2[2];
    *pitch = rpy2[1];
    *roll = rpy2[0];
  }
}

bool SBPLArmModel::checkJointLimits(std::vector<double> angles, bool verbose)
{
  std::vector<double> norm_angles(angles.size());
  for(int i = 0; i < int(angles.size()); ++i)
  {
    if(i==2)
      continue;
    norm_angles[i] = angles::normalize_angle(angles[i]);
  }

  //upper arm roll is a special case
  if(angles[2] > joints_[2].max)
    norm_angles[2] = angles[2] + (2*-M_PI);

  if(int(norm_angles.size()) < num_joints_)
    SBPL_DEBUG_NAMED(debug_log_,"  [%s] Joint array has %d joints. (should be %d joints)", name_.c_str(),int(norm_angles.size()),num_joints_);

  for(int i = 0; i < num_joints_; ++i)
  {
    if(!joints_[i].continuous)
    {
      if(joints_[i].min > norm_angles[i] || norm_angles[i] > joints_[i].max)
      {
        SBPL_DEBUG_NAMED(debug_log_, "  [%s] Joint %d is invalid with value %0.3f (limits are {%0.3f, %0.3f}.", name_.c_str(),i,norm_angles[i],joints_[i].min,joints_[i].max);
        return false;
      }
    }
  }
  return true;
}

bool SBPLArmModel::checkJointLimits(std::vector<double> angles, int &debug_code)
{
  std::vector<double> norm_angles(angles.size());
  debug_code = sbpl_arm_planner::SUCCESS;

  for(int i = 0; i < int(angles.size()); ++i)
  {
    if(i==2)
      continue;
    norm_angles[i] = angles::normalize_angle(angles[i]);
  }

  //upper arm roll is a special case
  if(angles[2] > joints_[2].max)
    norm_angles[2] = angles[2] + (2*-M_PI);

  if(int(norm_angles.size()) < num_joints_)
    SBPL_DEBUG_NAMED(debug_log_,"  [%s] Joint array has %d joints. (should be %d joints)", name_.c_str(), int(norm_angles.size()),num_joints_);

  for(int i = 0; i < num_joints_; ++i)
  {
    if(!joints_[i].continuous)
    {
      if(joints_[i].min > norm_angles[i] || norm_angles[i] > joints_[i].max)
      {
        SBPL_DEBUG_NAMED(debug_log_, "  [%s] Joint %d is invalid with value %0.3f (limits are {%0.3f, %0.3f}.",name_.c_str(),i,norm_angles[i],joints_[i].min,joints_[i].max);
        debug_code = debug_code_types_[i];
        return false;
      }
    }
  }
  return true;
}

void SBPLArmModel::printArmDescription(std::string stream)
{
  ROS_INFO_NAMED(stream,"SBPL Arm Description:");
  ROS_INFO_NAMED(stream,"# Joints: %d,   # Links: %d",num_joints_,num_links_);
  ROS_INFO_NAMED(stream,"Root Frame: %s, Tip Frame: %s", chain_root_name_.c_str(),chain_tip_name_.c_str());
  ROS_INFO_NAMED(stream,"Joint Limits:");
  for(int i = 0; i < num_joints_; ++i)
  {
    if(joints_[i].continuous)
      ROS_INFO_NAMED(stream,"%d:    continuous",i);
    else
      ROS_INFO_NAMED(stream,"%d: {%.3f, %0.3f}",i,joints_[i].min,joints_[i].max);
  }

  ROS_INFO_NAMED(stream,"Links:");
  for(int i = 0; i < num_links_; ++i)
    ROS_INFO_NAMED(stream,"(%d) radius: %0.3fm (%d cells)  length: %0.3fm   KDL chain index: %d",i,links_[i].radius,links_[i].radius_c, links_[i].length,joint_indeces_[i]);

  ROS_INFO_NAMED(stream,"\nKDL Arm Chain:");
  ROS_INFO_NAMED(stream,"# segments: %d, # joints: %d", chain_.getNrOfSegments(), chain_.getNrOfJoints());

  double jnt_pos = 0;
  for(size_t j = 0; j < chain_.getNrOfSegments(); ++j)
  {
    ROS_INFO_NAMED(stream,"frame %2d: segment: %0.3f %0.3f %0.3f  joint: %0.3f %0.3f %0.3f   joint_type: %s",int(j),
        chain_.getSegment(j).pose(jnt_pos).p.x(),
        chain_.getSegment(j).pose(jnt_pos).p.y(),
        chain_.getSegment(j).pose(jnt_pos).p.z(),
        chain_.getSegment(j).getJoint().pose(jnt_pos).p.x(),
        chain_.getSegment(j).getJoint().pose(jnt_pos).p.y(),
        chain_.getSegment(j).getJoint().pose(jnt_pos).p.z(),
        chain_.getSegment(j).getJoint().getTypeName().c_str());
  }
}

void SBPLArmModel::getJointLimits(int joint_num, double* min, double *max)
{
  *min = joints_[joint_num].min;
  *max = joints_[joint_num].max;
}

void SBPLArmModel::setRefFrameTransform(KDL::Frame f, std::string &name)
{
  reference_frame_ = name;

  transform_ = f;
  transform_inverse_ = transform_.Inverse();

  ROS_INFO("Transform  %s to torso: x: %0.4f y: %0.4f z: %0.4f", name.c_str(),transform_.p.x(),transform_.p.y(),transform_.p.z()); 
  ROS_INFO("Inverse Transform torso to %s: x: %0.4f y: %0.4f z: %0.4f", name.c_str(),transform_inverse_.p.x(),transform_inverse_.p.y(),transform_inverse_.p.z()); 
}

void SBPLArmModel::getArmChainRootLinkName(std::string &name)
{
  name = chain_root_name_;
}

bool SBPLArmModel::computeTranslationalIK(const std::vector<double> pose, double upperarm_roll, std::vector<double> &solution)
{
  std::vector<std::vector<double> > sol;
  KDL::Frame frame_des;

  //fill the goal frame
  frame_des.p.x(pose[0]);
  frame_des.p.y(pose[1]);
  frame_des.p.z(pose[2]);
  
  //RPY
  if(pose.size() == 6)
    frame_des.M = KDL::Rotation::RPY(pose[3],pose[4],pose[5]);
  //Quaternion
  else
    frame_des.M = KDL::Rotation::Quaternion(pose[3],pose[4],pose[5],pose[6]);

  //transform from reference_frame_ to root_link_
  frame_des.p = transform_inverse_ * frame_des.p;

  Eigen::Matrix4f g = pr2_arm_kinematics::KDLToEigenMatrix(frame_des);

ROS_ERROR("We need the modified pr2 arm kinematics!");
exit(0);
  //pr2_arm_ik_.computeTranslationIKShoulderRoll(g, upperarm_roll, sol);

/*
  printf("[computeTranslationalIK] Solution for xyz: %0.3f %0.3f %0.3f  upperarm_roll: %0.3f has length %d\n", g(0,3),g(1,3),g(2,3),upperarm_roll, int(sol.size()));
  for(size_t i = 0; i < sol.size(); ++i)
    printf("[%d] %0.3f %0.3f %0.3f %0.3f\n", int(i), sol[i][0],sol[i][1],sol[i][2],sol[i][3]);
*/

  if(sol.size() == 0)
    return false;

  return true;
}

/*
int SBPLArmModel::getJointLimitDebugCode(const int &joint)
{
  if(arm_id_ == 1)
  {
    switch(joint)
    {
      case 0:
        return sbpl_arm_planner::DebugCode::INVALID_LEFT_SHOULDER_PAN_ANGLE;
      case 1:
        return sbpl_arm_planner::DebugCode::INVALID_LEFT_SHOULDER_PITCH_ANGLE;
      case 2:
        return sbpl_arm_planner::DebugCode::INVALID_LEFT_UPPER_ARM_ROLL_ANGLE;
      case 3:
        return sbpl_arm_planner::DebugCode::INVALID_LEFT_ELBOW_FLEX_ANGLE;
      case 4:
        return sbpl_arm_planner::DebugCode::INVALID_LEFT_FOREARM_ROLL_ANGLE;
      case 5:
        return sbpl_arm_planner::DebugCode::INVALID_LEFT_WRIST_PITCH_ANGLE;
      case 6:
        return sbpl_arm_planner::DebugCode::INVALID_LEFT_WRIST_ROLL_ANGLE;
    }
  }
  else
  {
    switch(joint)
    {
      case 0:
        return sbpl_arm_planner::DebugCode::INVALID_RIGHT_SHOULDER_PAN_ANGLE;
      case 1:
        return sbpl_arm_planner::DebugCode::INVALID_RIGHT_SHOULDER_PITCH_ANGLE;
      case 2:
        return sbpl_arm_planner::DebugCode::INVALID_RIGHT_UPPER_ARM_ROLL_ANGLE;
      case 3:
        return sbpl_arm_planner::DebugCode::INVALID_RIGHT_ELBOW_FLEX_ANGLE;
      case 4:
        return sbpl_arm_planner::DebugCode::INVALID_RIGHT_FOREARM_ROLL_ANGLE;
      case 5:
        return sbpl_arm_planner::DebugCode::INVALID_RIGHT_WRIST_PITCH_ANGLE;
      case 6:
        return sbpl_arm_planner::DebugCode::INVALID_RIGHT_WRIST_ROLL_ANGLE;
    }
  }
}
*/

int SBPLArmModel::getSegmentIndex(std::string &name)
{
  for(size_t k = 0; k < chain_.getNrOfSegments(); ++k)
  {
    if(chain_.getSegment(k).getName().compare(name) == 0)
      return k;
  }
  ROS_DEBUG("[arm] Failed to find %s segment in the chain.", name.c_str());
  return -1;
}

}
