/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <pr2_collision_checker/pr2_collision_space_interface.h>
#include <pviz/pviz.h>

PViz *pviz;

arm_navigation_msgs::RobotState fillRobotState(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body)
{
  arm_navigation_msgs::RobotState state; 
  std::vector<std::string> ljoint_names_(7);
  std::vector<std::string> rjoint_names_(7);
  ljoint_names_[0] = "l_shoulder_pan_joint";
  ljoint_names_[1] = "l_shoulder_lift_joint";
  ljoint_names_[2] = "l_upper_arm_roll_joint";
  ljoint_names_[3] = "l_elbow_flex_joint";
  ljoint_names_[4] = "l_forearm_roll_joint";
  ljoint_names_[5] = "l_wrist_flex_joint";
  ljoint_names_[6] = "l_wrist_roll_joint";
  rjoint_names_[0] = "r_shoulder_pan_joint";
  rjoint_names_[1] = "r_shoulder_lift_joint";
  rjoint_names_[2] = "r_upper_arm_roll_joint";
  rjoint_names_[3] = "r_elbow_flex_joint";
  rjoint_names_[4] = "r_forearm_roll_joint";
  rjoint_names_[5] = "r_wrist_flex_joint";
  rjoint_names_[6] = "r_wrist_roll_joint";

  for(size_t i = 0; i < 7; ++i)
  {
    state.joint_state.name.push_back(rjoint_names_[i]);
    state.joint_state.position.push_back(rangles[i]);
    state.joint_state.name.push_back(ljoint_names_[i]);
    state.joint_state.position.push_back(langles[i]);
  }
  state.joint_state.name.push_back("torso_lift_joint");
  state.joint_state.position.push_back(body.z);

  state.multi_dof_joint_state.frame_ids.push_back("map");
  state.multi_dof_joint_state.child_frame_ids.push_back("base_footprint");
  state.multi_dof_joint_state.poses.resize(1);
  state.multi_dof_joint_state.poses[0].position.x = body.x;
  state.multi_dof_joint_state.poses[0].position.y = body.y;
  state.multi_dof_joint_state.poses[0].position.z = 0;
  leatherman::rpyToQuatMsg(0, 0, body.theta, state.multi_dof_joint_state.poses[0].orientation);

  return state;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pr2_collision_space");
  ros::NodeHandle nh("~");
  ROSCONSOLE_AUTOINIT;
  sleep(1);
  FILE* rarm_fp=NULL;
  FILE* larm_fp=NULL;
  std::string reference_frame, larm_filename, rarm_filename;
  double originX, originY, originZ, sizeX, sizeY, sizeZ, resolution; 
  sbpl_arm_planner::SBPLArmModel *larm, *rarm;
  sbpl_arm_planner::OccupancyGrid *grid;
  pr2_collision_checker::PR2CollisionSpace *cspace;
  pr2_collision_checker::PR2CollisionSpaceInterface *cspace_mon;
  nh.param<std::string>("planner/left_arm_description_file", larm_filename, "");
  nh.param<std::string>("planner/right_arm_description_file", rarm_filename, "");
  nh.param("collision_space/resolution",resolution,0.02);
  nh.param<std::string>("collision_space/reference_frame",reference_frame,"map");
  nh.param("collision_space/occupancy_grid/origin_x",originX,-0.6);
  nh.param("collision_space/occupancy_grid/origin_y",originY,-1.15);
  nh.param("collision_space/occupancy_grid/origin_z",originZ,-0.05);
  nh.param("collision_space/occupancy_grid/size_x",sizeX,1.6);
  nh.param("collision_space/occupancy_grid/size_y",sizeY,1.8);
  nh.param("collision_space/occupancy_grid/size_z",sizeZ,1.4);

  pviz = new PViz();
  pviz->setReferenceFrame(reference_frame);

  std::vector<std::string> ljoint_names_(7);
  std::vector<std::string> rjoint_names_(7);
  ljoint_names_[0] = "l_shoulder_pan_joint";
  ljoint_names_[1] = "l_shoulder_lift_joint";
  ljoint_names_[2] = "l_upper_arm_roll_joint";
  ljoint_names_[3] = "l_elbow_flex_joint";
  ljoint_names_[4] = "l_forearm_roll_joint";
  ljoint_names_[5] = "l_wrist_flex_joint";
  ljoint_names_[6] = "l_wrist_roll_joint";
  rjoint_names_[0] = "r_shoulder_pan_joint";
  rjoint_names_[1] = "r_shoulder_lift_joint";
  rjoint_names_[2] = "r_upper_arm_roll_joint";
  rjoint_names_[3] = "r_elbow_flex_joint";
  rjoint_names_[4] = "r_forearm_roll_joint";
  rjoint_names_[5] = "r_wrist_flex_joint";
  rjoint_names_[6] = "r_wrist_roll_joint";

  // create the left & right arm models
  if((rarm_fp=fopen(rarm_filename.c_str(),"r")) == NULL)
  {
    ROS_ERROR("[pr2cc] Failed to open right arm description file.");
    return -1;
  }
  if((larm_fp=fopen(larm_filename.c_str(),"r")) == NULL)
  {
    ROS_ERROR("[pr2cc] Failed to open left arm description file.");
    return -1;
  }
  ROS_INFO("Initializing Arms");
  rarm = new sbpl_arm_planner::SBPLArmModel(rarm_fp);
  larm = new sbpl_arm_planner::SBPLArmModel(larm_fp);
  rarm->setResolution(resolution);
  larm->setResolution(resolution);
  rarm->setDebugLogName("rarm");
  larm->setDebugLogName("larm");
  ROS_INFO("Initialized Arms");

  if(!rarm->initKDLChainFromParamServer() || !larm->initKDLChainFromParamServer())
  {
    ROS_ERROR("[pr2cc] Failed to initialize arm models.");
    return -1;
  }
  fclose(rarm_fp);
  fclose(larm_fp);

  // create the occupancy grid
  grid = new sbpl_arm_planner::OccupancyGrid(sizeX, sizeY, sizeZ, resolution, originX, originY, originZ);
  grid->setReferenceFrame(reference_frame);

  // create the collision space
  cspace = new pr2_collision_checker::PR2CollisionSpace(rarm, larm, grid);
  if(!cspace->init())
  {
    ROS_ERROR("[pr2cc] Failed to get the full body spheres from the param server.");
    return -1;
  }
  else
    cspace->printSphereGroups();

  // create the collision space monitor
  cspace_mon = new pr2_collision_checker::PR2CollisionSpaceInterface(grid, cspace);

  /*
  ROS_INFO("[pr2cc] Spinning...");
  ros::spin();
  */

  int debug_code=200;
  double dist=100.0;
  visualization_msgs::MarkerArray ma; 
  std::vector<double> rangles(7,0), langles(7,0);
  BodyPose body(0,0,0.3,0);
  arm_navigation_msgs::RobotState state;
  
  state = fillRobotState(rangles, langles, body);
  cspace_mon->setRobotState(state);
  if(!cspace->checkCollision(langles, rangles, body, true, dist, debug_code))
    ROS_ERROR("In collision! {dist: %0.3fm   debug_code: %d}", dist, debug_code);
  else 
    ROS_INFO("NO collision. {dist: %0.3fm   debug_code: %d}", dist, debug_code);

  ma = cspace_mon->getVisualization("collision_model");
  pviz->publishMarkerArray(ma);
  ma = cspace_mon->getVisualization("bounds");
  pviz->publishMarkerArray(ma);

  sleep(1.0);
  pviz->deleteVisualizations("collision_model", 400);

  debug_code=200; dist=100.0;
  rangles[0] = -1.5; rangles[3] = -1.0;
  langles[0] =  1.5; langles[3] = -1.0;
  body.x = 1.0; body.z = 0.0; body.theta = 1.0; 
  state = fillRobotState(rangles, langles, body);
  cspace_mon->setRobotState(state);
  if(!cspace->checkCollision(langles, rangles, body, true, dist, debug_code))
    ROS_ERROR("In collision! {dist: %0.3fm   debug_code: %d}", dist, debug_code);
  else 
    ROS_INFO("NO collision. {dist: %0.3fm   debug_code: %d}", dist, debug_code);

  ma = cspace_mon->getVisualization("collision_model");
  pviz->publishMarkerArray(ma);
  ma = cspace_mon->getVisualization("bounds");
  pviz->publishMarkerArray(ma);

  sleep(1.0);
  pviz->deleteVisualizations("collision_model", 400);

  debug_code=200; dist=100.0;
  rangles[0] = -1.5; rangles[3] = -1.0;
  langles[0] =  1.5; langles[3] = -1.0;
  body.x = 1.3; body.z = 0.0; body.theta = -1.0; 
  state = fillRobotState(rangles, langles, body);
  cspace_mon->setRobotState(state);
  if(!cspace->checkCollision(langles, rangles, body, true, dist, debug_code))
    ROS_ERROR("In collision! {dist: %0.3fm   debug_code: %d}", dist, debug_code);
  else 
    ROS_INFO("NO collision. {dist: %0.3fm   debug_code: %d}", dist, debug_code);

  ma = cspace_mon->getVisualization("collision_model");
  pviz->publishMarkerArray(ma);
  ma = cspace_mon->getVisualization("bounds");
  pviz->publishMarkerArray(ma);

  ROS_INFO("done");
  ros::spinOnce();
  return 0;
}


