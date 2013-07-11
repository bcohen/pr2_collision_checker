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
#include <ctime>
#include <leatherman/viz.h>

PViz *pviz;
pr2_collision_checker::PR2CollisionSpace *cspace;

void printRobotState(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text)
{
  ROS_INFO("robot state:  %s", text.c_str());
  ROS_INFO("     x: %0.3f  y: % 0.3f  z: %0.3f yaw: % 0.3f", body_pos.x, body_pos.y, body_pos.z, body_pos.theta);
  ROS_INFO(" right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", rangles[0], rangles[1], rangles[2], rangles[3], rangles[4], rangles[5], rangles[6]);
  ROS_INFO("  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", langles[0], langles[1], langles[2], langles[3], langles[4], langles[5], langles[6]);
}

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

bool addMesh(std::string filename, std::string name, geometry_msgs::Pose pose)
{
  std::vector<int32_t> triangles;
  std::vector<geometry_msgs::Point> vertices;
  if(!leatherman::getMeshComponentsFromResource(filename, triangles, vertices))
  {
    ROS_ERROR("Failed to get mesh.");
    return false;
  }

  cspace->addCollisionObjectMesh(vertices, triangles, pose, name);
  return true;
}

/*
Group getGroup(geometry_msgs::Pose &pose, std::string name, std::string filename)
{
  Group g;
  g.name = name;
  std::vector<int32_t> triangles;
  std::vector<geometry_msgs::Point> vertices;
  if(!leatherman::getMeshComponentsFromResource(filename, triangles, vertices))
  {
    ROS_ERROR("Failed to get mesh for group.");
    return g;
  }
        std::vector<Sphere> spheres;
          int kdl_chain;
            int kdl_segment;
              KDL::Frame f; // temp variable
}
*/

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
  //sbpl_arm_planner::OccupancyGrid *grid;
  //pr2_collision_checker::PR2CollisionSpace *cspace;
  //pr2_collision_checker::PR2CollisionSpaceInterface *cspace_mon;
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

  std::vector<double> dims(3,0), origin(3,0);
  dims[0] = sizeX; dims[1] = sizeY; dims[2] = sizeZ;
  origin[0] = originX; origin[1] = originY; origin[2] = originZ;
  cspace = new pr2_collision_checker::PR2CollisionSpace(rarm_filename, larm_filename, dims, origin, resolution, reference_frame);

  if(!cspace->init())
  {
    ROS_ERROR("[pr2cc] Failed to get the full body spheres from the param server.");
    return -1;
  }
  else
    cspace->printSphereGroups();

  
  // create the collision space monitor
  //cspace_mon = new pr2_collision_checker::PR2CollisionSpaceInterface(grid, cspace);

  int debug_code=200;
  double dist=100.0;
  visualization_msgs::MarkerArray ma; 
  std::vector<double> rangles(7,0), langles(7,0);
  BodyPose body(0,0,0,0);
  arm_navigation_msgs::RobotState state;
  srand(time(NULL));
  visualization_msgs::Marker m, m1;
  geometry_msgs::Pose p, p1;
  p.position.x = 3; p.position.y = -3; p.position.z = 3.0; p.orientation.w = 1;
  p1 = p;
  p1.position.x += 0.0; p1.position.y += 0.6; p1.position.z -= 0.5;

  geometry_msgs::Pose op;
  op.position.x = 0; op.position.y = 0; op.position.z = 1.5; op.orientation.w = 1;
  //if(!addMesh("/opt/ros/groovy/stacks/pr2_common/pr2_description/meshes/torso_v0/torso_lift.stl", "object", op))
  if(!addMesh("package://pr2_description/meshes/torso_v0/torso_lift.stl", "object", op))
  {
    ROS_ERROR("Failed to add object mesh to scene.");
    return 1;
  }

  for(size_t i = 0; i < 100; ++i)
  {
    ROS_INFO("-------------- %d ----------------", int(i));
      
    dist=100.0;

    // rand joint configurations for arms
    for(size_t j = 0; j < 7; ++j)
    {
      double rr = (double)rand()/(double)RAND_MAX;
      double rl = (double)rand()/(double)RAND_MAX;
      double dr = angles::shortest_angular_distance(rarm->getMinJointLimit(j), rarm->getMaxJointLimit(j));
      double dl = angles::shortest_angular_distance(larm->getMinJointLimit(j), larm->getMaxJointLimit(j));

      rangles[j] = rr*dr + rarm->getMinJointLimit(j);
      langles[j] = rl*dl + larm->getMinJointLimit(j);
    }

    // random location for base
    double rx = (double)rand()/(double)RAND_MAX;
    double ry = (double)rand()/(double)RAND_MAX;
    double rz = (double)rand()/(double)RAND_MAX;
    double rtheta = (double)rand()/(double)RAND_MAX;
  
    body.x = rx*sizeX + originX;
    body.y = ry*sizeY + originY; 
    body.z = rz*(0.3); 
    body.theta = rtheta*2*M_PI;

    //cspace_mon->printRobotState(rangles, langles, body, "pose");
    printRobotState(rangles, langles, body, "pose");
    ROS_INFO(" ");
    if(!cspace->checkRobotAgainstWorld(langles, rangles, body, true, dist))
    {
      ROS_ERROR("I'm in collision! {dist: %0.3fm}", dist);
      m = viz::getTextMarker(p, "world: ouchy!", 0.4, 10, "map", "robot-world_result", 0);
    }
    else 
    {
      ROS_INFO("Yay! I'm collision free! {dist: %0.3fm}", dist);
      m = viz::getTextMarker(p, "world:   yay!", 0.4, 100, "map", "robot-world_result", 0);
    }
 
    if(!cspace->checkRobotAgainstRobot(langles, rangles, body, true, dist))
    {
      ROS_ERROR("I'm in self-collision! {dist: %0.3fm}", dist);
      m1 = viz::getTextMarker(p1, "    me: ouchy!", 0.4, 10, "map", "robot-robot_result", 0);
    }
    else 
    {
      ROS_INFO("Yay! I'm self-collision free! {dist: %0.3fm}", dist);
      m1 = viz::getTextMarker(p1, "    me:   yay!", 0.4, 100, "map", "robot-robot_result", 0);
    }
   
    ma = cspace->getCollisionModelVisualization(rangles, langles, body, "collision_model", 0);
    ma.markers.push_back(m);
    ma.markers.push_back(m1);
    pviz->publishMarkerArray(ma);
    ma = cspace->getVisualization("bounds", "bounds", 0);
    pviz->publishMarkerArray(ma);
    ma = cspace->getVisualization("distance_field", "distance_field", 0);
    pviz->publishMarkerArray(ma);
    ma = cspace->getVisualization("collision_objects", "collision_objects", 0);
    pviz->publishMarkerArray(ma);
    sleep(1.0);
    pviz->deleteVisualizations("collision_model", 400);
  }
  
  ROS_INFO("done");
  ros::spinOnce();
  return 0;
}


