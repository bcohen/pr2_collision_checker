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

namespace pr2_collision_checker
{

PR2CollisionSpaceInterface::PR2CollisionSpaceInterface(sbpl_arm_planner::OccupancyGrid *grid, pr2_collision_checker::PR2CollisionSpace *cspace)
{
  cspace_ = cspace;
  grid_ = grid;
  attached_object_ = false;
  world_frame_ = "map";

	langles_.resize(7, 0);
	rangles_.resize(7, 0);
	ljoint_names_.resize(7);
	rjoint_names_.resize(7);

	// PR2 specific joint names
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
}

PR2CollisionSpaceInterface::~PR2CollisionSpaceInterface()
{
}

bool PR2CollisionSpaceInterface::init()
{
	return true;
}


void PR2CollisionSpaceInterface::setCollisionMap(const arm_navigation_msgs::CollisionMap &collision_map)
{
  if (collision_map.header.frame_id.compare(grid_->getReferenceFrame()) != 0 &&
      collision_map.header.frame_id.compare("/" + grid_->getReferenceFrame()) != 0) {
    ROS_WARN("[cspace] The collision map received is in %s frame but expected in %s frame.", collision_map.header.frame_id.c_str(), grid_->getReferenceFrame().c_str());
  }
  cspace_->storeCollisionMap(collision_map);
  world_frame_ = collision_map.header.frame_id;
  cspace_->putCollisionObjectsInGrid();
}

void PR2CollisionSpaceInterface::setCollisionObject(const arm_navigation_msgs::CollisionObject &collision_object)
{
  // for some reason, it wasn't getting all of the 'all' messages...
  if (collision_object.id.compare("all") == 0)
    cspace_->removeAllCollisionObjects();

  // debug: have we seen this collision object before?
  if (object_map_.find(collision_object.id) != object_map_.end())
    ROS_DEBUG("[cspace] We have seen this object ('%s')  before.", collision_object.id.c_str());
  else
    ROS_DEBUG("[cspace] We have NOT seen this object ('%s') before.", collision_object.id.c_str());

  object_map_[collision_object.id] = collision_object;
  cspace_->processCollisionObjectMsg(collision_object);
}

void PR2CollisionSpaceInterface::setJointStates(const sensor_msgs::JointState &state)
{
  // arms
	for(unsigned int i=0; i<rjoint_names_.size(); i++)
  {
		unsigned int j;
		for(j=0; j<state.name.size(); j++)
			if(rjoint_names_[i].compare(state.name[j])==0)
				break;
		if(j==state.name.size())
			ROS_WARN("[jointStatesCallback] Missing the value for planning joint (%s)\n",rjoint_names_[i].c_str());
		else
			rangles_[i] = state.position[j];
	}
	for(unsigned int i=0; i<ljoint_names_.size(); i++)
  {
		unsigned int j;
		for(j=0; j<state.name.size(); j++)
			if(ljoint_names_[i].compare(state.name[j])==0)
				break;
		if(j==state.name.size())
			ROS_WARN("[jointStatesCallback] Missing the value for planning joint (%s)\n",rjoint_names_[i].c_str());
		else
			langles_[i] = state.position[j];
	}

  // torso
	unsigned int j;
	for (j = 0; j < state.name.size(); j++)
		if (state.name[j].compare("torso_lift_joint") == 0) break;
	if(j==state.name.size())
		ROS_WARN("[jointStatesCallback] Missing the value for planning joint torso_lift_joint\n");
	else
		body_pos_.z = state.position[j];
}

void PR2CollisionSpaceInterface::setRobotState(const arm_navigation_msgs::RobotState &state)
{
  setJointStates(state.joint_state);
  
  geometry_msgs::Pose p;
  if(leatherman::getPose(state.multi_dof_joint_state, world_frame_, "base_footprint", p))
  {
    body_pos_.x = p.position.x;
    body_pos_.y = p.position.y;
    body_pos_.theta = 2 * atan2(p.orientation.z, p.orientation.w);
  }

  if(leatherman::getFrame(state.multi_dof_joint_state, world_frame_, grid_->getReferenceFrame(), T_collision_to_world_))
    setCollisionToWorldTransform(T_collision_to_world_, world_frame_);
  else
    ROS_ERROR("Failed to set the collision kinematics to world transform. {world: %s  collision kinematics: %s}", grid_->getReferenceFrame().c_str(), world_frame_.c_str());
  printRobotState(rangles_, langles_, body_pos_, "AFTER SETTING STATE");
}

void PR2CollisionSpaceInterface::setAttachedObject(const arm_navigation_msgs::AttachedCollisionObject &attached_object)
{
  // remove all objects
  if(attached_object.link_name.compare(arm_navigation_msgs::AttachedCollisionObject::REMOVE_ALL_ATTACHED_OBJECTS) == 0 &&
      attached_object.object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
  {
    ROS_DEBUG("[cspace] Removing all attached objects.");
    attached_object_ = false;
    cspace_->removeAllAttachedObjects();
  }
  // add object
  else if(attached_object.object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD)
  {
    ROS_INFO("[cspace] Received a message to ADD an object (%s) with %d shapes.", attached_object.object.id.c_str(), int(attached_object.object.shapes.size()));
    object_map_[attached_object.object.id] = attached_object.object;
    attachObject(attached_object.object, attached_object.link_name);
  }
  // attach object and remove it from collision space
  else if(attached_object.object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT)
  {
    ROS_INFO("[cspace] Received a message to ATTACH_AND_REMOVE_AS_OBJECT of object: %s", attached_object.object.id.c_str());

    // have we seen this collision object before?
    if(object_map_.find(attached_object.object.id) != object_map_.end())
    {
      ROS_INFO("[cspace] We have seen this object (%s) before.", attached_object.object.id.c_str());
      ROS_WARN("[cspace] Attached objects we have seen before are not handled correctly right now.");
      attachObject(object_map_.find(attached_object.object.id)->second, attached_object.link_name);
    }
    else
    {
      ROS_INFO("[cspace] We have NOT seen this object (%s) before (it's not in my internal object map).", attached_object.object.id.c_str());
      if(attached_object.object.shapes.empty())
      {
        ROS_WARN("[cspace] '%s' is not in my internal object map and the message doesn't contain any shapes. Can't attach it.", attached_object.object.id.c_str());
        return;
      }
      object_map_[attached_object.object.id] = attached_object.object;
      attachObject(attached_object.object, attached_object.link_name);
    }
    ROS_INFO("[3dnav] Just attached '%s', now I'll remove it from the world.", attached_object.object.id.c_str());
    cspace_->removeCollisionObject(attached_object.object);
  }
  // remove object
  else if(attached_object.object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
  {
    ROS_DEBUG("[cspace] Removing object (%s) from gripper.", attached_object.object.id.c_str());
    cspace_->removeAttachedObject(attached_object.object.id);
  }
  // detach and add as object
  else if(attached_object.object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT)
  {
    attached_object_ = false;
    ROS_DEBUG("[cspace] Removing object (%s) from gripper and adding to collision map.", attached_object.object.id.c_str());
    cspace_->removeAttachedObject(attached_object.object.id);
    attached_object_ = cspace_->isObjectAttached();
    
    //sometimes people are lazy and they don't fill in the object's
    //description. in those cases - we have to depend on our stored
    //description of the object to be added. if we can't find it in our
    //object map, we hope that they added in a description themselves.
    if(object_map_.find(attached_object.object.id) != object_map_.end())
    {
      ROS_INFO("[3dnav] Detached '%s' and now I'll add it as a known collision object. The message does not contain its description but I had it stored so I know what it is.", attached_object.object.id.c_str());
      cspace_->addCollisionObject(object_map_.find(attached_object.object.id)->second);
    }
    else
    {
      if(attached_object.object.shapes.empty())
      {
        ROS_WARN("[3dnav] '%s' was attached and now you want to add it as a collision object. Unfortunatly the message does not contain its decription and I don't have it in my internal database for some reason so I can't attach it. This is probably a bug.", attached_object.object.id.c_str());
        return;
      }
      object_map_[attached_object.object.id] = attached_object.object;
      cspace_->addCollisionObject(attached_object.object);
    }
  }
  else
    ROS_WARN("[3dnav] Received a collision object with an unknown operation");

  attached_object_ = cspace_->isObjectAttached();
}

void PR2CollisionSpaceInterface::attachObject(const arm_navigation_msgs::CollisionObject &obj, std::string link_name)
{
  geometry_msgs::PoseStamped pose_in, pose_out;
  arm_navigation_msgs::CollisionObject object(obj);
  attached_object_ = true;
  ROS_INFO("[cspace] Received a collision object message with id, '%s' and it contains %d shapes.", object.id.c_str(), int(object.shapes.size()));

  for(size_t i = 0; i < object.shapes.size(); i++)
  {
    // transform the object's pose into the link_name's pose
    // (we know that that link must be on the robot)
    pose_in.header = object.header;
    pose_in.header.stamp = ros::Time();
    pose_in.pose = object.poses[i];
    try
    {
      //tf_.transformPose(cspace_->getExpectedAttachedObjectFrame(link_name), pose_in, pose_out);
    }
    catch(int e)
    {
      ROS_ERROR("[cspace] Failed to transform the pose of the attached object from %s to %s. (exception: %d)", object.header.frame_id.c_str(), cspace_->getExpectedAttachedObjectFrame(object.header.frame_id).c_str(), e);
      ROS_ERROR("[cspace] Failed to attach '%s' object.", object.id.c_str());
      return;
    }
    object.poses[i] = pose_out.pose;
    ROS_WARN("[cspace] Converted attached object pose of '%s' shape from %s (%0.2f %0.2f %0.2f) to %s (%0.3f %0.3f %0.3f)", obj.id.c_str(), pose_in.header.frame_id.c_str(), pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z, pose_out.header.frame_id.c_str(), pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);

    if(object.shapes[i].type == arm_navigation_msgs::Shape::SPHERE)
    {
      ROS_INFO("[cspace] Attaching a '%s' sphere with radius: %0.3fm", object.id.c_str(), object.shapes[i].dimensions[0]);
      cspace_->attachSphere(object.id, link_name, object.poses[i], object.shapes[i].dimensions[0]);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::CYLINDER)
    {
      ROS_INFO("[cspace] Attaching a '%s' cylinder with radius: %0.3fm & length %0.3fm", object.id.c_str(), object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);
      cspace_->attachCylinder(object.id, link_name, object.poses[i], object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::MESH)
    {
      ROS_INFO("[3dnav] Attaching a '%s' mesh with %d triangles/3  & %d vertices.", object.id.c_str(), int(object.shapes[i].triangles.size()/3), int(object.shapes[i].vertices.size()));
      cspace_->attachMesh(object.id, link_name, object.poses[i], object.shapes[i].vertices, object.shapes[i].triangles);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::BOX)
    {
      ROS_INFO("[cspace] Attaching a '%s' cube with dimensions {%0.3fm x %0.3fm x %0.3fm}.", object.id.c_str(), object.shapes[i].dimensions[0], object.shapes[i].dimensions[1], object.shapes[i].dimensions[2]);
      cspace_->attachCube(object.id, link_name, object.poses[i], object.shapes[i].dimensions[0], object.shapes[i].dimensions[1], object.shapes[i].dimensions[2]);
    }
    else
      ROS_WARN("[cspace] Currently attaching objects of type '%d' aren't supported.", object.shapes[i].type);
  }
}

bool PR2CollisionSpaceInterface::getRobotPoseFromRobotState(arm_navigation_msgs::RobotState &state, vector<double>& langles, vector<double>& rangles, BodyPose& body)
{
	unsigned int lind = 0, rind = 0;
	langles.resize(ljoint_names_.size());
	rangles.resize(rjoint_names_.size());

	// arms
	for(size_t i = 0; i < state.joint_state.name.size(); i++)
	{
    ROS_DEBUG("Joint name: %s",state.joint_state.name[i].c_str());
    for(unsigned int j=0; j < rjoint_names_.size(); j++)
    {
			if(rjoint_names_[j].compare(state.joint_state.name[i]) == 0)
			{
				ROS_DEBUG("[exp] [right-start] %-20s: %0.3f", rjoint_names_[j].c_str(), state.joint_state.position[i]);
				rangles[j] = state.joint_state.position[i];
				rind++;
			}
    }
    for(unsigned int j=0; j < ljoint_names_.size(); j++)
		{
			if(ljoint_names_[j].compare(state.joint_state.name[i]) == 0)
			{
				ROS_DEBUG("[exp] [left-start] %-20s: %0.3f", ljoint_names_[j].c_str(), state.joint_state.position[i]);
				langles[j] = state.joint_state.position[i];
				lind++;
			}
		}
		if(rind == rjoint_names_.size() && lind == ljoint_names_.size())
			break;
	}

	if(rind != rjoint_names_.size() || lind != ljoint_names_.size())
	{
		ROS_WARN("[exp] Not all of the expected joints were assigned a starting position.");
		return false;
	}

	// torso
	for(size_t i = 0; i < state.joint_state.name.size(); i++)
	{
		if(state.joint_state.name[i].compare("torso_lift_joint") == 0)
		{
			body.z = state.joint_state.position[i];
			break;
		}
	}

	// base
  KDL::Frame f;
  if(!leatherman::getFrame(state.multi_dof_joint_state, world_frame_, grid_->getReferenceFrame(), f))
    return false;
  body.x = f.p.x();
  body.x = f.p.y();
  double r, p;
  f.M.GetRPY(r, p, body.theta);

  return true;
}

void PR2CollisionSpaceInterface::setCollisionToWorldTransform(const KDL::Frame &f, std::string &name)
{
  world_frame_ = name;
	cspace_->setKinematicsToReferenceTransform(f, world_frame_);
}

visualization_msgs::MarkerArray PR2CollisionSpaceInterface::getVisualization(std::string type, std::string ns, int id)
{
  visualization_msgs::MarkerArray ma;

  if(ns.empty())
    ns = type;

  if(type.compare("collision_object_voxels") == 0)
  {
    std::vector<geometry_msgs::Pose> poses;
    std::vector<std::vector<double> > points(1, std::vector<double>(3, 0));
    std::vector<double> color(4, 1); color[2] = 0;
    cspace_->getCollisionObjectVoxelPoses(poses);

    points.resize(poses.size());
    for (size_t i = 0; i < poses.size(); ++i)
    {
      points[i].resize(3);
      points[i][0] = poses[i].position.x;
      points[i][1] = poses[i].position.y;
      points[i][2] = poses[i].position.z;
    }
    ma.markers.push_back(viz::getCubesMarker(points, 0.01, color, world_frame_, ns, id));
  }
  else if(type.compare("collision_objects") == 0)
  {
    for(std::map<std::string, arm_navigation_msgs::CollisionObject>::const_iterator iter = object_map_.begin(); iter != object_map_.end(); ++iter)
    {
      std::vector<double> hue(iter->second.shapes.size(), 94);
      ma = viz::getCollisionObjectMarkerArray(iter->second, hue, iter->second.id, 0);
    }
  }
  else if(type.compare("attached_objects") == 0)
  {
    if(!attached_object_)
      return ma;

    std::vector<std::vector<double> > spheres;
    cspace_->getAttachedObjectSpheres(langles_, rangles_, body_pos_, spheres); 

    if(spheres.empty())
      return ma;
    
    std::vector<int> hue(spheres.size(), 163);
    ma = viz::getSpheresMarkerArray(spheres, hue, world_frame_, ns, id);
  }
  else if(type.compare("collision_model") == 0)
  {
    printRobotState(rangles_, langles_, body_pos_, "COLLISION MODEL FOR VIZ");
    ma = getCollisionModelVisualization(rangles_, langles_, body_pos_, ns, id);
  }
  else
    ma = grid_->getVisualization(type);

  return ma;
}

visualization_msgs::MarkerArray PR2CollisionSpaceInterface::getCollisionModelVisualization(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string ns, int id)
{
  visualization_msgs::MarkerArray ma;
  std::vector<std::vector<double> > spheres, all_spheres;
  std::vector<int> hues, all_hues;
  std::string frame;

  std::vector<std::string> sphere_groups(12);
  sphere_groups[0] = "base"; 
  sphere_groups[1] = "torso_upper";
  sphere_groups[2] = "torso_lower";
  sphere_groups[3] = "turrets";
  sphere_groups[4] = "tilt_laser";
  sphere_groups[5] = "head";
  sphere_groups[6] = "left_gripper";
  sphere_groups[7] = "right_gripper";
  sphere_groups[8] = "left_forearm";
  sphere_groups[9] = "right_forearm";
  sphere_groups[10] = "right_arm";
  sphere_groups[11] = "left_arm";

  for(std::size_t i = 0; i < sphere_groups.size(); ++i)
  {
    cspace_->getCollisionSpheres(langles, rangles, body_pos, sphere_groups[i], spheres); 
    all_spheres.insert(all_spheres.end(), spheres.begin(), spheres.end());
    hues.clear();
    hues.resize(spheres.size(), (i+1)*20);
    all_hues.insert(all_hues.end(), hues.begin(), hues.end());
  }
  ma = viz::getSpheresMarkerArray(all_spheres, all_hues, world_frame_, ns, id);
  return ma;
}

void PR2CollisionSpaceInterface::printRobotState(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text)
{
	ROS_INFO("robot state:  %s", text.c_str());
	ROS_INFO("     x: %0.3f  y: % 0.3f  z: %0.3f yaw: % 0.3f", body_pos.x, body_pos.y, body_pos.z, body_pos.theta);
	ROS_INFO(" right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", rangles[0], rangles[1], rangles[2], rangles[3], rangles[4], rangles[5], rangles[6]);
	ROS_INFO("  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", langles[0], langles[1], langles[2], langles[3], langles[4], langles[5], langles[6]);
}

}

