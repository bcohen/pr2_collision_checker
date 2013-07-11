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

#include <pr2_collision_checker/pr2_collision_space_monitor.h>

namespace pr2_collision_checker
{

/** Initializers -------------------------------------------------------------*/
PR2CollisionSpaceMonitor::PR2CollisionSpaceMonitor(sbpl_arm_planner::OccupancyGrid *grid, pr2_collision_checker::PR2CollisionSpace *cspace) :
	node_handle_("~"),
	collision_map_filter_(NULL),
	collision_map_subscriber_(root_handle_, "collision_map_occ", 1),
	attached_object_(false),
	map_frame_("map")
{
  cspace_ = cspace;
  grid_ = grid;

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

PR2CollisionSpaceMonitor::~PR2CollisionSpaceMonitor()
{
	if (collision_map_filter_ != NULL) delete collision_map_filter_;
}

bool PR2CollisionSpaceMonitor::init()
{
	// general params
	node_handle_.param<std::string>("reference_frame", reference_frame_, std::string("map"));
	node_handle_.param<std::string>("collision_map_topic", collision_map_topic_, "collision_map_occ");

	// visualizations params
	node_handle_.param("visualizations/collision_model", visualize_collision_model_, false);

	typedef tf::MessageFilter<arm_navigation_msgs::CollisionMap> collisionMapFilter;
	collision_map_filter_ = new collisionMapFilter(collision_map_subscriber_, tf_, reference_frame_, 1);
	collision_map_filter_->registerCallback(boost::bind(&PR2CollisionSpaceMonitor::collisionMapCallback, this, _1));
	joint_states_subscriber_ = root_handle_.subscribe("joint_states", 1, &PR2CollisionSpaceMonitor::jointStatesCallback, this);
	collision_object_subscriber_ = root_handle_.subscribe("collision_object", 1, &PR2CollisionSpaceMonitor::collisionObjectCallback, this);
	object_subscriber_ = root_handle_.subscribe("attached_collision_object", 1, &PR2CollisionSpaceMonitor::attachedObjectCallback, this);

  ROS_INFO("[cspace] The PR2CollisionSpaceMonitor is ready to rock.");
	return true;
}


/********************************** Callbacks *********************************/
void PR2CollisionSpaceMonitor::collisionMapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map)
{
  if (collision_map->header.frame_id.compare(reference_frame_) != 0 &&
      collision_map->header.frame_id.compare("/" + reference_frame_) != 0) {
    ROS_WARN("[cspace] The collision map received is in %s frame but expected in %s frame.", collision_map->header.frame_id.c_str(), reference_frame_.c_str());
  }

  // add collision map msg
  if (use_collision_map_from_sensors_) {
    grid_->updateFromCollisionMap(*collision_map);
  }

  last_collision_map_ = *collision_map;
  cspace_->storeCollisionMap(*collision_map);
  map_frame_ = collision_map->header.frame_id;
  setKinematicsToReferenceTransform(map_frame_);

  cspace_->putCollisionObjectsInGrid();
  //grid_->visualize();
}

void PR2CollisionSpaceMonitor::collisionObjectCallback(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object)
{
  // for some reason, it wasn't getting all of the 'all' messages...
  if (collision_object->id.compare("all") == 0)
    cspace_->removeAllCollisionObjects();

  // debug: have we seen this collision object before?
  if (object_map_.find(collision_object->id) != object_map_.end())
    ROS_DEBUG("[cspace] We have seen this object ('%s')  before.", collision_object->id.c_str());
  else
    ROS_DEBUG("[cspace] We have NOT seen this object ('%s') before.", collision_object->id.c_str());

  // add the object to our internal map for later attaching
  // make sure to transform the poses into the correct frame
  arm_navigation_msgs::CollisionObject obj = *collision_object;
  obj.header.stamp = ros::Time(0);
  for (unsigned int i = 0; i < obj.poses.size(); i++) {
    geometry_msgs::PoseStamped ps;
    ps.header = obj.header;
    ps.pose = obj.poses[i];
    tf_.transformPose("map", ps, ps);
    obj.poses[i] = ps.pose;
  }
  obj.header.frame_id = "map";
  object_map_[collision_object->id] = obj;

  ROS_DEBUG("[cspace] %s", collision_object->id.c_str());
  cspace_->processCollisionObjectMsg(*collision_object);

  //visualize exact collision object
  visualizeCollisionObject(*collision_object);

  //visualize collision voxels
  //visualizeCollisionObjects(true);

  if (attached_object_)
    visualizeAttachedObject();
}

void PR2CollisionSpaceMonitor::jointStatesCallback(const sensor_msgs::JointStateConstPtr &state)
{
  // arms
	for(unsigned int i=0; i<rjoint_names_.size(); i++){
		unsigned int j;
		for(j=0; j<state->name.size(); j++)
			if(rjoint_names_[i].compare(state->name[j])==0)
				break;
		if(j==state->name.size())
			ROS_WARN("[jointStatesCallback] Missing the value for planning joint (%s)\n",rjoint_names_[i].c_str());
		else
			rangles_[i] = state->position[j];
	}
	for(unsigned int i=0; i<ljoint_names_.size(); i++){
		unsigned int j;
		for(j=0; j<state->name.size(); j++)
			if(ljoint_names_[i].compare(state->name[j])==0)
				break;
		if(j==state->name.size())
			ROS_WARN("[jointStatesCallback] Missing the value for planning joint (%s)\n",rjoint_names_[i].c_str());
		else
			langles_[i] = state->position[j];
	}

  // torso
	unsigned int j;
	for (j = 0; j < state->name.size(); j++)
		if (state->name[j].compare("torso_lift_joint") == 0) break;
	if(j==state->name.size())
		ROS_WARN("[jointStatesCallback] Missing the value for planning joint torso_lift_joint\n");
	else
		body_pos_.z = state->position[j];

  // base
	try {
		tf_.lookupTransform("map", "base_footprint", ros::Time(0), base_map_transform_);
		body_pos_.x = base_map_transform_.getOrigin().x();
		body_pos_.y = base_map_transform_.getOrigin().y();
		body_pos_.theta = 2 * atan2(base_map_transform_.getRotation().getZ(), base_map_transform_.getRotation().getW());
		ROS_DEBUG("Received transform from base_footprint to map (x: %f y: %f yaw: %f)", body_pos_.x, body_pos_.y, body_pos_.theta);
	}
	catch (tf::TransformException& ex) {
		ROS_ERROR("Is there a map? The map-robot transform failed. (%s)", ex.what());
	}

  if(visualize_collision_model_)
    visualizeCollisionModel(rangles_, langles_, body_pos_, "pr2_collision_model");
	
  if (attached_object_)
		visualizeAttachedObject();

  ROS_DEBUG("[cspace] [robot state] x: %0.3f  y: %0.3f  theta: %0.3f  torso: %0.3f", body_pos_.x, body_pos_.y, body_pos_.theta, body_pos_.z); 
}

void PR2CollisionSpaceMonitor::attachedObjectCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr &attached_object)
{
  // remove all objects
  if(attached_object->link_name.compare(arm_navigation_msgs::AttachedCollisionObject::REMOVE_ALL_ATTACHED_OBJECTS) == 0 &&
      attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
  {
    ROS_DEBUG("[cspace] Removing all attached objects.");
    attached_object_ = false;
    cspace_->removeAllAttachedObjects();
    visualizeAttachedObject(true);
  }
  // add object
  else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD)
  {
    ROS_INFO("[cspace] Received a message to ADD an object (%s) with %d shapes.", attached_object->object.id.c_str(), int(attached_object->object.shapes.size()));
    object_map_[attached_object->object.id] = attached_object->object;
    attachObject(attached_object->object, attached_object->link_name);
  }
  // attach object and remove it from collision space
  else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT)
  {
    ROS_INFO("[cspace] Received a message to ATTACH_AND_REMOVE_AS_OBJECT of object: %s", attached_object->object.id.c_str());

    // have we seen this collision object before?
    if(object_map_.find(attached_object->object.id) != object_map_.end())
    {
      ROS_INFO("[cspace] We have seen this object (%s) before.", attached_object->object.id.c_str());
      ROS_WARN("[cspace] Attached objects we have seen before are not handled correctly right now.");
      attachObject(object_map_.find(attached_object->object.id)->second, attached_object->link_name);
    }
    else
    {
      ROS_INFO("[cspace] We have NOT seen this object (%s) before (it's not in my internal object map).", attached_object->object.id.c_str());
      if(attached_object->object.shapes.empty())
      {
        ROS_WARN("[cspace] '%s' is not in my internal object map and the message doesn't contain any shapes. Can't attach it.", attached_object->object.id.c_str());
        return;
      }
      object_map_[attached_object->object.id] = attached_object->object;
      attachObject(attached_object->object, attached_object->link_name);
    }
    ROS_INFO("[3dnav] Just attached '%s', now I'll remove it from the world.", attached_object->object.id.c_str());
    cspace_->removeCollisionObject(attached_object->object);
    visualizeCollisionObjects(true);
    //grid_->visualize();
  }
  // remove object
  else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
  {
    ROS_DEBUG("[cspace] Removing object (%s) from gripper.", attached_object->object.id.c_str());
    cspace_->removeAttachedObject(attached_object->object.id);
    visualizeAttachedObject(true);
  }
  // detach and add as object
  else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT)
  {
    attached_object_ = false;
    ROS_DEBUG("[cspace] Removing object (%s) from gripper and adding to collision map.", attached_object->object.id.c_str());
    cspace_->removeAttachedObject(attached_object->object.id);
    attached_object_ = cspace_->isObjectAttached();
    visualizeAttachedObject(true);
    //sometimes people are lazy and they don't fill in the object's
    //description. in those cases - we have to depend on our stored
    //description of the object to be added. if we can't find it in our
    //object map, we hope that they added in a description themselves.
    if(object_map_.find(attached_object->object.id) != object_map_.end())
    {
      ROS_INFO("[3dnav] Detached '%s' and now I'll add it as a known collision object. The message does not contain its description but I had it stored so I know what it is.", attached_object->object.id.c_str());
      cspace_->addCollisionObject(object_map_.find(attached_object->object.id)->second);
    }
    else
    {
      if(attached_object->object.shapes.empty())
      {
        ROS_WARN("[3dnav] '%s' was attached and now you want to add it as a collision object. Unfortunatly the message does not contain its decription and I don't have it in my internal database for some reason so I can't attach it. This is probably a bug.", attached_object->object.id.c_str());
        return;
      }
      object_map_[attached_object->object.id] = attached_object->object;
      cspace_->addCollisionObject(attached_object->object);
    }
    //visualize exact collision object
    //visualizeCollisionObject(attached_object->object);
    visualizeCollisionObjects(true);
    //grid_->visualize();
  }
  else
    ROS_WARN("[3dnav] Received a collision object with an unknown operation");

  attached_object_ = cspace_->isObjectAttached();
}

void PR2CollisionSpaceMonitor::attachObject(const arm_navigation_msgs::CollisionObject &obj, std::string link_name)
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
    ROS_ERROR("Need to transform attached object properly!");
    /*
    try
    {
      tf_.transformPose(cspace_->getExpectedAttachedObjectFrame(link_name), pose_in, pose_out);
    }
    catch(int e)
    {
      ROS_ERROR("[cspace] Failed to transform the pose of the attached object from %s to %s. (exception: %d)", object.header.frame_id.c_str(), cspace_->getExpectedAttachedObjectFrame(object.header.frame_id).c_str(), e);
      ROS_ERROR("[cspace] Failed to attach '%s' object.", object.id.c_str());
      return;
    }
    */
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

  visualizeAttachedObject(true);
}

bool PR2CollisionSpaceMonitor::getRobotPoseFromRobotState(arm_navigation_msgs::RobotState &state, vector<double>& langles, vector<double>& rangles, BodyPose& body)
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
	for(size_t i = 0; i < state.multi_dof_joint_state.frame_ids.size(); ++i)
	{
		if(state.multi_dof_joint_state.frame_ids[i].compare("map") == 0 &&
				state.multi_dof_joint_state.child_frame_ids[i].compare("base_footprint") == 0)
		{
			body.x = state.multi_dof_joint_state.poses[i].position.x;
			body.y = state.multi_dof_joint_state.poses[i].position.y;

			tf::Quaternion qbase;
			tf::quaternionMsgToTF(state.multi_dof_joint_state.poses[i].orientation, qbase);
			body.theta =  2 * atan2(qbase.getZ(), qbase.getW());
			break;
		}
	}
	return true;
}

void PR2CollisionSpaceMonitor::setKinematicsToReferenceTransform(std::string &map_frame)
{
	std::string kinematics_frame = cspace_->getKinematicsFrame();

	// get transform from kinematics frame to reference frame
	try {
		tf_.lookupTransform(map_frame, kinematics_frame, ros::Time(0), transform_);

		ROS_DEBUG("Received transform from %s to %s (translation: %f %f %f)", kinematics_frame.c_str(), map_frame.c_str(), transform_.getOrigin().x(), transform_.getOrigin().y(), transform_.getOrigin().z());
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
	}

	// convert transform to a KDL object
	tf::TransformTFToKDL(transform_, kdl_transform_);
	cspace_->setKinematicsToReferenceTransform(kdl_transform_, map_frame);
}

/* Visualizations ------------------------------------------------------------*/
void PR2CollisionSpaceMonitor::visualizeCollisionObjects(bool delete_first)
{
	std::vector<geometry_msgs::Pose> poses;
	std::vector<std::vector<double> > points(1, std::vector<double>(3, 0));
	std::vector<double> color(4, 1);
	color[2] = 0;

  // first delete old visualizations
  if(delete_first)
    pviz_.deleteVisualizations("known_objects", 1000);
  
	cspace_->getCollisionObjectVoxelPoses(poses);

	points.resize(poses.size());
	for (size_t i = 0; i < poses.size(); ++i) {
		points[i].resize(3);
		points[i][0] = poses[i].position.x;
		points[i][1] = poses[i].position.y;
		points[i][2] = poses[i].position.z;
	}

	ROS_DEBUG("[cspace] Displaying %d known collision object voxels.", int(points.size()));
	pviz_.visualizeBasicStates(points, color, "known_objects", 0.01);
}

void PR2CollisionSpaceMonitor::visualizeAttachedObject(bool delete_first)
{
	std::vector<std::vector<double> > spheres;
	std::vector<double> color(4, 1);
	std::vector<double> radius;
	color[2] = 0;

  // first delete old visualizations
  if(delete_first)
    pviz_.deleteVisualizations("attached_objects", 500);

  cspace_->getAttachedObjectSpheres(langles_, rangles_, body_pos_, spheres); 
  
  if(spheres.empty())
    return;

	ROS_DEBUG("[cspace] Displaying %d spheres for the attached object.", int(spheres.size()));
	for (size_t i = 0; i < spheres.size(); ++i)
    ROS_DEBUG("[%d] xyz: %0.3f %0.3f %0.3f radius: %0.3f", int(i), spheres[i][0], spheres[i][1] , spheres[i][2], spheres[i][3]);

  pviz_.visualizeSpheres(spheres, 163, "attached_objects");
}

void PR2CollisionSpaceMonitor::visualizeCollisionObject(const arm_navigation_msgs::CollisionObject &object)
{
	geometry_msgs::PoseStamped pose;
  std::vector<double> dim;

  for (size_t i = 0; i < object.shapes.size(); ++i)
  {
    if (object.shapes[i].type == arm_navigation_msgs::Shape::BOX)
    {
      pose.pose = object.poses[i];
      pose.header = object.header;
      dim.resize(object.shapes[i].dimensions.size());
      for (size_t j = 0; j < object.shapes[i].dimensions.size(); ++j)
        dim[j] = object.shapes[i].dimensions[j];

      ROS_INFO("[cspace] Visualizing %s", object.id.c_str());
      pviz_.visualizeCube(pose, 94, object.id, int(i), dim);
    }
    else
      ROS_WARN("[cspace] Collision objects of type %d are not yet supported.", object.shapes[i].type);
	}
}

void PR2CollisionSpaceMonitor::visualizeCollisionModel(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text)
{
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
  pviz_.visualizeSpheres(all_spheres, all_hues, text);  
}

void PR2CollisionSpaceMonitor::getRobotState(BodyPose &body_pos, std::vector<double> &rangles, std::vector<double> &langles)
{
	sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage < sensor_msgs::JointState > ("joint_states");
	rangles[0] = state->position[17];
	rangles[1] = state->position[18];
	rangles[2] = state->position[16];
	rangles[3] = state->position[20];
	rangles[4] = state->position[19];
	rangles[5] = state->position[21];
	rangles[6] = state->position[22];

	langles[0] = state->position[29];
	langles[1] = state->position[30];
	langles[2] = state->position[28];
	langles[3] = state->position[32];
	langles[4] = state->position[31];
	langles[5] = state->position[33];
	langles[6] = state->position[34];
	body_pos_.z = state->position[12];

	try {
		tf_.lookupTransform("map", "base_footprint", ros::Time(0), base_map_transform_);
		body_pos.x = base_map_transform_.getOrigin().x();
		body_pos.y = base_map_transform_.getOrigin().y();
		body_pos_.theta = 2 * atan2(base_map_transform_.getRotation().getX(), base_map_transform_.getRotation().getW());
	}
	catch (tf::TransformException& ex) {
		ROS_ERROR("[cspace] Is there a map? The map-robot transform failed. (%s)", ex.what());
	}
}

void PR2CollisionSpaceMonitor::printRobotState(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text)
{
	ROS_INFO("robot state:  %s", text.c_str());
	ROS_INFO("     x: %0.3f  y: % 0.3f  z: %0.3f yaw: % 0.3f", body_pos.x, body_pos.y, body_pos.z, body_pos.theta);
	ROS_INFO(" right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", rangles[0], rangles[1], rangles[2], rangles[3], rangles[4], rangles[5], rangles[6]);
	ROS_INFO("  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", langles[0], langles[1], langles[2], langles[3], langles[4], langles[5], langles[6]);
}

void PR2CollisionSpaceMonitor::changeLoggerLevel(std::string name, std::string level)
{
	//ROSCONSOLE_AUTOINIT;

	std::string logger_name = ROSCONSOLE_DEFAULT_NAME + std::string(".") + name;

	ROS_INFO("[cspace] Setting the %s logger to %s level", logger_name.c_str(), level.c_str());

	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(logger_name);

	// Set the logger for this package to output all statements
	if (level.compare("debug") == 0)
		my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
	else
		my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

	ROS_DEBUG("[cspace] This is a debug statement, and should print if you enabled debug.");
}

}

