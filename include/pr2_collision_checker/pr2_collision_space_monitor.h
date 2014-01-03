/*
 * Copyright (c) 2011, Maxim Likhachev
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

/** \author Benjamin Cohen  */

#ifndef _PR2_COLLISION_SPACE_MONITOR_H_
#define _PR2_COLLISION_SPACE_MONITOR_H_

#include <iostream>
#include <map>
#include <log4cxx/logger.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <message_filters/subscriber.h>
#include <pviz/pviz.h>
#include <pr2_collision_checker/body_pose.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>

/** Messages **/
#include <arm_navigation_msgs/CollisionMap.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h> 


namespace pr2_collision_checker
{

class PR2CollisionSpaceMonitor
{
  public:

    PR2CollisionSpaceMonitor(sbpl_arm_planner::OccupancyGrid *grid, pr2_collision_checker::PR2CollisionSpace *cspace);

    ~PR2CollisionSpaceMonitor();

    bool init();

    void attachObject(const arm_navigation_msgs::CollisionObject &obj, std::string link_name);

    void getRobotState(BodyPose &body_pos, std::vector<double> &rangles, std::vector<double> &langles);
    
    bool getRobotPoseFromRobotState(arm_navigation_msgs::RobotState &state, vector<double>& langles, vector<double>& rangles, BodyPose& body);

    /** debug output **/
    void printRobotState(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text);

    /** visualizations **/
    void visualizeCollisionObjects(bool delete_first);
    void visualizeAttachedObject(bool delete_first=false);
    void visualizeCollisionObject(const arm_navigation_msgs::CollisionObject &object);
    void visualizeCollisionModel(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string text);

  private:

    ros::NodeHandle root_handle_;
    ros::NodeHandle node_handle_;
    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber collision_object_subscriber_;
    ros::Subscriber object_subscriber_;
    arm_navigation_msgs::CollisionMap last_collision_map_;        
    tf::MessageFilter<arm_navigation_msgs::CollisionMap>* collision_map_filter_;
    message_filters::Subscriber<arm_navigation_msgs::CollisionMap> collision_map_subscriber_;

    PViz pviz_;
    pr2_collision_checker::PR2CollisionSpace* cspace_;
    sbpl_arm_planner::OccupancyGrid* grid_;
    std::map<std::string, arm_navigation_msgs::CollisionObject> object_map_;

    /** general params **/
    bool attached_object_;
    std::string map_frame_;
    std::string reference_frame_;
    bool visualize_collision_model_;
    bool use_collision_map_from_sensors_;

    /** robot state & kinematics **/
    double torso_lift_;
    double head_pan_;
    double head_tilt_;
    BodyPose body_pos_;
    std::vector<double> rangles_;
    std::vector<double> langles_;
    std::vector<std::string> ljoint_names_;
    std::vector<std::string> rjoint_names_;
    tf::TransformListener tf_;
    tf::StampedTransform transform_;
    tf::StampedTransform base_map_transform_;
    KDL::Frame kdl_transform_;

    /** callbacks **/
    void collisionMapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map);
    void collisionObjectCallback(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object);
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr &state);
    void attachedObjectCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr &attached_object);
    
    void changeLoggerLevel(std::string name, std::string level);
    
    void setKinematicsToReferenceTransform(std::string &map_frame);
};

}

#endif
