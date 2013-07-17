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

/* /author Benjamin Cohen */

#ifndef _PR2_COLLISION_SPACE_
#define _PR2_COLLISION_SPACE_

#include <ros/ros.h>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>
#include <leatherman/bresenham.h>
#include <leatherman/utils.h>
#include <pr2_collision_checker/sbpl_arm_model.h>
#include <pr2_collision_checker/sbpl_arm_planning_error_codes.h>
#include <sbpl_manipulation_components/occupancy_grid.h>
#include <sbpl_geometry_utils/Voxelizer.h>
#include <sbpl_geometry_utils/Interpolator.h>
#include <sbpl_geometry_utils/SphereEncloser.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <pviz/pviz.h>

using namespace std;

static const std::string arm_side_names[2] = {"right", "left"};

namespace pr2_collision_checker
{

enum Side
{
  Right, 
  Left,
  Body
};

typedef struct
{
  short unsigned int x;
  short unsigned int y;
  short unsigned int z;
  bool bIsObstacle;
} CELL3V;

typedef struct
{
  std::string name;
  KDL::Vector v;
  double radius;
  int radius_c;
  int priority;
  int voxel[3]; // temp variable
} Sphere;

typedef struct
{
  std::string name;
  std::string root_frame;
  std::vector<Sphere> spheres;
  int kdl_chain;
  int kdl_segment;
  KDL::Frame f; // temp variable
} Group;

typedef struct
{
  int priority;
  double x1;
  double y1;
  double z1;
  double x2;
  double y2;
  double z2;
  double radius;
  KDL::Vector p1;
  KDL::Vector p2;
  std::string name;
  std::string group;
  std::string frame;
} CollisionLink;

typedef struct
{
  Side side;   // 0: right, 1: left
  std::string name;
  int kdl_segment;
  std::string link;
  KDL::Frame pose;  // pose in link frame
  KDL::Frame f;     // temp variable
  std::vector<Sphere> spheres;
} AttachedObject;

class PR2CollisionSpace
{
  public:
    /* constructors */
    PR2CollisionSpace(sbpl_arm_planner::SBPLArmModel* right_arm, sbpl_arm_planner::SBPLArmModel* left_arm, sbpl_arm_planner::OccupancyGrid* grid);
    
    PR2CollisionSpace(std::string rarm_file, std::string larm_file, std::vector<double> &dims, std::vector<double> &origin, double resolution, std::string frame_id);

    ~PR2CollisionSpace();

    bool init();

    /* collision checking */
    bool checkCollision(const std::vector<double> &angles, BodyPose &pose, char i_arm, bool verbose, double &dist);

    bool checkCollisionArms(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, bool verbose, double &dist);

    bool checkCollisionArms(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, bool verbose, double &dist, int &debug_code, bool self=true);

    bool checkCollision(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, double &dist, int &debug_code);

    bool checkLinkForCollision(const std::vector<double> &angles, BodyPose &pose, char i_arm, int link_num, bool verbose, double &dist);

    bool checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, BodyPose &pose, char i_arm, bool verbose, double &dist);

    bool checkPathForCollision(const std::vector<double> &start0, const std::vector<double> &end0, const std::vector<double> &start1, const std::vector<double> &end1, BodyPose &pose, bool verbose, double &dist);
    
    bool checkPathForCollision(const std::vector<double> &start0, const std::vector<double> &end0, const std::vector<double> &start1, const std::vector<double> &end1, BodyPose &pose, bool verbose, double &dist, int &debug_code);
      
    bool checkLinkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, BodyPose &pose, char i_arm, int link_num, bool verbose, double &dist);
    
    bool checkCollisionBetweenArms(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, bool verbose, double &dist);

    /* linearly interpolate trajectories */
    void getFixedLengthInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, int path_length, std::vector<std::vector<double> > &path);

    /* access grid */ 
    double isValidLineSegment(const std::vector<int> a, const std::vector<int> b, double radius);
 
    /** collision objects **/
    void addArmCuboidsToGrid(char i_arm);
    void processCollisionObjectMsg(const arm_navigation_msgs::CollisionObject &object);
    void removeCollisionObject(const arm_navigation_msgs::CollisionObject &object);
    void removeAllCollisionObjects();
    void putCollisionObjectsInGrid();
    void getCollisionObjectVoxelPoses(std::vector<geometry_msgs::Pose> &points);
    void addCollisionObject(const arm_navigation_msgs::CollisionObject &object);

    /* attached objects */
    void removeAttachedObject(std::string name);
    void removeAllAttachedObjects();
    void attachSphere(std::string name, std::string link, geometry_msgs::Pose pose, double radius);
    void attachCube(std::string name, std::string link, geometry_msgs::Pose pose, double x_dim, double y_dim, double z_dim);
    void attachCylinder(std::string name, std::string link, geometry_msgs::Pose pose, double radius, double length);
    void attachMesh(std::string name, std::string link, geometry_msgs::Pose pose, const std::vector<geometry_msgs::Point> &vertices, const std::vector<int> &triangles);
    void getAttachedObjectSpheres(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, std::vector<std::vector<double> > &spheres);
    void getAttachedObjectVoxels(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, std::vector<std::vector<int> > &voxels);
    bool isAttachedObjectValid(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, bool verbose, double &dist, int &debug_code);

    /* kinematics & transformations */
    bool getJointPosesInGrid(const sbpl_arm_planner::SBPLArmModel* arm, std::vector<double> angles, BodyPose &pose, std::vector<std::vector<int> > &jnts, bool verbose);
    bool getJointPosesInGrid(const std::vector<double> angles, BodyPose &pose, char i_arm, std::vector<std::vector<int> > &jnts);
    void transformPose(const std::string &current_frame, const std::string &desired_frame, const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out);

    /* full body planning */
    bool getCollisionLinks();
    void printCollisionLinks();
    bool getSphereGroups();
    void printSphereGroups();
    bool initFullBodyKinematics();
    void printKDLChain(std::string name, KDL::Chain &chain);
    bool computeFullBodyKinematics(double x, double y, double theta, double torso, int frame_num, KDL::Frame &fk_out);
    void getMaptoRobotTransform(double x, double y, double theta, KDL::Frame &frame);
    void getVoxelsInGroup(KDL::Frame &frame, Group &group);
    void getPointsInGroup(KDL::Frame &frame, Group &group, std::vector<std::vector<double> > &points);
    void setNonPlanningJointPosition(std::string name, double value);

    bool isBaseValid(double x, double y, double theta, double &dist);
    bool isTorsoValid(double x, double y, double theta, double torso, double &dist);
    bool isHeadValid(double x, double y, double theta, double torso, double &dist);
    bool isBodyValid(double x, double y, double theta, double torso, double &dist);
    bool checkCollisionArmsToBody(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, double &dist);

    void getCollisionSpheres(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, std::string group_name, std::vector<std::vector<double> > &spheres);

    void printGroupVoxels(Group &g, std::string text);

    bool checkArmsMotion(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, double &dist, int &debug_code);
    bool checkBaseMotion(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, double &dist, int &debug_code);
    bool checkSpineMotion(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, double &dist, int &debug_code);
    bool checkAllMotion(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, double &dist, int &debug_code);

    //std::string getExpectedAttachedObjectFrame(std::string frame);

    void setKinematicsToReferenceTransform(KDL::Frame f, std::string &name);

    std::string getKinematicsFrame();

    bool isObjectAttached();

    void storeCollisionMap(const arm_navigation_msgs::CollisionMap &collision_map);

    /* TAR Project  (Note: rangles first)*/
    bool checkGroupAgainstWorld(Group* group, double &dist);
    bool checkGroupAgainstGroup(Group *g1, Group *g2, double &dist);
    bool checkRobotAgainstWorld(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &pose, bool verbose, double &dist);
    bool checkRobotAgainstGroup(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &pose, Group *group, bool verbose, bool gripper, double &dist);
    bool checkRobotAgainstRobot(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &pose, bool verbose, double &dist);

    visualization_msgs::MarkerArray getCollisionModelVisualization(std::vector<double> &rangles, std::vector<double> &langles, BodyPose &body_pos, std::string ns, int id);

    visualization_msgs::MarkerArray getVisualization(std::string type, std::string ns, int id);

    void addCollisionObjectMesh(const std::vector<geometry_msgs::Point> &vertices, const std::vector<int> &triangles, const geometry_msgs::Pose &pose, std::string name);
    bool addCollisionObjectMesh(std::string mesh_resource, geometry_msgs::Pose &pose, std::string name);
    bool addCollisionObjectMesh(std::string mesh_resource, geometry_msgs::Vector3 &scale, geometry_msgs::Pose &pose, std::string name);

    visualization_msgs::MarkerArray getGroupVisualization(Group &group, std::string ns, int id);

    void visualizeGroup(Group &group, std::string ns, int id);

    void printRobotState(std::vector<double> &rangles, std::vector<double> &langles,    BodyPose &body_pos, std::string text);

    void visualizeResult(bool enable){visualize_result_ = enable; };

  private:

    /** @brief arm model used by planner */
    std::vector<sbpl_arm_planner::SBPLArmModel*> arm_;

    /** @brief occupancy grid used by planner */
    sbpl_arm_planner::OccupancyGrid* grid_;

    /** Just for TAR project **/
    PViz pviz_;
    bool visualize_result_;

    std::string cspace_log_;

    /** \brief map from object id to object details */
    std::map<std::string, arm_navigation_msgs::CollisionObject> object_map_;

    /** \brief map from object id to list of occupying voxels */
    std::map<std::string, std::vector<Eigen::Vector3d> > object_voxel_map_;

    std::vector<std::string> known_objects_;
    std::vector<std::vector<double> > arm_min_limits_;
    std::vector<std::vector<double> > arm_max_limits_;
    std::vector<double> inc_;

    /** @brief temporary variable that's used often */
    KDL::Frame frame_;

    bool delete_objects_;

    /* attached objects */
    bool is_object_attached_;
    std::vector<KDL::Vector> attached_object_;
    std::vector<double> object_radius_w_;
    std::vector<int> object_radius_;
    std::vector<std::vector<int> > attached_voxels_;  //temp
    std::string attached_robot_link_; ///< Frame name of link the attached_object_pose_ is defined in
    KDL::Frame attached_object_pose_; ///< Pose of attached object in attached robot link's frame
    KDL::Frame attached_robot_link_in_multi_dof_; ///< Pose of attached robot link in multi-dof joint frame
    KDL::Frame attached_object_in_multi_dof_; ///< Pose of attached object in multi-dof joint frame

    double object_filling_sphere_radius_;

    /* full body planning */
    std::vector<CollisionLink> cl_;
    std::vector<CollisionLink> basel_;
    std::vector<CollisionLink> torsol_;
    std::vector<CollisionLink> headl_;

    Group base_g_;
    Group turrets_g_;
    Group torso_upper_g_;
    Group torso_lower_g_;
    Group tilt_laser_g_;
    Group head_g_;
    Group rgripper_g_;
    Group lgripper_g_;
    Group lforearm_g_;
    Group rforearm_g_;
    std::vector<Group> all_g_;

    std::string full_body_chain_root_name_;
    std::string full_body_chain_tip_name_;
    KDL::JntArray jnt_array_;
    KDL::Frame fk_out_;
    KDL::Frame map_to_robot_;
    KDL::Tree full_body_tree_;
    KDL::Chain full_body_chain_;
    KDL::ChainFkSolverPos_recursive* fk_solver_;

    /* non-planning joint angles */
    double head_pan_angle_;
    double head_tilt_angle_;

    bool checkCollisionArmsToGroup(Group &group, double &dist);

    /* ---- attached object ---- */
    std::vector<AttachedObject> objects_;
    std::string attached_object_frame_suffix_;

    int getAttachedObjectIndex(std::string name);
    //bool getAttachedFrameInfo(std::string frame, int &segment, int &chain);
    int getSegmentIndex(std::string &name, KDL::Chain &chain);

    arm_navigation_msgs::CollisionMap last_collision_map_;
};

} 
#endif

