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

#ifndef _SBPL_ARM_MODEL_
#define _SBPL_ARM_MODEL_

#include <string>
#include <map>
#include <vector>
#include <sbpl/config.h>
#include <ros/ros.h>
#include <angles/angles.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <urdf/model.h>
#include <pr2_collision_checker/sbpl_arm_planning_error_codes.h>
#include <pr2_arm_kinematics/pr2_arm_ik_solver.h>
#include <pr2_arm_kinematics/pr2_arm_ik.h>
//#include <Eigen/Array>
#include <Eigen/Core>
#include <pr2_collision_checker/body_pose.h>

using namespace std;

namespace sbpl_arm_planner {

/** \brief this struct contains a description of each joint in the arm. It's
 * mainly used for checking joint limits. It's very primitive and will be
 * redesigned later on.
*/
struct ArmJoint
{
  bool continuous;
  double min;
  double max;
  std::string name;
};

/** \brief this struct contains a description of a link on the arm.
 * It's mainly used for collision checking. It is very primitive and will be
 * redesigned later on.
*/
struct ArmLink
{
  short unsigned int radius_c;
  short unsigned int length_c;
  int ind_chain;
  int ind_jnt1;
  int ind_jnt2;
  double length;
  double radius;
  std::string name;
};


class SBPLArmModel{

  public:

    /** 
     * @brief default constructor 
     * @param the filename of the sbpl arm description text file 
     * */
    SBPLArmModel(FILE* arm_file);
    
    /** \brief destructor */
    ~SBPLArmModel();
    
    /** \brief number of joints to be planned for */
    int num_joints_;

    /** \brief number of links (for collision checking)*/
    int num_links_;

    /** resolution of grid for the discretized arm dims */
    double resolution_;
    
    /** \brief initialize the KDL chain for FK and IK */
    bool initKDLChain(const std::string &fKDL);

    /** \brief grab the robot description file from the param server */
    bool initKDLChainFromParamServer();

    /** \brief parse the arm description text file */
    bool getArmDescription(FILE* fCfg);

    /** \brief compute the axis angle between 2 rotation matrices */
    double getAxisAngle(const double R1[3][3], const double R2[3][3]);

    /** \brief convert RPY values to a rotation matrix */
    void RPY2Rot(double roll, double pitch, double yaw, double Rot[3][3]);

    KDL::Frame computeBodyFK(const BodyPose pose);

    bool computeArmFK(const std::vector<double> angles, int frame_num, KDL::Frame *frame_out);
    
    /** \brief compute the forward kinematics to get the pose of the specified joint*/
    bool computeFK(const std::vector<double> angles, const BodyPose pose, int frame_num, KDL::Frame *frame_out);
    bool computeFK(const std::vector<double> angles, int frame_num, KDL::Frame *frame_out){printf("Using old FK 1\n"); exit(0);};

    /** \brief compute the forward kinematics to get the pose of the specified joint*/
    bool computeFK(const std::vector<double> angles, const BodyPose pose, int f_num, std::vector<double> &xyzrpy);
    bool computeFK(const std::vector<double> angles, int f_num, std::vector<double> &xyzrpy){printf("Using old FK 2\n"); exit(0);};

    /** \brief (added for debugging - can be removed) compute the forward
     * kinematics to get the pose of the specified joint, use sol_num to
     * choose which RPY representation you want */
    bool computeFK(const std::vector<double> angles, const BodyPose pose, int frame_num, std::vector<double> &xyzrpy, int sol_num);
    bool computeFK(const std::vector<double> angles, int frame_num, std::vector<double> &xyzrpy, int sol_num){printf("Using old FK 3\n"); exit(0);};
    
    /** \brief compute the position {x,y,z} of the planning_joint & the axis
     * angle which represents the difference to the target frame */ 
    bool computeEndEffPose(const std::vector<double> angles, const BodyPose pose, double R_target[3][3], double *x, double *y, double *z, double *axis_angle);

    /** \brief compute the inverse kinematics for the planning_joint pose */
    bool computeIK(const std::vector<double> pose, const std::vector<double> start, std::vector<double> &solution);
    
    bool computeIK(const KDL::Frame &frame, const std::vector<double> start, std::vector<double> &solution);

    /** \brief compute the closed form inverse kinematics solution  */
    bool computeFastIK(const std::vector<double> pose, const std::vector<double> start, std::vector<double> &solution);

    bool computeFastIK(const KDL::Frame &frame, const std::vector<double> start, std::vector<double> &solution);

    /** \brief compute the positions of all the tips of the links (collision checking)*/ 
    bool getJointPositions(const std::vector<double> angles, const BodyPose pose, const double R_target[3][3], std::vector<std::vector<double> > &links, double *axis_angle);
  
    bool getJointPositions(const std::vector<double> angles, const BodyPose pose, std::vector<std::vector<double> > &links, KDL::Frame &f_out);
    
    bool getJointPositions(const std::vector<double> angles, std::vector<std::vector<double> > &links, KDL::Frame &f_out) {printf("NOT IMPLEMENTED"); exit(1);};

    /** \brief compute the coordinates in the world frame of the planning_joint */ 
    bool computePlanningJointPos(const std::vector<double> angles, const BodyPose pose, double *x, double *y, double *z);
   
    bool getPlanningJointPose(const std::vector<double> angles, const BodyPose body_pose, std::vector<double> &pose);
    bool getPlanningJointPose(const std::vector<double> angles, std::vector<double> &pose)
    {
    	printf("Called old getPlanningJointPose\n");
    	exit(0);
    }

    /** \brief compute the pose in the world frame of the planning_joint */ 
    bool getPlanningJointPose(const std::vector<double> angles, const BodyPose body_pose, double R_target[3][3], std::vector<double> &pose, double *axis_angle);

    /** \brief return true or false if angles are within specified joint limits */
    bool checkJointLimits(std::vector<double> angles, bool verbose);

    bool checkJointLimits(std::vector<double> angles, int &debug_code);

    /** \brief print description of arm from SBPL arm text file */
    void printArmDescription(std::string stream);

    /** \brief get the radius (in meters) of the widest link */
    int getMaxLinkRadius();

    /** \brief get radius of link in grid cells */
    short unsigned int getLinkRadiusCells(int link);
    
    double getLinkRadius(int link);

    /** \brief get radius of link in grid cells */
    short unsigned int getPlanningJointRadius();
   
    /** \brief set the resolution used for collision checking */
    void setResolution(double resolution);

    /** \brief set the transform from the torso frame to the base frame
     * (really supposed to be a transform from the frame that the kinematics
     * is done to the world frame that the planning is done) */
    void setRefFrameTransform(KDL::Frame f, std::string &name);

    void getArmChainRootLinkName(std::string &name);

    /** \brief get the self collision cuboids that the robot occupies */
    std::vector<std::vector<double> >  getCollisionCuboids();

    /** \brief set the file used for debug output  */
    void setDebugLogName(std::string stream_name);

    /** \brief get the joint postition limits of specified joint */
    void getJointLimits(int joint_num, double* min, double *max);

    double getMaxJointLimit(int joint_num);

    double getMinJointLimit(int joint_num);

    bool computeTranslationalIK(const std::vector<double> pose, double upperarm_roll, std::vector<double> &solution);

    //int getJointLimitDebugCode(const int &joint);

    int getSegmentIndex(std::string &name);

  private:

    /** \brief a string containing the URDF that describes the robot arm */
    std::string name_;

	  std::string robot_description_;

    /** \brief the name of the root frame in the KDL chain */
    std::string chain_root_name_;

    /** \brief the name of the tip frame in the KDL chain */
    std::string chain_tip_name_;

    /** \brief frame that the planning is done in */
    std::string reference_frame_;

    /** \brief the robot model used by the KDL */
    urdf::Model robot_model_;

    /** \brief the kdl tree for the robot description */
    KDL::Tree kdl_tree_;

    /** \brief the index of the joint that is being planned for*/
    int planning_joint_;

    double ik_search_inc_;

    /** \brief the name of the joint that is being planned for */
    std::string planning_joint_name_;

    /** \brief the radius of the widest link */
    short unsigned int max_radius_;

    /** \brief a vector of joints */
	  std::vector<ArmJoint> joints_;

    /** \brief a vector of links */
	  std::vector<ArmLink> links_;

    /** \brief a vector of joint indeces used for collision checking. */
    std::vector<int> joint_indeces_;

    KDL::Frame transform_;
    
    KDL::Frame transform_inverse_;

    std::vector<std::vector<double> > collision_cuboids_;

    int num_collision_cuboids_;

    std::string debug_log_;

    std::vector<int> debug_code_types_;

    //copied from CHOMP - but not used as of now
    void parseKDLTree();
	  int num_kdl_joints_;
    std::map<std::string, std::string> joint_segment_mapping_;    /**< Joint -> Segment mapping for KDL tree */
    std::map<std::string, std::string> segment_joint_mapping_;    /**< Segment -> Joint mapping for KDL tree */
    std::vector<std::string> kdl_number_to_urdf_name_;            /**< Mapping from KDL joint number to URDF joint name */
    std::map<std::string, int> urdf_name_to_kdl_number_;          /**< Mapping from URDF joint name to KDL joint number */

    KDL::JntArray jnt_pos_in_;
	  KDL::JntArray jnt_pos_out_;
	  KDL::Frame p_out_;
	  
    KDL::Chain chain_;
	  KDL::ChainFkSolverPos_recursive *fk_solver_;
	  KDL::JntArray ik_jnt_pos_in_;
	  KDL::JntArray ik_jnt_pos_out_;
	 
    pr2_arm_kinematics::PR2ArmIKSolver* pr2_arm_ik_solver_;
   
    //added for Cartesian Planner 
    pr2_arm_kinematics::PR2ArmIK pr2_arm_ik_;

    /** \brief calculate axis angle between a KDL frame and a Rot matrix */
    double frameToAxisAngle(const KDL::Frame frame, const double R_target[3][3]);
   
    /** \brief convert a rotation matrix into roll,pitch,yaw */ 
    void getRPY(double Rot[3][3], double* roll, double* pitch, double* yaw, int solution_number);
};


inline short unsigned int SBPLArmModel::getLinkRadiusCells(int link)
{
  if(link < num_links_)
    return links_[link].radius_c;
  return 0;
}

inline double SBPLArmModel::getLinkRadius(int link)
{
  if(link < num_links_)
    return links_[link].radius;
  return 0;
}

inline int SBPLArmModel::getMaxLinkRadius()
{
  return max_radius_;
}

inline std::vector<std::vector<double> >  SBPLArmModel::getCollisionCuboids()
{
  return collision_cuboids_;
}

inline double SBPLArmModel::getMaxJointLimit(int joint_num)
{
  return joints_[joint_num].max;
}

inline double SBPLArmModel::getMinJointLimit(int joint_num)
{
  return joints_[joint_num].min;
}

}

#endif
