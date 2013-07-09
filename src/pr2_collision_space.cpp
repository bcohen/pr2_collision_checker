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
 /** \author Benjamin Cohen */

#include <pr2_collision_checker/pr2_collision_space.h>

#define SMALL_NUM  0.00000001     // to avoid division overflow

namespace pr2_collision_checker
{

double distance(const KDL::Vector& a, const KDL::Vector& b)
{
	return sqrt((a.x()-b.x())*(a.x()-b.x()) + (a.y()-b.y())*(a.y()-b.y()) + (a.z()-b.z())*(a.z()-b.z()));
}

PR2CollisionSpace::PR2CollisionSpace(sbpl_arm_planner::SBPLArmModel* right_arm, sbpl_arm_planner::SBPLArmModel* left_arm, sbpl_arm_planner::OccupancyGrid* grid) : grid_(grid)
{
  arm_.resize(2);
  arm_[0] = right_arm;
  arm_[1] = left_arm;
  fOut_ = stdout;
  cube_filling_sphere_radius_ = 0.04;

  cspace_log_ = "cspace";
  attached_object_frame_suffix_ = "_wrist_roll_link";

  //changed inc_ to a vector 8/28/2010
  inc_.resize(arm_[0]->num_joints_,0.0348);
  inc_[5] = 0.1392; // 8 degrees
  inc_[6] = M_PI; //rolling the wrist doesn't change the arm's shape
}

bool PR2CollisionSpace::checkCollisionArms(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist)
{
  int code;
  return checkCollisionArms(langles,rangles,pose,verbose,dist,code);
}

bool PR2CollisionSpace::checkCollisionArms(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  unsigned char dist_temp = 100;
  //dist = 100; NOTE: We are assuming dist comes in with some value that matters
  
  //check the right arm against the environment
  if(!checkCollision(rangles, pose, 0, verbose, dist_temp))
  {
    if(verbose)
      ROS_DEBUG_NAMED(cspace_log_, "  Right arm is in collision with the environment. (dist: %d)", int(dist_temp));
    dist = dist_temp;
    //code_ = "Right arm + Environment";
    debug_code = sbpl_arm_planner::RIGHT_ARM_IN_COLLISION;
    return false;
  }
  dist = min(dist_temp,dist);
  ROS_DEBUG("[Right] dist: %d dist_temp: %d\n",int(dist), int(dist_temp));

  //check the left arm against the environment
  if(!checkCollision(langles, pose, 1, verbose, dist_temp))
  {
    if(verbose)
      ROS_DEBUG_NAMED(cspace_log_,"  Left arm is in collision with the environment. (dist: %d)", int(dist_temp));
    dist = dist_temp;
    //code_ = "Left arm + Environment";
    debug_code = sbpl_arm_planner::LEFT_ARM_IN_COLLISION;
    return false;
  }
  dist = min(dist_temp,dist);
  ROS_DEBUG("[Left] dist: %d dist_temp: %d\n",int(dist), int(dist_temp));
 
  //check the arms against each other
  if(!checkCollisionBetweenArms(langles,rangles, pose, verbose, dist_temp))
  {
    if(verbose)
      ROS_DEBUG_NAMED(cspace_log_,"  Arms are in collision with each other. (dist %d)", int(dist_temp));
    dist = dist_temp;
    //code_ = "Left arm + Right arm";
    debug_code = sbpl_arm_planner::COLLISION_BETWEEN_ARMS;
    return false;
  }
  dist = min(dist_temp,dist);
  ROS_DEBUG("[Arms] dist: %d dist_temp: %d\n",int(dist), int(dist_temp));

  return true;
}

bool PR2CollisionSpace::checkCollision(const std::vector<double> &angles, BodyPose &pose, char i_arm, bool verbose, unsigned char &dist)
{
  unsigned char dist_temp=100;
  std::vector<std::vector<int> > jnts;
  //dist = 100; NOTE: We are assuming dist comes in with some value that matters

  //get position of joints in the occupancy grid
  if(!getJointPosesInGrid(angles, pose, i_arm, jnts))
  {
    if(verbose)
      ROS_DEBUG_NAMED(cspace_log_,"  Unable to get poses of the joints to check for collision. [arm: %d]",int(i_arm));
    return false;
  }

  //check bounds
  for(size_t i = 0; i < jnts.size(); ++i)
  {
    if(!grid_->isInBounds(jnts[i][0],jnts[i][1],jnts[i][2]))
    {
      if(verbose)
        ROS_DEBUG_NAMED(cspace_log_,"  End of link %d is out of bounds (%d %d %d). [arm: %d]", int(i), jnts[i][0],jnts[i][1],jnts[i][2],int(i_arm));
      return false;
    }
  }

  //test each line segment for collision
  for(int i = 0; i < int(jnts.size()-1); i++)
  {
    dist_temp = isValidLineSegment(jnts[i], jnts[i+1], arm_[i_arm]->getLinkRadiusCells(i));
    
    //if the line's distance to the nearest obstacle is less than the radius
    if(dist_temp <= arm_[i_arm]->getLinkRadiusCells(i))
    { 
      if(verbose)
        ROS_DEBUG_NAMED(cspace_log_,"  Link %d: {%d %d %d} -> {%d %d %d} with radius %0.2f is in collision. [arm: %d]",i,jnts[i][0],jnts[i][1],jnts[i][2],jnts[i+1][0],jnts[i+1][1],jnts[i+1][2], arm_[i_arm]->getLinkRadius(i), int(i_arm));
      dist = dist_temp;
      return false;
    }

    if(dist_temp < dist)
      dist = dist_temp;
  }

  if(dist_temp < dist)
      dist = dist_temp;

  return true;
}

bool PR2CollisionSpace::checkCollision(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  if(!checkCollisionArms(langles, rangles, pose, verbose, dist, debug_code))
  {
    ROS_DEBUG("[cspace] Arms are in collision.");
    return false;
  }
  if(!isBodyValid(pose.x, pose.y, pose.theta, pose.z, dist))
  {
    ROS_DEBUG("[cspace] Body is in collision.");
    return false;
  }
  if(!checkCollisionArmsToBody(langles, rangles, pose, dist))
  {
    ROS_DEBUG("[cspace] Arms are in collision with body.");
    return false;
  }
  return true;
}

bool PR2CollisionSpace::checkLinkForCollision(const std::vector<double> &angles, BodyPose &pose, char i_arm, int link_num, bool verbose, unsigned char &dist)
{
  std::vector<std::vector<int> > jnts;

  if(link_num >= arm_[i_arm]->num_links_)
  {
    ROS_WARN("[checkLinkInCollision] %d is not a valid link index. There are %d links.", link_num, arm_[i_arm]->num_links_);
    return false;
  }
  
  //get position of joints in the occupancy grid
  if(!getJointPosesInGrid(angles, pose, i_arm, jnts))
    return false;

  //check bounds
  if(!grid_->isInBounds(jnts[link_num][0],jnts[link_num][1],jnts[link_num][2]))
  {
    if(verbose)
      ROS_DEBUG_NAMED(cspace_log_,"End of link %d is out of bounds. (%d %d %d)", link_num, jnts[link_num][0],jnts[link_num][1],jnts[link_num][2]);
    return false;
  }

  //is link in collision?
  dist = isValidLineSegment(jnts[link_num], jnts[link_num+1], arm_[i_arm]->getLinkRadiusCells(link_num));

  //if the line's distance to the nearest obstacle is less than the radius
  if(dist <= arm_[i_arm]->getLinkRadiusCells(link_num))
  {
    if(verbose)
      ROS_DEBUG_NAMED(cspace_log_,"Link %d: {%d %d %d} -> {%d %d %d} with radius %0.2f is in collision.",link_num,jnts[link_num][0],jnts[link_num][1],jnts[link_num][2],jnts[link_num+1][0],jnts[link_num+1][1],jnts[link_num+1][2],arm_[i_arm]->getLinkRadius(link_num));
    return false;
  }

  return true;
}

bool PR2CollisionSpace::checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, BodyPose &pose, char i_arm, bool verbose, unsigned char &dist)
{
  int inc_cc = 10;
  unsigned char dist_temp = 0;
  std::vector<double> start_norm(start);
  std::vector<double> end_norm(end);
  std::vector<std::vector<double> > path;
  //dist = 100; NOTE: We are assuming dist comes in with some value that matters
  
  for(size_t i=0; i < start.size(); i++)
  {
    start_norm[i] = angles::normalize_angle(start[i]);
    end_norm[i] = angles::normalize_angle(end[i]);
  }
 
  //TODO: should read from text file
  if(i_arm == 0)
  {
    if(start[2] > arm_[i_arm]->getMaxJointLimit(2))
      start_norm[2] = start[2] + (2*-M_PI);

    if(end[2] > arm_[i_arm]->getMaxJointLimit(2))
      end_norm[2] = end[2] + (2*-M_PI);
  }
  else
  {
    if(start[2] < arm_[i_arm]->getMinJointLimit(2))
      start_norm[2] = start[2] + (2*M_PI);

    if(end[2] < arm_[i_arm]->getMinJointLimit(2))
      end_norm[2] = end[2] + (2*M_PI);
  }

  getInterpolatedPath(start_norm, end_norm, inc_, path);

  // optimization: try to find collisions that might come later in the path earlier
  if(int(path.size()) > inc_cc)
  {
    for(int i = 0; i < inc_cc; i++)
    {
      for(size_t j = i; j < path.size(); j=j+inc_cc)
      {
        if(!checkCollision(path[j], pose, i_arm, verbose, dist_temp))
        {
          dist = dist_temp;
          return false; 
        }

        if(dist_temp < dist)
          dist = dist_temp;
      }
    }
  }
  else
  {
    for(size_t i = 0; i < path.size(); i++)
    {
      if(!checkCollision(path[i], pose, i_arm, verbose, dist_temp))
      {
        dist = dist_temp;
        return false;
      }

      if(dist_temp < dist)
        dist = dist_temp;
    }
  }

  return true;
}

bool PR2CollisionSpace::checkPathForCollision(const std::vector<double> &start0, const std::vector<double> &end0, const std::vector<double> &start1, const std::vector<double> &end1, BodyPose &pose, bool verbose, unsigned char &dist)
{
  int debug_code;
  return checkPathForCollision(start0,end0,start1,end1,pose,verbose,dist,debug_code);
}

bool PR2CollisionSpace::checkPathForCollision(const std::vector<double> &start0, const std::vector<double> &end0, const std::vector<double> &start1, const std::vector<double> &end1, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  int inc_cc = 10;
  unsigned char dist_temp = 0;
  std::vector<double> start0_norm(start0), start1_norm(start1), end0_norm(end0), end1_norm(end1);
  std::vector<std::vector<double> > path0, path1;
  //dist = 100;  NOTE: We are assuming dist comes in with some value that matters

  for(size_t i=0; i < start0.size(); i++)
  {
    start0_norm[i] = angles::normalize_angle(start0[i]);
    end0_norm[i] = angles::normalize_angle(end0[i]);
    start1_norm[i] = angles::normalize_angle(start1[i]);
    end1_norm[i] = angles::normalize_angle(end1[i]);
  }

  //right arm - upper_arm roll
  if(start0[2] > arm_[0]->getMaxJointLimit(2))
    start0_norm[2] = start0[2] + (2*-M_PI);
  if(end0[2] > arm_[0]->getMaxJointLimit(2))
    end0_norm[2] = end0[2] + (2*-M_PI);

  //left arm - upper_arm roll
  if(start1[2] < arm_[1]->getMinJointLimit(2))
    start1_norm[2] = start1[2] + (2*M_PI);
  if(end1[2] < arm_[1]->getMinJointLimit(2))
    end1_norm[2] = end1[2] + (2*M_PI);

  getInterpolatedPath(start0_norm, end0_norm, inc_, path0);
  getFixedLengthInterpolatedPath(start1_norm, end1_norm, path0.size(), path1);

  // optimization: try to find collisions that might come later in the path earlier
  if(int(path0.size()) > inc_cc)
  {
    for(int i = 0; i < inc_cc; i++)
    {
      for(size_t j = i; j < path0.size(); j=j+inc_cc)
      {
        if(!checkCollision(path0[j], path1[j], pose, verbose, dist_temp, debug_code))
        {
          dist = dist_temp;
          return false; 
        }

        if(dist_temp < dist)
          dist = dist_temp;
      }
    }
  }
  else
  {
    for(size_t i = 0; i < path0.size(); i++)
    {
      if(!checkCollision(path0[i], path1[i], pose, verbose, dist_temp, debug_code))
      {
        dist = dist_temp;
        return false;
      }

      if(dist_temp < dist)
        dist = dist_temp;
    }
  }
  return true;
}

bool PR2CollisionSpace::checkLinkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, BodyPose &pose, char i_arm, int link_num, bool verbose, unsigned char &dist)
{
  int inc_cc = 10;
  unsigned char dist_temp = 0;
  std::vector<double> start_norm(start);
  std::vector<double> end_norm(end);
  std::vector<std::vector<double> > path;
  //dist = 100;  NOTE: We are assuming dist comes in with some value that matters

  for(size_t i=0; i < start.size(); i++)
  {
    start_norm[i] = angles::normalize_angle(start[i]);
    end_norm[i] = angles::normalize_angle(end[i]);
  }
 
  //problem with upper_arm_roll
  //TODO: Do this more gracefully?
  if(i_arm == 0)
  {
    if(start[2] > arm_[i_arm]->getMaxJointLimit(2))
      start_norm[2] = start[2] + (2*-M_PI);

    if(end[2] > arm_[i_arm]->getMaxJointLimit(2))
      end_norm[2] = end[2] + (2*-M_PI);
  }
  else
  {
    if(start[2] < arm_[i_arm]->getMinJointLimit(2))
      start_norm[2] = start[2] + (2*M_PI);

    if(end[2] < arm_[i_arm]->getMinJointLimit(2))
      end_norm[2] = end[2] + (2*M_PI);
  }

  getInterpolatedPath(start_norm, end_norm, inc_, path);

  //try to find collisions that might come later in the path earlier
  if(int(path.size()) > inc_cc)
  {
    for(int i = 0; i < inc_cc; i++)
    {
      for(size_t j = i; j < path.size(); j=j+inc_cc)
      {
        if(!checkLinkForCollision(path[j], pose, i_arm, link_num, verbose, dist_temp))
        {
          dist = dist_temp;
          return false; 
        }

        if(dist_temp < dist)
          dist = dist_temp;
      }
    }
  }
  else
  {
    for(size_t i = 0; i < path.size(); i++)
    {
      if(!checkLinkForCollision(path[i], pose, i_arm, link_num, verbose, dist_temp))
      {
        dist = dist_temp;
        return false;
      }

      if(dist_temp < dist)
        dist = dist_temp;
    }
  }

  return true;
}

bool PR2CollisionSpace::getJointPosesInGrid(const std::vector<double> angles, BodyPose &pose, char i_arm, std::vector<std::vector<int> > &jnts)
{
  std::vector<std::vector<double> > jnts_m;

  if(!arm_[i_arm]->getJointPositions(angles, pose, jnts_m, frame_))
    return false;

  // replace r_finger_tip_link 6/15/11
  KDL::Vector tip_f_ref, tip_f_wrist(0.16,0,0);
  tip_f_ref = frame_.M * tip_f_wrist + frame_.p;
  //jnts_m[3][0] = tip_f_ref.x();
  //jnts_m[3][1] = tip_f_ref.y();
  //jnts_m[3][2] = tip_f_ref.z();

  jnts_m.back()[0] = tip_f_ref.x();
  jnts_m.back()[1] = tip_f_ref.y();
  jnts_m.back()[2] = tip_f_ref.z();

  jnts.resize(jnts_m.size());
  for(size_t i = 0; i < jnts.size(); ++i)
  {
    jnts[i].resize(3);
    grid_->worldToGrid(jnts_m[i][0],jnts_m[i][1],jnts_m[i][2],jnts[i][0],jnts[i][1],jnts[i][2]);
  }

  return true;
}

unsigned char PR2CollisionSpace::isValidLineSegment(const std::vector<int> a, const std::vector<int> b, const short unsigned int radius)
{
  leatherman::bresenham3d_param_t params;
  int nXYZ[3], retvalue = 1;
  unsigned char cell_val, min_dist = 255;
  CELL3V tempcell;
  vector<CELL3V>* pTestedCells=NULL;

  //iterate through the points on the segment
  leatherman::get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
  do {
    leatherman::get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

    if(!grid_->isInBounds(nXYZ[0],nXYZ[1],nXYZ[2]))
      return 0;

    cell_val = grid_->getCell(nXYZ[0],nXYZ[1],nXYZ[2]);
    if(cell_val <= radius)
    {
      if(pTestedCells == NULL)
        return cell_val;   //return 0
      else
        retvalue = 0;
    }

    if(cell_val < min_dist)
      min_dist = cell_val;

    //insert the tested point
    if(pTestedCells)
    {
      if(cell_val <= radius)
        tempcell.bIsObstacle = true;
      else
        tempcell.bIsObstacle = false;
      tempcell.x = nXYZ[0];
      tempcell.y = nXYZ[1];
      tempcell.z = nXYZ[2];
      pTestedCells->push_back(tempcell);
    }
  } while (leatherman::get_next_point3d(&params));

  if(retvalue)
    return min_dist;
  else
    return 0;
}

double PR2CollisionSpace::distanceBetween3DLineSegments(std::vector<int> l1a, std::vector<int> l1b,std::vector<int> l2a, std::vector<int> l2b)
{
  // Copyright 2001, softSurfer (www.softsurfer.com)
  // This code may be freely used and modified for any purpose
  // providing that this copyright notice is included with it.
  // SoftSurfer makes no warranty for this code, and cannot be held
  // liable for any real or imagined damage resulting from its use.
  // Users of this code must verify correctness for their application.

  double u[3];
  double v[3];
  double w[3];
  double dP[3];

  u[0] = l1b[0] - l1a[0];
  u[1] = l1b[1] - l1a[1];
  u[2] = l1b[2] - l1a[2];

  v[0] = l2b[0] - l2a[0];
  v[1] = l2b[1] - l2a[1];
  v[2] = l2b[2] - l2a[2];

  w[0] = l1a[0] - l2a[0];
  w[1] = l1a[1] - l2a[1];
  w[2] = l1a[2] - l2a[2];

  double a = u[0] * u[0] + u[1] * u[1] + u[2] * u[2]; // always >= 0
  double b = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; // dot(u,v);
  double c = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; // dot(v,v);        // always >= 0
  double d = u[0] * w[0] + u[1] * w[1] + u[2] * w[2]; // dot(u,w);
  double e = v[0] * w[0] + v[1] * w[1] + v[2] * w[2]; // dot(v,w);
  double D = a*c - b*b;       // always >= 0
  double sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
  double tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

  // compute the line parameters of the two closest points
  if (D < SMALL_NUM) { // the lines are almost parallel
    sN = 0.0;        // force using point P0 on segment S1
    sD = 1.0;        // to prevent possible division by 0.0 later
    tN = e;
    tD = c;
  }
  else {                // get the closest points on the infinite lines
    sN = (b*e - c*d);
    tN = (a*e - b*d);
    if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
      sN = 0.0;
      tN = e;
      tD = c;
    }
    else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
    tN = 0.0;
    // recompute sc for this edge
    if (-d < 0.0)
      sN = 0.0;
    else if (-d > a)
      sN = sD;
    else {
      sN = -d;
      sD = a;
    }
  }
  else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
    tN = tD;
    // recompute sc for this edge
    if ((-d + b) < 0.0)
      sN = 0;
    else if ((-d + b) > a)
      sN = sD;
    else {
      sN = (-d + b);
      sD = a;
    }
  }

  // finally do the division to get sc and tc
  sc = (fabs(sN) < SMALL_NUM ? 0.0 : sN / sD);
  tc = (fabs(tN) < SMALL_NUM ? 0.0 : tN / tD);

  // get the difference of the two closest points
  // dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)

  dP[0] = w[0] + (sc * u[0]) - (tc * v[0]);
  dP[1] = w[1] + (sc * u[1]) - (tc * v[1]);
  dP[2] = w[2] + (sc * u[2]) - (tc * v[2]);

  return  sqrt(dP[0]*dP[0] + dP[1]*dP[1] + dP[2]*dP[2]);   // return the closest distance
}

void PR2CollisionSpace::addArmCuboidsToGrid(char i_arm)
{
  std::vector<std::vector<double> > cuboids = arm_[i_arm]->getCollisionCuboids();

  ROS_DEBUG("[PR2CollisionSpace] received %d cuboids\n",int(cuboids.size()));

  for(unsigned int i = 0; i < cuboids.size(); i++)
  {
    if(cuboids[i].size() == 6)
      grid_->addCube(cuboids[i][0],cuboids[i][1],cuboids[i][2],cuboids[i][3],cuboids[i][4],cuboids[i][5]);
    else
      ROS_DEBUG("[addArmCuboidsToGrid] Self-collision cuboid #%d has an incomplete description.\n", i);
  }
}
/*
bool PR2CollisionSpace::getCollisionCylinders(const std::vector<double> &angles, BodyPose &pose, char i_arm, std::vector<std::vector<double> > &cylinders)
{
  int num_arm_spheres = 0;
  std::vector<double> xyzr(4,0);
  std::vector<std::vector<int> > points, jnts;

  //get position of joints in the occupancy grid
  if(!getJointPosesInGrid(angles, pose, i_arm, jnts))
    return false;

  for(int i = 0; i < int(jnts.size()-1); ++i)
  {
    points.clear();
    getLineSegment(jnts[i], jnts[i+1], points);

    //write points to cylinder {x,y,z,radius} all in centimeters
    for(int j = 0; j < int(points.size()); ++j)
    {
      grid_->gridToWorld(points[j][0],points[j][1],points[j][2],xyzr[0],xyzr[1],xyzr[2]); 
      xyzr[3]=double(arm_[i_arm]->getLinkRadiusCells(i))*arm_[i_arm]->resolution_;
      cylinders.push_back(xyzr);
    }
  }

  num_arm_spheres = cylinders.size();
  return true;
}
*/

void PR2CollisionSpace::getLineSegment(const std::vector<int> a,const std::vector<int> b,std::vector<std::vector<int> > &points){
  leatherman::bresenham3d_param_t params;
  std::vector<int> nXYZ(3,0);

  //iterate through the points on the segment
  leatherman::get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
  do {
    leatherman::get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

    points.push_back(nXYZ);

  } while (leatherman::get_next_point3d(&params));
}

bool PR2CollisionSpace::checkCollisionBetweenArms(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist)
{
  double d = 100, d_min = 100;
  std::vector<std::vector<int> > ljnts, rjnts;

  //get position of joints in the occupancy grid
  if(!getJointPosesInGrid(rangles, pose, 0, rjnts))
    return false;
  if(!getJointPosesInGrid(langles, pose, 1, ljnts))
    return false;

  //measure distance between links of each arms
  ROS_DEBUG_NAMED(cspace_log_,"right arm joints: %d  left arm joints: %d",int(rjnts.size()), int(ljnts.size()));

  for(size_t i = 0; i < rjnts.size()-1; ++i)
  {
    for(size_t j = 0; j < ljnts.size()-1; ++j)
    {
      d = distanceBetween3DLineSegments(rjnts[i], rjnts[i+1], ljnts[j], ljnts[j+1]);
      if(verbose)
      {
        ROS_DEBUG_NAMED(cspace_log_,"Right %d: %d %d %d -> %d %d %d  dist: %0.3f", int(i), rjnts[i][0], rjnts[i][1], rjnts[i][2], rjnts[i+1][0],rjnts[i+1][1],rjnts[i+1][2],d);
        ROS_DEBUG_NAMED(cspace_log_,"Left  %d: %d %d %d -> %d %d %d  dist: %0.3f", int(j), ljnts[j][0], ljnts[j][1], ljnts[j][2], ljnts[j+1][0],ljnts[j+1][1],ljnts[j+1][2],d);
      }
      if(d <= max(arm_[0]->getLinkRadiusCells(i), arm_[1]->getLinkRadiusCells(j)))
      {
        if(verbose)
          ROS_DEBUG_NAMED(cspace_log_,"  Right arm link %d is in collision with left arm link %d. (dist: %0.3fm)", int(i), int(j), d*arm_[0]->resolution_);
        dist = d;
        return false;
      }
      /*
      else
        if(verbose)
          ROS_INFO("Distance between right link %d and left link %d is %0.3fm", int(i), int(j), d*arm_[0]->resolution_);
      */

      if(d < d_min)
        d_min = d; //min(d_min, d);
    }
  }
  dist = d_min;
  return true;
}

void PR2CollisionSpace::getInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, double inc, std::vector<std::vector<double> > &path)
{
  bool changed = true; 
  std::vector<double> next(start);
  
  //check if both input configurations have same size
  if(start.size() != end.size())
  {
    ROS_WARN("[getInterpolatedPath] The start and end configurations have different sizes.\n");
    return;
  }

  while(changed)
  {
    changed = false;

    for (int i = 0; i < int(start.size()); i++) 
    {
      if (fabs(next[i] - end[i]) > inc) 
      {
        changed = true;
        
        if(end[i] > next[i]) 
          next[i] += inc;
        else
          next[i] += -inc;
      }
    }

    if (changed)
      path.push_back(next);
  }
}

void PR2CollisionSpace::getInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, std::vector<double> &inc, std::vector<std::vector<double> > &path)
{
  bool changed = true; 
  std::vector<double> next(start);
  
  //check if both input configurations have same size
  if(start.size() != end.size())
  {
    ROS_WARN("[getInterpolatedPath] The start and end configurations have different sizes.\n");
    return;
  }

  while(changed)
  {
    changed = false;

    for (int i = 0; i < int(start.size()); i++) 
    {
      if (fabs(next[i] - end[i]) > inc[i]) 
      {
        changed = true;
        
        if(end[i] > next[i]) 
          next[i] += inc[i];
        else
          next[i] += -inc[i];
      }
    }

    if (changed)
      path.push_back(next);
  }
}

void PR2CollisionSpace::getFixedLengthInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, int path_length, std::vector<std::vector<double> > &path)
{
  std::vector<double> next(start), inc(start.size(),0);
  path.clear();

  //check if both input configurations have same size
  if(start.size() != end.size())
  {
    ROS_WARN("[getInterpolatedPath] The start and end configurations have different sizes.\n");
    return;
  }

  //compute the increments from start to end configurations
  for(size_t i = 0; i < start.size(); ++i)
  {
    if((start[i]-end[i]) == 0.0)
      inc[i] = 0.0;
    else
      inc[i] = (start[i]-end[i])/path_length;
  }

  for(int i = 0; i < path_length-1; ++i)
  {
    for(int j = 0; j < int(start.size()); ++j)
      next[j] += inc[j];

    path.push_back(next);
  }

  path.push_back(end);
}

void PR2CollisionSpace::processCollisionObjectMsg(const arm_navigation_msgs::CollisionObject &object)
{
  if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD)
  {
    object_map_[object.id] = object;
    addCollisionObject(object);
  }
  else if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
  {
    if(object.id.compare("all") == 0)
      removeAllCollisionObjects();
    else
      removeCollisionObject(object);
  }
  else if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT)
  {
    object_map_[object.id] = object;
    addCollisionObject(object);
  }
  else if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT)
  {
    //TODO: Attach to gripper
    removeCollisionObject(object);
  }
  else
    ROS_WARN("*** Operation isn't supported. ***\n\n");
}

void PR2CollisionSpace::addCollisionObject(const arm_navigation_msgs::CollisionObject &object)
{
  geometry_msgs::Pose pose;

  for(size_t i = 0; i < object.shapes.size(); ++i)
  {
    if(object.shapes[i].type == arm_navigation_msgs::Shape::BOX)
    {
      transformPose(object.header.frame_id, grid_->getReferenceFrame(), object.poses[i], pose);
      object_voxel_map_[object.id].clear();
      //grid_->getVoxelsInBox(pose, object.shapes[i].dimensions, object_voxel_map_[object.id]);

      ROS_DEBUG_NAMED(cspace_log_,"[%s] TransformPose from %s to %s.", object.id.c_str(), object.header.frame_id.c_str(), grid_->getReferenceFrame().c_str());
      ROS_DEBUG_NAMED(cspace_log_,"[%s] %s: xyz: %0.3f %0.3f %0.3f   quat: %0.3f %0.3f %0.3f %0.3f", object.id.c_str(), object.header.frame_id.c_str(), object.poses[i].position.x,  object.poses[i].position.y, object.poses[i].position.z, object.poses[i].orientation.x, object.poses[i].orientation.y, object.poses[i].orientation.z,object.poses[i].orientation.w);
      ROS_DEBUG_NAMED(cspace_log_,"[%s] %s: xyz: %0.3f %0.3f %0.3f   quat: %0.3f %0.3f %0.3f %0.3f", object.id.c_str(), grid_->getReferenceFrame().c_str(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      ROS_DEBUG_NAMED(cspace_log_,"[%s] occupies %d voxels.",object.id.c_str(), int(object_voxel_map_[object.id].size()));
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::MESH)
    {
      transformPose(object.header.frame_id, grid_->getReferenceFrame(), object.poses[i], pose);
      object_voxel_map_[object.id].clear();
      //sbpl::Voxelizer::voxelizeMesh(object.shapes[i].vertices, object.shapes[i].triangles, 0.02, object_voxel_map_[object.id], true);

      // transform voxels in mesh frame into the world frame
      btVector3 v(pose.position.x, pose.position.y, pose.position.z);
      btMatrix3x3 m(btQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
      //btTransform trans = Transform(Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), Vector3(pose.position.x, pose.position.y, pose.position.z));
      for(size_t j = 0; j <  object_voxel_map_[object.id].size(); ++j)
      {
        object_voxel_map_[object.id][j] = m *  object_voxel_map_[object.id][j];
        object_voxel_map_[object.id][j] += v;
      }

      ROS_DEBUG_NAMED(cspace_log_,"[%s] TransformPose from %s to %s.", object.id.c_str(), object.header.frame_id.c_str(), grid_->getReferenceFrame().c_str());
      ROS_DEBUG_NAMED(cspace_log_,"[%s] %s: xyz: %0.3f %0.3f %0.3f   quat: %0.3f %0.3f %0.3f %0.3f", object.id.c_str(), object.header.frame_id.c_str(), object.poses[i].position.x,  object.poses[i].position.y, object.poses[i].position.z, object.poses[i].orientation.x, object.poses[i].orientation.y, object.poses[i].orientation.z,object.poses[i].orientation.w);
      ROS_DEBUG_NAMED(cspace_log_,"[%s] %s: xyz: %0.3f %0.3f %0.3f   quat: %0.3f %0.3f %0.3f %0.3f", object.id.c_str(), grid_->getReferenceFrame().c_str(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      ROS_DEBUG_NAMED(cspace_log_,"[%s] occupies %d voxels.",object.id.c_str(), int(object_voxel_map_[object.id].size()));
    }

    else
      ROS_WARN("[cspace] Collision objects of type %d are not yet supported.", object.shapes[i].type);
  }

  // add this object to list of objects that get added to grid
  bool new_object = true;
  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    if(known_objects_[i].compare(object.id) == 0)
      new_object = false;
  }
  if(new_object)
    known_objects_.push_back(object.id);

  grid_->addPointsToField(btVectorToEigenVector(object_voxel_map_[object.id]));
  ROS_INFO("[cspace] Just added %s to the distance field, represented as %d voxels.", object.id.c_str(), int(object_voxel_map_[object.id].size()));
}

void PR2CollisionSpace::getCollisionObjectVoxelPoses(std::vector<geometry_msgs::Pose> &points)
{
  geometry_msgs::Pose pose;

  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    for(size_t j = 0; j < object_voxel_map_[known_objects_[i]].size(); ++j)
    {
      pose.position.x = object_voxel_map_[known_objects_[i]][j].x();
      pose.position.y = object_voxel_map_[known_objects_[i]][j].y();
      pose.position.z = object_voxel_map_[known_objects_[i]][j].z();
      points.push_back(pose);
    }
  }
}

void PR2CollisionSpace::removeCollisionObject(const arm_navigation_msgs::CollisionObject &object)
{
  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    if(known_objects_[i].compare(object.id) == 0)
    {
      known_objects_.erase(known_objects_.begin() + i);
      object_voxel_map_[object.id].clear();
      ROS_DEBUG("[cspace] Removing %s from list of known objects.", object.id.c_str());
    }
  }
  // reset the map, add all the known objects again
  grid_->updateFromCollisionMap(last_collision_map_);
  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    grid_->addPointsToField(btVectorToEigenVector(object_voxel_map_[known_objects_[i]]));
  }
}

void PR2CollisionSpace::removeAllCollisionObjects()
{
  known_objects_.clear();
}

void PR2CollisionSpace::putCollisionObjectsInGrid()
{
  ROS_DEBUG("[putCollisionObjectsInGrid] Should we reset first?");

  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    grid_->addPointsToField(btVectorToEigenVector(object_voxel_map_[known_objects_[i]]));
    ROS_DEBUG("[putCollisionObjectsInGrid] Added %s to grid with %d voxels.",known_objects_[i].c_str(), int(object_voxel_map_[known_objects_[i]].size()));
  }
}

void PR2CollisionSpace::transformPose(const std::string &current_frame, const std::string &desired_frame, const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out)
{
  geometry_msgs::PoseStamped stpose_in, stpose_out;
  stpose_in.header.frame_id = current_frame;
  stpose_in.header.stamp = ros::Time();
  stpose_in.pose = pose_in;
  tf_.transformPose(desired_frame, stpose_in, stpose_out);
  pose_out = stpose_out.pose;
}

/*
void PR2CollisionSpace::setAttachedRobotLinkToMultiDofTransform(KDL::Frame& transform)
{
	attached_robot_link_in_multi_dof_ = transform;
	if (is_object_attached_) attached_object_in_multi_dof_ = attached_robot_link_in_multi_dof_ * attached_object_pose_;
	//attached_object_in_multi_dof_ = attached_object_pose_ * attached_robot_link_in_multi_dof_;
	  // TODO: possibly add these in to sbpl_arm_planner. They're in Ben's version
//	sbpl_arm_planner::printKDLFrame(attached_object_pose_, "object_in_right_wrist");
//	sbpl_arm_planner::printKDLFrame(attached_robot_link_in_multi_dof_, "right_wrist_in_multi-dof");
//	sbpl_arm_planner::printKDLFrame(attached_object_in_multi_dof_, "object_in_multi-dof");
}
*/

bool PR2CollisionSpace::getCollisionLinks()
{
  XmlRpc::XmlRpcValue xml_links;
  CollisionLink cl;
  cl_.clear();
  std::string links_name = "collision_links";

  ros::NodeHandle ph("~");
  if(!ph.hasParam(links_name)) 
  {
    ROS_WARN_STREAM("No groups for planning specified in " << links_name);
    return false;
  }
  ph.getParam(links_name, xml_links);

  if(xml_links.getType() != XmlRpc::XmlRpcValue::TypeArray) 
  {
    ROS_WARN("'Collision Links' is not an array.");
    return false;
  }

  if(xml_links.size() == 0)
  {
    ROS_WARN("'Collision Links' is empty.");
    return false;
  }

  for(int i = 0; i < xml_links.size(); i++) 
  {
    if(xml_links[i].hasMember("name"))
      cl.name = std::string(xml_links[i]["name"]);
    else
      cl.name = "link_" + boost::lexical_cast<std::string>(i);

    if(xml_links[i].hasMember("x1") && xml_links[i].hasMember("y1") && xml_links[i].hasMember("z1") &&
       xml_links[i].hasMember("x2") && xml_links[i].hasMember("y2") && xml_links[i].hasMember("z2"))
    {
      cl.x1 = xml_links[i]["x1"];
      cl.y1 = xml_links[i]["y1"];
      cl.z1 = xml_links[i]["z1"];
      cl.x2 = xml_links[i]["x2"];
      cl.y2 = xml_links[i]["y2"];
      cl.z2 = xml_links[i]["z2"]; 
      cl.p1 = KDL::Vector(double(xml_links[i]["x1"]),double(xml_links[i]["y1"]),double(xml_links[i]["z1"]));
      cl.p2 = KDL::Vector(double(xml_links[i]["x2"]),double(xml_links[i]["y2"]),double(xml_links[i]["z2"]));
    }
    else
    {
      ROS_ERROR("Collision link '%s' is missing one of the coordinates.", cl.name.c_str());
      return false;
    }

    if(xml_links[i].hasMember("frame"))
      cl.frame = std::string(xml_links[i]["frame"]);
    else
    {
      ROS_WARN("Collision link '%s' is missing the reference frame.", cl.name.c_str());
      return false;
    }

    if(xml_links[i].hasMember("radius"))
      cl.radius = xml_links[i]["radius"];
    else
    {
      ROS_WARN("Description of collision link '%s' is missing the radius.", cl.name.c_str());
      return false;
    }
 
    if(xml_links[i].hasMember("priority"))
      cl.priority = xml_links[i]["priority"];
    else
      cl.priority = 1;
 
    if(xml_links[i].hasMember("group"))
      cl.group = std::string(xml_links[i]["group"]);
    else
    {
      ROS_WARN("Description of collision link '%s' is missing the group.", cl.name.c_str());
      return false;
    }
  
    cl_.push_back(cl);
    if(cl.group.compare("base") == 0)
      basel_.push_back(cl);
    else if(cl.group.compare("torso") == 0)
      torsol_.push_back(cl);
    else if(cl.group.compare("head") == 0)
      headl_.push_back(cl);
    else
    {
      ROS_WARN("%s link is in the %s group and it isn't supported. Right now, only base, torso, head groups are supported.", cl.name.c_str(),cl.group.c_str());
    }
  }
  ROS_INFO("Successfully parsed list of %d collision links.", int(cl_.size()));
  return true;
}

void PR2CollisionSpace::printCollisionLinks()
{
  for(size_t i = 0; i < cl_.size(); ++i)
  {
    ROS_INFO("-------------------------------------");
    ROS_INFO("%d: %s", int(i), cl_[i].name.c_str());
    ROS_INFO("x1: %0.3f  y1: %0.3f  z1: %0.3f", cl_[i].x1, cl_[i].y1, cl_[i].z1);
    ROS_INFO("x2: %0.3f  y2: %0.3f  z2: %0.3f", cl_[i].x2, cl_[i].y2, cl_[i].z2);
    ROS_INFO("radius: %0.3fm", cl_[i].radius);
    ROS_INFO("priority: %d", cl_[i].priority);
    ROS_INFO("frame: %s", cl_[i].frame.c_str());
    if(i == cl_.size()-1)
      ROS_INFO("-------------------------------------");
  }
}

bool PR2CollisionSpace::getSphereGroups()
{
  XmlRpc::XmlRpcValue all_groups;
  Sphere s;
  Group g;
  std::string groups_name = "groups";

  ros::NodeHandle ph("~");
  ph.param<std::string>("full_body_kinematics_chain/root_frame", full_body_chain_root_name_, "base_footprint");
  ph.param<std::string>("full_body_kinematics_chain/tip_frame", full_body_chain_tip_name_, "head_tilt_link");

  if(!initFullBodyKinematics())
  {
    ROS_ERROR("[cspace] Failed to initialize the full body kinematics chain.");
    return false;
  }

  if(!ph.hasParam(groups_name))
  {
    ROS_WARN_STREAM("No groups for planning specified in " << groups_name);
    return false;
  }
  ph.getParam(groups_name, all_groups);

  if(all_groups.getType() != XmlRpc::XmlRpcValue::TypeArray) 
  {
    ROS_WARN("Groups is not an array.");
    return false;
  }

  if(all_groups.size() == 0) 
  {
    ROS_WARN("No groups in groups");
    return false;
  }

  for(int i = 0; i < all_groups.size(); i++) 
  {
    if(!all_groups[i].hasMember("name"))
    {
      ROS_WARN("All groups must have a name.");
      return false;
    } 
    g.name = std::string(all_groups[i]["name"]);

    if(!all_groups[i].hasMember("frame"))
    {
      ROS_WARN("All groups must have a frame.");
      return false;
    }
    g.root_frame = std::string(all_groups[i]["frame"]);

    for(size_t k = 0; k < full_body_chain_.getNrOfSegments(); ++k)
    {
      if(full_body_chain_.getSegment(k).getName().compare(g.root_frame) == 0)
      {
        g.kdl_segment = k + 1;
        ROS_INFO("[cspace] %s group is rooted at %s with kdl segment #%d", g.name.c_str(), g.root_frame.c_str(), int(k));
        break;
      }
    }

    if((g.name.compare("right_gripper") == 0) || (g.name.compare("left_gripper") == 0))
      g.kdl_segment = 10;
    else if((g.name.compare("right_forearm") == 0) || (g.name.compare("left_forearm") == 0))
      g.kdl_segment = 7;

    std::stringstream ss(all_groups[i]["spheres"]);
    double x,y,z;
    g.spheres.clear();
    std::string sphere_name;
    while(ss >> sphere_name)
    {
      ros::NodeHandle s_nh(ph, sphere_name);
      s.name = sphere_name;
      s_nh.param("x", x, 0.0);
      s_nh.param("y", y, 0.0);
      s_nh.param("z", z, 0.0);
      s_nh.param("radius", s.radius, 0.0);
      s_nh.param("priority", s.priority, 1);
      s.v.x(x);
      s.v.y(y);
      s.v.z(z);
      s.radius_c = s.radius / arm_[0]->resolution_ + 0.5;
      g.spheres.push_back(s);
    }
    
    if(g.name.compare("base") == 0)
      base_g_ = g;
    else if(g.name.compare("torso_upper") == 0)
      torso_upper_g_ = g;
    else if(g.name.compare("torso_lower") == 0)
      torso_lower_g_ = g;
    else if(g.name.compare("turrets") == 0)
      turrets_g_ = g;
    else if(g.name.compare("tilt_laser") == 0)
      tilt_laser_g_ = g;
    else if(g.name.compare("head") == 0)
      head_g_ = g;
    else if(g.name.compare("left_gripper") == 0)
      lgripper_g_ = g;
    else if(g.name.compare("right_gripper") == 0)
      rgripper_g_ = g;
    else if(g.name.compare("left_forearm") == 0)
      lforearm_g_ = g;
    else if(g.name.compare("right_forearm") == 0)
      rforearm_g_ = g;
    else
    {
      ROS_ERROR("The list of spheres contains a group with an unrecognized name, '%s'. Temporarily, only specific names are supported.Exiting.", g.name.c_str());
      return false;
    }
  }

  // make a master list of sphere groups
  all_g_.push_back(base_g_);
  all_g_.push_back(torso_upper_g_);
  all_g_.push_back(torso_lower_g_);
  all_g_.push_back(turrets_g_);
  all_g_.push_back(tilt_laser_g_);
  all_g_.push_back(rgripper_g_);
  all_g_.push_back(lgripper_g_);
  all_g_.push_back(rforearm_g_);
  all_g_.push_back(lforearm_g_);
  all_g_.push_back(head_g_);
  
  ROS_INFO("Successfully parsed collision groups.");
  return true;
}

void PR2CollisionSpace::printSphereGroups()
{
  for(size_t i = 0; i < all_g_.size(); ++i)
  {
    ROS_INFO("----------------[%d]-------------------",int(i));
    ROS_INFO("group: %s", all_g_[i].name.c_str());
    ROS_INFO("frame: %s", all_g_[i].root_frame.c_str());

    if(all_g_[i].spheres.size() == 0)
      ROS_WARN("(no spheres)"); 
    for(size_t j = 0; j < all_g_[i].spheres.size(); ++j)
      ROS_INFO("[%s] x: %0.3f  y: %0.3f  z: %0.3f  radius: %0.3fm  priority: %d", all_g_[i].spheres[j].name.c_str(), all_g_[i].spheres[j].v.x(), all_g_[i].spheres[j].v.y(), all_g_[i].spheres[j].v.z(), all_g_[i].spheres[j].radius, all_g_[i].spheres[j].priority);
    
    if(i == all_g_.size()-1)
      ROS_INFO("-------------------------------------");   
  }
}

bool PR2CollisionSpace::initFullBodyKinematics()
{
  ros::NodeHandle nh("~");
  std::string robot_description;
  std::string robot_param;

  nh.searchParam("robot_description",robot_param);
  nh.param<std::string>(robot_param,robot_description,"");

  if(robot_description.empty())
  {
    ROS_ERROR("[cspace] Unable to get robot_description from param server.");
    return false;
  }

  if(!kdl_parser::treeFromString(robot_description, full_body_tree_))
  {
    ROS_ERROR("[cspace] Failed to parse tree from robot_description on param server.");
    return false;;
  }

  if(!full_body_tree_.getChain(full_body_chain_root_name_, full_body_chain_tip_name_, full_body_chain_))
  {
    ROS_ERROR("[cspace] Failed to fetch the KDL chain for the desired robot frames. Exiting.");
    return false;
  }

  fk_solver_ = new KDL::ChainFkSolverPos_recursive(full_body_chain_);
  jnt_array_.resize(full_body_chain_.getNrOfJoints());

  ROS_INFO("[cspace] The full body kinematics chain has %d segments & %d joints.", full_body_chain_.getNrOfSegments(), full_body_chain_.getNrOfJoints());
  ROS_INFO("[cspace] root: %s tip: %s.", full_body_chain_root_name_.c_str(), full_body_chain_tip_name_.c_str());

  printKDLChain("full body chain", full_body_chain_);
  return true;
}

void PR2CollisionSpace::printKDLChain(std::string name, KDL::Chain &chain)
{
  ROS_INFO("chain: %s", name.c_str());
  for(unsigned int j = 0; j < chain.getNrOfSegments(); ++j)
  {
    ROS_INFO("[%2d] segment: % 0.3f % 0.3f %0.3f joint: % 0.3f % 0.3f % 0.3f type: %9s name: %21s",j,
        chain.getSegment(j).pose(0).p.x(),
        chain.getSegment(j).pose(0).p.y(),
        chain.getSegment(j).pose(0).p.z(),
        chain.getSegment(j).getJoint().pose(0).p.x(),
        chain.getSegment(j).getJoint().pose(0).p.y(),
        chain.getSegment(j).getJoint().pose(0).p.z(),
        chain.getSegment(j).getJoint().getTypeName().c_str(),
        chain.getSegment(j).getJoint().getName().c_str());
  }
  ROS_INFO(" ");
}

void PR2CollisionSpace::setNonPlanningJointPosition(std::string name, double value)
{
  if(name.compare("head_tilt_joint") == 0)
    head_tilt_angle_ = value;
  else if(name.compare("head_pan_joint") == 0)
    head_pan_angle_ = value;
  else
    ROS_ERROR("[cspace] The %s is not a known non-planning joint.", name.c_str());
}

bool PR2CollisionSpace::computeFullBodyKinematics(double x, double y, double theta, double torso, int frame_num, KDL::Frame &fk_out)
{
  jnt_array_(0) = torso; // fill in torso height
  //TODO: Fill in head pan and tilt angles to be extra fancy

  if(fk_solver_->JntToCart(jnt_array_, fk_out, frame_num) < 0)
  {
    ROS_ERROR("JntToCart returned < 0. Exiting.");
    return false;
  }

  getMaptoRobotTransform(x,y,theta,map_to_robot_);
  fk_out = map_to_robot_*fk_out;

  ROS_DEBUG("[cspace] x: %0.3f  y: %0.3f  theta: %0.3f  torso: %0.3f  frame_num: %d  pose: %0.3f %0.3f %0.3f", x, y, theta, torso, frame_num, fk_out.p.x(), fk_out.p.y(), fk_out.p.z());
  return true;
}

void PR2CollisionSpace::getMaptoRobotTransform(double x, double y, double theta, KDL::Frame &frame)
{
  KDL::Rotation r1;
  r1.DoRotZ(theta);
  KDL::Vector t1(x,y,0.0);
  KDL::Frame base_footprint_in_map(r1,t1);
  frame = base_footprint_in_map;
}

void PR2CollisionSpace::getVoxelsInGroup(KDL::Frame &frame, Group &group)
{
  KDL::Vector v;
  for(size_t i = 0; i < group.spheres.size(); ++i)
  {
    v = frame*group.spheres[i].v;
    grid_->worldToGrid(v.data[0],v.data[1],v.data[2],group.spheres[i].voxel[0], group.spheres[i].voxel[1], group.spheres[i].voxel[2]);
  }
}

void PR2CollisionSpace::getPointsInGroup(KDL::Frame &frame, Group &group, std::vector<std::vector<double> > &points)
{
  KDL::Vector v;
  points.resize(group.spheres.size(), std::vector<double>(4,0));
  for(size_t i = 0; i < group.spheres.size(); ++i)
  {
    v = frame*group.spheres[i].v;
    points[i][0] = v.data[0];
    points[i][1] = v.data[1];
    points[i][2] = v.data[2];
    points[i][3] = group.spheres[i].radius;
  }
}

bool PR2CollisionSpace::isBaseValid(double x, double y, double theta, unsigned char &dist)
{
  unsigned char dist_temp;

  getMaptoRobotTransform(x,y,theta,base_g_.f);

  // base
  getVoxelsInGroup(base_g_.f, base_g_);
  for(size_t i = 0; i < base_g_.spheres.size(); ++i)
  {
    // check bounds
    if(!grid_->isInBounds(base_g_.spheres[i].voxel[0], base_g_.spheres[i].voxel[1], base_g_.spheres[i].voxel[2]))
    {
      int dimx, dimy, dimz;
      grid_->getGridSize(dimx,dimy,dimz);
      ROS_DEBUG_NAMED(cspace_log_,"[cspace] [base] Sphere %d is out of bounds. (xyz: %d %d %d,  dims: %d %d %d)", int(i), base_g_.spheres[i].voxel[0], base_g_.spheres[i].voxel[1], base_g_.spheres[i].voxel[2], dimx, dimy, dimz);
      return false;
    }
    if((dist_temp = grid_->getCell(base_g_.spheres[i].voxel[0], base_g_.spheres[i].voxel[1], base_g_.spheres[i].voxel[2])) <= base_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }

  // lower torso
  torso_lower_g_.f = base_g_.f;
  getVoxelsInGroup(base_g_.f, torso_lower_g_);
  for(size_t i = 0; i < torso_lower_g_.spheres.size(); ++i)
  {
    // check bounds
    if(!grid_->isInBounds(torso_lower_g_.spheres[i].voxel[0], torso_lower_g_.spheres[i].voxel[1], torso_lower_g_.spheres[i].voxel[2]))
    {
      int dimx, dimy, dimz;
      grid_->getGridSize(dimx,dimy,dimz);
      ROS_DEBUG_NAMED(cspace_log_,"[cspace] [base] Sphere %d is out of bounds. (xyz: %d %d %d,  dims: %d %d %d)", int(i), torso_lower_g_.spheres[i].voxel[0], torso_lower_g_.spheres[i].voxel[1], torso_lower_g_.spheres[i].voxel[2], dimx, dimy, dimz);
      return false;
    }
    if((dist_temp = grid_->getCell(torso_lower_g_.spheres[i].voxel[0], torso_lower_g_.spheres[i].voxel[1], torso_lower_g_.spheres[i].voxel[2])) <= torso_lower_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }
  return true;
}

bool PR2CollisionSpace::isTorsoValid(double x, double y, double theta, double torso, unsigned char &dist)
{
  unsigned char dist_temp;

  ROS_DEBUG("[cspace] Checking Torso. x: %0.3f y: %0.3f theta: %0.3f torso: %0.3f torso_kdl_num: %d",x,y,theta,torso,torso_upper_g_.kdl_segment);
  if(!computeFullBodyKinematics(x,y,theta,torso, torso_upper_g_.kdl_segment, torso_upper_g_.f))
  {
    ROS_ERROR("[cspace] Failed to compute FK for torso.");
    return false;
  }

  // upper torso
  getVoxelsInGroup(torso_upper_g_.f, torso_upper_g_);
  for(size_t i = 0; i < torso_upper_g_.spheres.size(); ++i)
  {
    // check bounds
    if(!grid_->isInBounds(torso_upper_g_.spheres[i].voxel[0], torso_upper_g_.spheres[i].voxel[1], torso_upper_g_.spheres[i].voxel[2]))
    {
      int dimx, dimy, dimz;
      grid_->getGridSize(dimx,dimy,dimz);
      ROS_DEBUG_NAMED(cspace_log_,"[torso] [base] Sphere %d is out of bounds. (xyz: %d %d %d,  dims: %d %d %d)", int(i), torso_upper_g_.spheres[i].voxel[0], torso_upper_g_.spheres[i].voxel[1], torso_upper_g_.spheres[i].voxel[2], dimx, dimy, dimz);
      return false;
    }
    if((dist_temp = grid_->getCell(torso_upper_g_.spheres[i].voxel[0], torso_upper_g_.spheres[i].voxel[1], torso_upper_g_.spheres[i].voxel[2])) <= torso_upper_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }
  // tilt laser
  tilt_laser_g_.f = torso_upper_g_.f;
  getVoxelsInGroup(tilt_laser_g_.f, tilt_laser_g_);
  for(size_t i = 0; i < tilt_laser_g_.spheres.size(); ++i)
  {
    // check bounds
    if(!grid_->isInBounds(tilt_laser_g_.spheres[i].voxel[0], tilt_laser_g_.spheres[i].voxel[1], tilt_laser_g_.spheres[i].voxel[2]))
    {
      int dimx, dimy, dimz;
      grid_->getGridSize(dimx,dimy,dimz);
      ROS_DEBUG_NAMED(cspace_log_,"[torso] [base] Sphere %d is out of bounds. (xyz: %d %d %d,  dims: %d %d %d)", int(i), tilt_laser_g_.spheres[i].voxel[0], tilt_laser_g_.spheres[i].voxel[1], tilt_laser_g_.spheres[i].voxel[2], dimx, dimy, dimz);
      return false;
    }
    if((dist_temp = grid_->getCell(tilt_laser_g_.spheres[i].voxel[0], tilt_laser_g_.spheres[i].voxel[1], tilt_laser_g_.spheres[i].voxel[2])) <= tilt_laser_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }
  // turrets
  turrets_g_.f = torso_upper_g_.f;
  getVoxelsInGroup(turrets_g_.f, turrets_g_);
  for(size_t i = 0; i < turrets_g_.spheres.size(); ++i)
  {
    // check bounds
    if(!grid_->isInBounds(turrets_g_.spheres[i].voxel[0], turrets_g_.spheres[i].voxel[1], turrets_g_.spheres[i].voxel[2]))
    {
      int dimx, dimy, dimz;
      grid_->getGridSize(dimx,dimy,dimz);
      ROS_DEBUG_NAMED(cspace_log_,"[cspace] [torso] Sphere %d is out of bounds. (xyz: %d %d %d,  dims: %d %d %d)", int(i), turrets_g_.spheres[i].voxel[0], turrets_g_.spheres[i].voxel[1], turrets_g_.spheres[i].voxel[2], dimx, dimy, dimz);
      return false;
    }
    if((dist_temp = grid_->getCell(turrets_g_.spheres[i].voxel[0], turrets_g_.spheres[i].voxel[1], turrets_g_.spheres[i].voxel[2])) <= turrets_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }

  return true;
}

bool PR2CollisionSpace::isHeadValid(double x, double y, double theta, double torso, unsigned char &dist)
{
  unsigned char dist_temp;

  ROS_DEBUG("[cspace] Checking Head. x: %0.3f y: %0.3f theta: %0.3f torso: %0.3f head_kdl_num: %d",x,y,theta,torso,head_g_.kdl_segment);
  if(!computeFullBodyKinematics(x,y,theta,torso, head_g_.kdl_segment, head_g_.f))
  {
    ROS_ERROR("[cspace] Failed to compute FK for torso.");
    return false;
  }

  // head
  getVoxelsInGroup(head_g_.f, head_g_);
  for(size_t i = 0; i < head_g_.spheres.size(); ++i)
  {
    // check bounds
    if(!grid_->isInBounds(head_g_.spheres[i].voxel[0], head_g_.spheres[i].voxel[1], head_g_.spheres[i].voxel[2]))
    {
      int dimx, dimy, dimz;
      grid_->getGridSize(dimx,dimy,dimz);
      ROS_DEBUG_NAMED(cspace_log_,"[cspace] [head] Sphere %d is out of bounds. (xyz: %d %d %d,  dims: %d %d %d)", int(i), head_g_.spheres[i].voxel[0], head_g_.spheres[i].voxel[1], head_g_.spheres[i].voxel[2], dimx, dimy, dimz);
      return false;
    }
    if((dist_temp = grid_->getCell(head_g_.spheres[i].voxel[0], head_g_.spheres[i].voxel[1], head_g_.spheres[i].voxel[2])) <= head_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }
  return true;
}

bool PR2CollisionSpace::isBodyValid(double x, double y, double theta, double torso, unsigned char &dist)
{
  return isBaseValid(x,y,theta,dist) &
         isTorsoValid(x,y,theta,torso,dist) &
         isHeadValid(x,y,theta,torso,dist);
}

bool PR2CollisionSpace::checkCollisionArmsToGroup(Group &group, unsigned char &dist)
{
  //NOTE: Assumes you computed the kinematics for all the joints already.
  int d;
  /*
  printGroupVoxels(group, group.name);
  printGroupVoxels(rgripper_g_, "right gripper");
  printGroupVoxels(lgripper_g_, "left gripper");
  printGroupVoxels(rforearm_g_, "right forearm");
  printGroupVoxels(lforearm_g_, "left forearm");
  */

  for(size_t i = 0; i < group.spheres.size(); ++i)
  {
    // vs right gripper
    for(size_t j = 0; j < rgripper_g_.spheres.size(); ++j)
    {
      if((d = getDistanceBetweenPoints(rgripper_g_.spheres[j].voxel[0], rgripper_g_.spheres[j].voxel[1], rgripper_g_.spheres[j].voxel[2],group.spheres[i].voxel[0], group.spheres[i].voxel[1], group.spheres[i].voxel[2])) <= max(rgripper_g_.spheres[j].radius_c, group.spheres[i].radius_c))
      {
        if(d < dist)
          dist = d;
        ROS_INFO("COLLISION");
        return false;
      }
      dist = min(d, int(dist));
      //ROS_INFO("[cspace] [%s-right_gripper] sphere: %d arm_sphere: %d dist: %d   arm_radius: %d  other_radius: %d", group.name.c_str(), int(i), int(j), d, rgripper_g_.spheres[j].radius_c, group.spheres[i].radius_c);
    }
    // vs left gripper
    for(size_t j = 0; j < lgripper_g_.spheres.size(); ++j)
    {
      if((d = getDistanceBetweenPoints(lgripper_g_.spheres[j].voxel[0], lgripper_g_.spheres[j].voxel[1], lgripper_g_.spheres[j].voxel[2],group.spheres[i].voxel[0], group.spheres[i].voxel[1], group.spheres[i].voxel[2])) <= max(lgripper_g_.spheres[j].radius_c, group.spheres[i].radius_c))
      {
        if(d < dist)
          dist = d;
        ROS_INFO("COLLISION");
        return false;
      }
      dist = min(d, int(dist));
      //ROS_INFO("[cspace] [%s-left_gripper] sphere: %d arm_sphere: %d dist: %d   arm_radius: %d  other_radius: %d", group.name.c_str(), int(i), int(j), d, lgripper_g_.spheres[j].radius_c, group.spheres[i].radius_c);
    }
    // vs right forearm
    for(size_t j = 0; j < rforearm_g_.spheres.size(); ++j)
    {
      if((d = getDistanceBetweenPoints(rforearm_g_.spheres[j].voxel[0], rforearm_g_.spheres[j].voxel[1], rforearm_g_.spheres[j].voxel[2],group.spheres[i].voxel[0], group.spheres[i].voxel[1], group.spheres[i].voxel[2])) <= max(rforearm_g_.spheres[j].radius_c, group.spheres[i].radius_c))
      {
        if(d < dist)
          dist = d;
        ROS_INFO("COLLISION");
        return false;
      }
      dist = min(d, int(dist));
      //ROS_INFO("[cspace] [%s-right_forearm] sphere: %d arm_sphere: %d dist: %d  arm_radius: %d  other_radius: %d", group.name.c_str(), int(i), int(j), d, rforearm_g_.spheres[j].radius_c, group.spheres[i].radius_c);
    }
    // vs left forearm
    for(size_t j = 0; j < lforearm_g_.spheres.size(); ++j)
    {
      if((d = getDistanceBetweenPoints(lforearm_g_.spheres[j].voxel[0], lforearm_g_.spheres[j].voxel[1], lforearm_g_.spheres[j].voxel[2],group.spheres[i].voxel[0], group.spheres[i].voxel[1], group.spheres[i].voxel[2])) <= max(lforearm_g_.spheres[j].radius_c, group.spheres[i].radius_c))
      {
        if(d < int(dist))
          dist = d;
        ROS_INFO("COLLISION");
        return false;
      }
      dist = min(d, int(dist));
      //ROS_INFO("[cspace] [%s-left_forearm] sphere: %d arm_sphere: %d dist: %d  arm_radius: %d  other_radius: %d", group.name.c_str(), int(i), int(j), d, lforearm_g_.spheres[j].radius_c, group.spheres[i].radius_c);
    }
  }
  return true;
}

bool PR2CollisionSpace::checkCollisionArmsToBody(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, unsigned char &dist)
{
  // compute arm kinematics
  arm_[0]->computeFK(rangles, pose, 10, &(rgripper_g_.f)); // right gripper
  getVoxelsInGroup(rgripper_g_.f, rgripper_g_); 
  arm_[0]->computeFK(rangles, pose, 7, &(rforearm_g_.f)); // right forearm
  getVoxelsInGroup(rforearm_g_.f, rforearm_g_); 
  arm_[1]->computeFK(langles, pose, 10, &(lgripper_g_.f)); // left gripper
  getVoxelsInGroup(lgripper_g_.f, lgripper_g_); 
  arm_[1]->computeFK(langles, pose, 7, &(lforearm_g_.f)); // left forearm
  getVoxelsInGroup(lforearm_g_.f, lforearm_g_); 
 
  if(!checkCollisionArmsToGroup(base_g_, dist))
  {
    ROS_INFO("[cspace] base - arms collision. (dist: %d)",dist);
    return false;
  }
  if(!checkCollisionArmsToGroup(turrets_g_, dist))
  {
    ROS_INFO("[cspace] turrets - arms collision. (dist: %d)",dist);
    return false;
  }
  if(!checkCollisionArmsToGroup(tilt_laser_g_, dist))
  {
    ROS_INFO("[cspace] tilt_laser - arms collision. (dist: %d)",dist);
    return false;
  }
  if(!checkCollisionArmsToGroup(torso_upper_g_, dist))
  {
    ROS_INFO("[cspace] torso_upper - arms collision. (dist: %d)",dist);
    return false;
  }
  if(!checkCollisionArmsToGroup(torso_lower_g_, dist))
  {
    ROS_INFO("[cspace] torso_lower - arms collision. (dist: %d)",dist);
    return false;
  }
  if(!checkCollisionArmsToGroup(head_g_, dist))
  {
  ROS_INFO("[cspace] head - arms collision. (dist: %d)",dist);
  return false;
  }

  return true;
}

void PR2CollisionSpace::getCollisionSpheres(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, std::string group_name, std::vector<std::vector<double> > &spheres)
{
  if(group_name.compare("right_arm") == 0)
  {
    std::vector<double> xyzr(4,0);
    std::vector<std::vector<int> > points, jnts;
    if(!getJointPosesInGrid(rangles, pose, 0, jnts))
      return;

    for(int i = 0; i < int(jnts.size()-1); ++i)
    {
      points.clear();
      getLineSegment(jnts[i], jnts[i+1], points);
      for(int j = 0; j < int(points.size()); ++j)
      {
        grid_->gridToWorld(points[j][0],points[j][1],points[j][2],xyzr[0],xyzr[1],xyzr[2]); 
        xyzr[3]=double(arm_[0]->getLinkRadiusCells(i))*arm_[0]->resolution_;
        spheres.push_back(xyzr);
      }
    }
    return;
  }
  else if(group_name.compare("left_arm") == 0)
  {
    std::vector<double> xyzr(4,0);
    std::vector<std::vector<int> > points, jnts;
    if(!getJointPosesInGrid(langles, pose, 1, jnts))
      return;

    for(int i = 0; i < int(jnts.size()-1); ++i)
    {
      points.clear();
      getLineSegment(jnts[i], jnts[i+1], points);
      for(int j = 0; j < int(points.size()); ++j)
      {
        grid_->gridToWorld(points[j][0],points[j][1],points[j][2],xyzr[0],xyzr[1],xyzr[2]); 
        xyzr[3]=double(arm_[1]->getLinkRadiusCells(i))*arm_[1]->resolution_;
        spheres.push_back(xyzr);
      }
    }
    return;
  }

  Group g;
  if(group_name.compare("base") == 0)
  {
    g = base_g_;
    if(!computeFullBodyKinematics(pose.x,pose.y,pose.theta,pose.z, g.kdl_segment, g.f))
    {
      ROS_ERROR("[cspace] Failed to compute FK.");
      return;
    }
  }
  else if(group_name.compare("torso_upper") == 0)
  {
    g = torso_upper_g_;
    if(!computeFullBodyKinematics(pose.x,pose.y,pose.theta,pose.z, g.kdl_segment, g.f))
    {
      ROS_ERROR("[cspace] Failed to compute FK.");
      return;
    }
  }
  else if(group_name.compare("torso_lower") == 0)
  {
    g = torso_lower_g_;
    if(!computeFullBodyKinematics(pose.x,pose.y,pose.theta,pose.z, g.kdl_segment, g.f))
    {
      ROS_ERROR("[cspace] Failed to compute FK.");
      return;
    }
  }
  else if(group_name.compare("turrets") == 0)
  {
    g = turrets_g_;
    if(!computeFullBodyKinematics(pose.x,pose.y,pose.theta,pose.z, g.kdl_segment, g.f))
    {
      ROS_ERROR("[cspace] Failed to compute FK.");
      return;
    }
  }
  else if(group_name.compare("tilt_laser") == 0)
  {
    g = tilt_laser_g_;
    if(!computeFullBodyKinematics(pose.x,pose.y,pose.theta,pose.z, g.kdl_segment, g.f))
    {
      ROS_ERROR("[cspace] Failed to compute FK.");
      return;
    }
  }
  else if(group_name.compare("head") == 0)
  {
    g = head_g_;
    if(!computeFullBodyKinematics(pose.x,pose.y,pose.theta,pose.z, g.kdl_segment, g.f))
    {
      ROS_ERROR("[cspace] Failed to compute FK.");
      return;
    }
  }
  else if(group_name.compare("left_gripper") == 0)
  {
    g = lgripper_g_;
    arm_[1]->computeFK(langles, pose, g.kdl_segment, &(g.f));
  }
  else if(group_name.compare("right_gripper") == 0)
  {
    g = rgripper_g_;
    arm_[0]->computeFK(rangles, pose, g.kdl_segment, &(g.f));
  }
  else if(group_name.compare("left_forearm") == 0)
  {
    g = lforearm_g_;
    arm_[1]->computeFK(langles, pose, g.kdl_segment, &(g.f));
  }
  else if(group_name.compare("right_forearm") == 0)
  {
    g = rforearm_g_;
    arm_[0]->computeFK(rangles, pose, g.kdl_segment, &(g.f));
  }
  
  getPointsInGroup(g.f, g, spheres);
  ROS_DEBUG("[cspace] Returning %d spheres for %s group", int(spheres.size()), group_name.c_str());
}

void PR2CollisionSpace::printGroupVoxels(Group &g, std::string text)
{
  ROS_INFO("[cspace] voxels: %s", text.c_str());
  for(size_t i = 0; i < g.spheres.size(); ++i)
  {
    ROS_INFO("[cspace] voxel %d: %d %d %d", int(i), g.spheres[i].voxel[0], g.spheres[i].voxel[1], g.spheres[i].voxel[2]);
  }
}

bool PR2CollisionSpace::checkSpineMotion(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  // arm-base
  getMaptoRobotTransform(pose.x,pose.y,pose.theta,base_g_.f);
  getVoxelsInGroup(base_g_.f, base_g_);
  if(!checkCollisionArmsToGroup(base_g_, dist))
    return false;

  // torso-world
  if(!isTorsoValid(pose.x, pose.y, pose.theta, pose.z, dist))
    return false;

  // head-world
  if(!isHeadValid(pose.x, pose.y, pose.theta, pose.z, dist))
    return false;

  // arms-world
  if(!checkCollision(rangles, pose, 0, verbose, dist))
  {
    debug_code = sbpl_arm_planner::RIGHT_ARM_IN_COLLISION;
    return false;
  }
  if(!checkCollision(langles, pose, 1, verbose, dist))
  {
    debug_code = sbpl_arm_planner::LEFT_ARM_IN_COLLISION;
    return false;
  }
 
  return true;
}

bool PR2CollisionSpace::checkBaseMotion(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  // base-world
  if(!isBaseValid(pose.x, pose.y, pose.theta, dist))
  {
    //ROS_INFO("[cspace] base-world collision.");
    return false;
  }

  // attached_object-world
  if(!isAttachedObjectValid(langles, rangles, pose, verbose, dist, debug_code))
    return false;

  // arms-world
  if(!checkCollision(rangles, pose, 0, verbose, dist))
  {
    debug_code = sbpl_arm_planner::RIGHT_ARM_IN_COLLISION;
    return false;
  }
  if(!checkCollision(langles, pose, 1, verbose, dist))
  {
    debug_code = sbpl_arm_planner::LEFT_ARM_IN_COLLISION;
    return false;
  }

  // torso-world
  if(!isTorsoValid(pose.x, pose.y, pose.theta, pose.z, dist))
    return false;

  // head-world
  if(!isHeadValid(pose.x, pose.y, pose.theta, pose.z, dist))
    return false;

  return true;
}

bool PR2CollisionSpace::checkArmsMotion(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  // arms-world, arms-arms
  if(!checkCollisionArms(langles, rangles, pose, verbose, dist, debug_code))
    return false;

  // arms-body
  if(!checkCollisionArmsToBody(langles, rangles, pose, dist))
    return false;

  return true;
}

bool PR2CollisionSpace::checkAllMotion(std::vector<double> &langles, std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  // attached_object-world
  if(!isAttachedObjectValid(langles, rangles, pose, verbose, dist, debug_code))
    return false;

  // arms-world, arms-arms
  if(!checkCollisionArms(langles, rangles, pose, verbose, dist, debug_code))
    return false;

  // body-world
  if(!isBodyValid(pose.x, pose.y, pose.theta, pose.z, dist))
    return false;

  // arms-body
  if(!checkCollisionArmsToBody(langles, rangles, pose, dist))
    return false;

  return true;
}

void PR2CollisionSpace::getIntermediatePoints(KDL::Vector a, KDL::Vector b, double d, std::vector<KDL::Vector>& points)
{
	KDL::Vector pt, dir;
	int interm_points = floor(distance(a, b) / d + 0.5);

	dir = b - a;
	points.clear();
	points.push_back(a);
	for (int i = 1; i <= interm_points; i++) {
		pt = a + dir * i * d;
		points.push_back(pt);
	}
	points.push_back(b);
}

/* ******************  Attached Object **************** */
void PR2CollisionSpace::removeAllAttachedObjects()
{
  is_object_attached_ = false;
  objects_.clear();
  ROS_INFO("[cspace] Removed all attached objects.");
}

void PR2CollisionSpace::removeAttachedObject(std::string name)
{
  bool removed = false;
  for(size_t i = 0; i < objects_.size(); ++i)
  {
    if(objects_[i].name.compare(name) == 0)
    {
      objects_.erase(objects_.begin()+i);
      removed = true;
    }   
  }
  if(removed)
    ROS_INFO("[cspace] Removed requested attached object, '%s'.", name.c_str());
  else
    ROS_WARN("[cspace] Was asked to remove %s attached object but it wasn't attached.", name.c_str());
}

void PR2CollisionSpace::attachSphere(std::string name, std::string link, geometry_msgs::Pose pose, double radius)
{
  is_object_attached_ = true;
  AttachedObject obj;

  // TODO: Get KDL segment index for 'frame', for now we know it's _wrist_roll_link, 8
  obj.kdl_segment = 10;
  obj.name = name;
  tf::PoseMsgToKDL(pose, obj.pose);
  obj.link = link;

  if(link.substr(0,1).compare("r") == 0)
    obj.side = pr2_collision_checker::Right; 
  else if(link.substr(0,1).compare("l") == 0)
    obj.side = pr2_collision_checker::Left;
  else
  {
    obj.side = pr2_collision_checker::Body;
    obj.kdl_segment = getSegmentIndex(link, full_body_chain_);
    if(obj.kdl_segment == -1)
      return;
  }

  obj.spheres.resize(1);
  obj.spheres[0].radius = radius;
  obj.spheres[0].radius_c = radius / grid_->getResolution() + 0.5;
  obj.spheres[0].v.x(0.0); 
  obj.spheres[0].v.y(0.0); 
  obj.spheres[0].v.z(0.0); 

  int ind = -1;
  if((ind = getAttachedObjectIndex(name)) > -1)
  {
    ROS_WARN("[cspace] Already had an object attached with the name, '%s'. Replacing that object with this new one.", name.c_str());
    objects_[ind] = obj;
  }
  else
    objects_.push_back(obj);

  //ROS_INFO("[cspace] [attached_object] Attached '%s' sphere to the %s arm.  pose: %0.3f %0.3f %0.3f radius: %0.3fm (%d cells)", name.c_str(), arm_side_names[obj.side].c_str(), pose.position.x,pose.position.y,pose.position.z, obj.spheres[0].radius, obj.spheres[0].radius_c);
}

void PR2CollisionSpace::attachCylinder(std::string name, std::string link, geometry_msgs::Pose pose, double radius, double length)
{
  std::vector<KDL::Vector> points;
  
  is_object_attached_ = true;
  AttachedObject obj;
  obj.kdl_segment = 10;
  obj.name = name;
  obj.link = link;
  tf::PoseMsgToKDL(pose, obj.pose);

  if(link.substr(0,1).compare("r") == 0)
    obj.side = pr2_collision_checker::Right; 
  else if(link.substr(0,1).compare("l") == 0)
    obj.side = pr2_collision_checker::Left;
  else
  {
    obj.side = pr2_collision_checker::Body;
    obj.kdl_segment = getSegmentIndex(link, full_body_chain_);
    if(obj.kdl_segment == -1)
      return;
  }

  // compute end points of cylinder
  KDL::Vector top, bottom;
  top.data[2] += length/2.0;
  bottom.data[2] -= length/2.0;

  // get spheres 
  getIntermediatePoints(top, bottom, radius, points);

  if(points.size() < 1)
  {
    ROS_WARN("[cspace] Trying to attach '%s' cylinder (with radius %0.3fm) to %s arm but it got converted to 0 spheres?", name.c_str(), radius, arm_side_names[obj.side].c_str());
    return;
  }
  obj.spheres.resize(points.size());
  ROS_INFO("[cspace] %s spheres:", name.c_str());
  for(size_t i = 0; i < points.size(); ++i)
  {
    obj.spheres[i].name = name + "_" + boost::lexical_cast<std::string>(i);
    obj.spheres[i].v = points[i];
    obj.spheres[i].radius = radius;
    obj.spheres[i].radius_c = radius / grid_->getResolution() + 0.5;
    ROS_INFO("[cspace] [%d] xyz: %0.3f %0.3f %0.3f  radius: %0.3fm", int(i), obj.spheres[i].v.x(), obj.spheres[i].v.y(), obj.spheres[i].v.z(), radius);
  }
  int ind = -1;
  if((ind = getAttachedObjectIndex(name)) > -1)
  {
    ROS_WARN("[cspace] Already had an object attached with the name, '%s'. Replacing that object with this new one.", name.c_str());
    objects_[ind] = obj;
  }
  else
    objects_.push_back(obj);

  ROS_INFO("[cspace] [attached_object] Attaching cylinder. pose: %0.3f %0.3f %0.3f radius: %0.3f length: %0.3f spheres: %d", pose.position.x,pose.position.y,pose.position.z, radius, length, int(obj.spheres.size()));
  ROS_INFO("[cspace] [attached_object]    top: xyz: %0.3f %0.3f %0.3f  radius: %0.3fm (%d cells)", top.x(), top.y(), top.z(), radius, obj.spheres[0].radius_c);
  ROS_INFO("[cspace] [attached_object] bottom: xyz: %0.3f %0.3f %0.3f  radius: %0.3fm (%d cells)", bottom.x(), bottom.y(), bottom.z(), radius, obj.spheres[0].radius_c);
}

void PR2CollisionSpace::attachCube(std::string name, std::string link, geometry_msgs::Pose pose, double x_dim, double y_dim, double z_dim)
{
  std::vector<std::vector<double> > spheres;
  is_object_attached_ = true;
  AttachedObject obj;
  obj.kdl_segment = 10;
  obj.name = name;
  obj.link = link;
  tf::PoseMsgToKDL(pose, obj.pose);

  if(link.substr(0,1).compare("r") == 0)
    obj.side = pr2_collision_checker::Right; 
  else if(link.substr(0,1).compare("l") == 0)
    obj.side = pr2_collision_checker::Left;
  else
  {
    obj.side = pr2_collision_checker::Body;
    obj.kdl_segment = getSegmentIndex(link, full_body_chain_);
    if(obj.kdl_segment == -1)
      return;
  }

  sbpl::SphereEncloser::encloseBox(x_dim, y_dim, z_dim, cube_filling_sphere_radius_, spheres);

  if(spheres.size() <= 3)
    ROS_WARN("[cspace] Attached cube is represented by %d collision spheres. Consider lowering the radius of the spheres used to populate the attached cube. (radius = %0.3fm)", int(spheres.size()), cube_filling_sphere_radius_);

  obj.spheres.resize(spheres.size());
  for(size_t i = 0; i < spheres.size(); ++i)
  {
    obj.spheres[i].name = name + "_" + boost::lexical_cast<std::string>(i);
    obj.spheres[i].v.x(spheres[i][0]);
    obj.spheres[i].v.y(spheres[i][1]);
    obj.spheres[i].v.z(spheres[i][2]);
    obj.spheres[i].radius = spheres[i][3];
    obj.spheres[i].radius_c = spheres[i][3] / grid_->getResolution() + 0.5;
    ROS_INFO("[%d] %0.3f %0.3f %0.3f", int(i), obj.spheres[i].v.x(), obj.spheres[i].v.y(), obj.spheres[i].v.z());
  }
 
  int ind = -1;
  if((ind = getAttachedObjectIndex(name)) > -1)
  {
    ROS_WARN("[cspace] Already had an object attached with the name, '%s'. Replacing that object with this new one.", name.c_str());
    objects_[ind] = obj;
  }
  else
    objects_.push_back(obj);
   
  ROS_INFO("[cspace] Attaching cube represented by %d spheres with dimensions: %0.3f %0.3f %0.3f", int(spheres.size()), x_dim, y_dim, z_dim);
}

void PR2CollisionSpace::attachMesh(std::string name, std::string link, geometry_msgs::Pose pose, const std::vector<geometry_msgs::Point> &vertices, const std::vector<int> &triangles)
{
  std::vector<std::vector<double> > spheres;
  is_object_attached_ = true;
  AttachedObject obj;
  obj.kdl_segment = 10;
  obj.name = name;
  obj.link = link;
  tf::PoseMsgToKDL(pose, obj.pose);

  if(link.substr(0,1).compare("r") == 0)
    obj.side = pr2_collision_checker::Right; 
  else if(link.substr(0,1).compare("l") == 0)
    obj.side = pr2_collision_checker::Left;
  else
  {
    obj.side = pr2_collision_checker::Body;
    obj.kdl_segment = getSegmentIndex(link, full_body_chain_);
    if(obj.kdl_segment == -1)
      return;
  }
  sbpl::SphereEncloser::encloseMesh(vertices, triangles, cube_filling_sphere_radius_, spheres);
  
  if(spheres.size() <= 3)
    ROS_WARN("[cspace] Attached mesh is represented by %d collision spheres. Consider lowering the radius of the spheres used to populate the attached mesh more accuratly. (radius = %0.3fm)", int(spheres.size()), cube_filling_sphere_radius_);

  obj.spheres.resize(spheres.size());
  for(size_t i = 0; i < spheres.size(); ++i)
  {
    obj.spheres[i].name = name + "_" + boost::lexical_cast<std::string>(i);
    obj.spheres[i].v.x(spheres[i][0]);
    obj.spheres[i].v.y(spheres[i][1]);
    obj.spheres[i].v.z(spheres[i][2]);
    obj.spheres[i].radius = spheres[i][3];
    obj.spheres[i].radius_c = spheres[i][3] / grid_->getResolution() + 0.5;
    ROS_DEBUG("[%d] %0.3f %0.3f %0.3f", int(i), obj.spheres[i].v.x(), obj.spheres[i].v.y(), obj.spheres[i].v.z());
  }

  int ind = -1;
  if((ind = getAttachedObjectIndex(name)) > -1)
  {
    ROS_WARN("[cspace] Already had an object attached with the name, '%s'. Replacing that object with this new one.", name.c_str());
    objects_[ind] = obj;
  }
  else
    objects_.push_back(obj);

  ROS_INFO("[cspace] Attaching mesh represented by %d spheres with %d vertices and %d triangles.", int(spheres.size()), int(vertices.size()), int(triangles.size()));
}

void PR2CollisionSpace::getAttachedObjectSpheres(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, std::vector<std::vector<double> > &spheres)
{
  KDL::Frame f;
  KDL::Vector v;
  spheres.clear();
  std::vector<std::vector<double> > obj_sph;
  for(size_t i = 0; i < objects_.size(); ++i)
  {
    if(objects_[i].side == pr2_collision_checker::Right)
      arm_[objects_[i].side]->computeFK(rangles, pose, objects_[i].kdl_segment, &(objects_[i].f));
    else if(objects_[i].side == pr2_collision_checker::Left)
      arm_[objects_[i].side]->computeFK(langles, pose, objects_[i].kdl_segment, &(objects_[i].f));
    else
      computeFullBodyKinematics(pose.x, pose.y, pose.theta, pose.z, objects_[i].kdl_segment, objects_[i].f);
  
    ROS_DEBUG_NAMED(cspace_log_, "[cspace]    pose of attached link in map: %0.3f %0.3f %0.3f", objects_[i].f.p.x(), objects_[i].f.p.y(), objects_[i].f.p.z());
    // T_obj_in_world = T_link_in_world * T_obj_in_link
    f = objects_[i].f * objects_[i].pose;
    
    ROS_DEBUG_NAMED(cspace_log_, "[cspace]   pose of object in map: %0.3f %0.3f %0.3f", f.p.x(), f.p.y(), f.p.z());

    //spheres.resize(spheres.size()+objects_[i].spheres.size(),std::vector<double>(5,0));
    obj_sph.resize(objects_[i].spheres.size(), std::vector<double>(5,0));
    for(size_t j = 0; j < objects_[i].spheres.size(); ++j)
    {
      // v_sphere_in_world = T_obj_in_world * v_pos_in_obj
      v = f * objects_[i].spheres[j].v;
      obj_sph[j][0] = v.x();
      obj_sph[j][1] = v.y();
      obj_sph[j][2] = v.z();
      obj_sph[j][3] = objects_[i].spheres[j].radius;
      obj_sph[j][4] = objects_[i].spheres[j].radius_c;
      ROS_DEBUG_NAMED(cspace_log_, "[cspace] [%d] pose of sphere in object: %0.3f %0.3f %0.3f",  int(i), objects_[i].spheres[j].v.x(),  objects_[i].spheres[j].v.y(), objects_[i].spheres[j].v.z());
    }
    spheres.insert(spheres.end(), obj_sph.begin(), obj_sph.end());
  }

  // debug
  for(size_t i = 0; i < spheres.size(); ++i)
    ROS_DEBUG_NAMED(cspace_log_, "[cspace] [%d] xyz: %0.3f %0.3f %0.3f  radius: %0.3f  radius_c: %2.0f", int(i), spheres[i][0], spheres[i][1], spheres[i][2], spheres[i][3], spheres[i][4]);
  ROS_DEBUG_NAMED(cspace_log_, "[cspace] Fetched %d spheres.", int(spheres.size()));
}

void PR2CollisionSpace::getAttachedObjectVoxels(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, std::vector<std::vector<int> > &voxels)
{
  std::vector<std::vector<double> > spheres;
  getAttachedObjectSpheres(langles, rangles, pose, spheres);
  
  voxels.resize(spheres.size(), std::vector<int> (4,0));
  for(size_t i = 0; i < spheres.size(); ++i)
  {
    grid_->worldToGrid(spheres[i][0],spheres[i][1],spheres[i][2],voxels[i][0],voxels[i][1],voxels[i][2]);
    voxels[i][3] = spheres[i][4];
  }
  ROS_DEBUG("[cspace] Fetched %d voxels.", int(voxels.size()));
}

bool PR2CollisionSpace::isAttachedObjectValid(const std::vector<double> &langles, const std::vector<double> &rangles, BodyPose &pose, bool verbose, unsigned char &dist, int &debug_code)
{
  if(!is_object_attached_)
    return true;

  unsigned char dist_temp = 100;
  std::vector<std::vector<int> > voxels;
  getAttachedObjectVoxels(langles, rangles, pose, voxels);

  for(size_t i = 0; i < voxels.size(); ++i)
  {
    // check bounds
    if(!grid_->isInBounds(voxels[i][0], voxels[i][1], voxels[i][2]))
    {
      //if(verbose)
      //{
      int dimx, dimy, dimz;
      grid_->getGridSize(dimx,dimy,dimz);

      //ROS_DEBUG_NAMED(cspace_log_,"[cspace] Sphere %d is out of bounds. (xyz: %d %d %d,  dims: %d %d %d)", int(i), voxels[i][0], voxels[i][1], voxels[i][2], dimx, dimy, dimz);
      ROS_INFO("[cspace] [attached object] Sphere %d is out of bounds. (xyz: %d %d %d,  dims: %d %d %d)", int(i), voxels[i][0], voxels[i][1], voxels[i][2], dimx, dimy, dimz);
      //}
      return false;
    }

    // check collision
    if((dist_temp = grid_->getCell(voxels[i][0], voxels[i][1], voxels[i][2])) <= voxels[i][3])
    {
      dist = dist_temp;
      debug_code = sbpl_arm_planner::ATTACHED_OBJECT_IN_COLLISION;
      return false;
    }

    if(dist > dist_temp)
      dist = dist_temp;
  }
  return true;
}

std::string PR2CollisionSpace::getExpectedAttachedObjectFrame(std::string frame)
{
  int ind=-1;

  // is the object attached to the right arm?
  if((ind = arm_[0]->getSegmentIndex(frame)) > -1 || frame.compare("r_gripper_l_finger_tip_link") == 0)
  {
    ROS_INFO("[cspace] Attached object frame (%s) was found in the right arm kdl chain at index: %d.", frame.c_str(), ind);
    return "r" + attached_object_frame_suffix_;
  }
  // is the object attached to the left arm?
  else if((ind = arm_[1]->getSegmentIndex(frame)) > -1 || frame.compare("l_gripper_l_finger_tip_link") == 0)
  {
    ROS_INFO("[cspace] Attached object frame (%s) was found in the left arm kdl chain at index: %d.", frame.c_str(), ind);
    return "l" + attached_object_frame_suffix_;
  }
  // is the object attached to the body?
  else if((ind = getSegmentIndex(frame, full_body_chain_)) > -1)
  {
    ROS_INFO("[cspace] Attached object frame (%s) was found in the body kdl chain at index: %d.", frame.c_str(), ind);
    return "base_link";
  }
  else
  {
    ROS_ERROR("[cspace] Cannot compute kinematics for the attached object frame, '%s', so I'm not attaching it.", frame.c_str());
    return "";
  }
}

// NOT USED
bool PR2CollisionSpace::getAttachedFrameInfo(std::string frame, int &segment, int &chain)
{  
  // is the object attached to the right arm?
  if((segment = arm_[0]->getSegmentIndex(frame)) > -1)
  {
    ROS_INFO("[cspace] Attached object frame (%s) was found in the right arm kdl chain at index: %d.", frame.c_str(), segment);
    chain = pr2_collision_checker::Right;
    return true;
  }
  // is the object attached to the left arm?
  else if((segment = arm_[1]->getSegmentIndex(frame)) > -1)
  {
    ROS_INFO("[cspace] Attached object frame (%s) was found in the left arm kdl chain at index: %d.", frame.c_str(), segment);
    chain = pr2_collision_checker::Left;
    return true;
  }
  // is the object attached to the body?
  else if((segment = getSegmentIndex(frame, full_body_chain_)) > -1)
  {
    ROS_INFO("[cspace] Attached object frame (%s) was found in the body kdl chain at index: %d.", frame.c_str(), segment);
    chain = pr2_collision_checker::Body;
    return true;
  }
  else
  {
    ROS_ERROR("[cspace] Cannot compute kinematics for the attached object frame, '%s', so can't attach it.", frame.c_str());
    return false;
  }
}

int PR2CollisionSpace::getAttachedObjectIndex(std::string name)
{
  for(int i = 0; i < int(objects_.size()); ++i)
  {
    if(objects_[i].name.compare(name) == 0)
      return i;
  }
  return -1;
}

void PR2CollisionSpace::setKinematicsToReferenceTransform(KDL::Frame f, std::string &name)
{
  arm_[0]->setRefFrameTransform(f, name);
  arm_[1]->setRefFrameTransform(f, name);
}

std::string PR2CollisionSpace::getKinematicsFrame()
{
  std::string name;
  arm_[0]->getArmChainRootLinkName(name);
  return name;
}

int PR2CollisionSpace::getSegmentIndex(std::string &name, KDL::Chain &chain)
{
  for(size_t k = 0; k < chain.getNrOfSegments(); ++k)
  {
    if(chain.getSegment(k).getName().compare(name) == 0)
      return k;
  }
  ROS_DEBUG("Failed to find %s segment in the chain.", name.c_str());
  return -1;
}

bool PR2CollisionSpace::isObjectAttached()
{
  if(objects_.empty())
    return false;
  else
    return true;
}

void PR2CollisionSpace::storeCollisionMap(const arm_navigation_msgs::CollisionMap &collision_map)
{
  last_collision_map_ = collision_map;
}

std::vector<Eigen::Vector3d> PR2CollisionSpace::btVectorToEigenVector(const std::vector<btVector3> &bt)
{
  std::vector<Eigen::Vector3d> e(bt.size());
  for(size_t i = 0; i < bt.size(); ++i)
    leatherman::btVector3ToEigen(bt[i],e[i]);
  return e;
}
}
