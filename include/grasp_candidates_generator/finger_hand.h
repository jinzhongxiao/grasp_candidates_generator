/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Andreas ten Pas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef FINGER_HAND_H
#define FINGER_HAND_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>


/** FingerHand class
 *
 * \brief Calculate collision-free finger placements.
 * 
 * This class calculates collision-free finger placements. The parameters are the outer diameter, the width of the
 * fingers, and the length of the fingers of the robot hand. Also uses the "bite" that the grasp must have. The
 * bite is how far the hand can be moved onto the object.
 * 
*/
class FingerHand
{
public:

  /**
   * \brief Default constructor.
  */
  FingerHand()
  {
  }
	
	/**
   * \brief Constructor.
   * \param finger_width the width of the fingers
   * \param hand_outer_diameter the maximum aperture of the robot hand
   * \param hand_depth the length of the fingers
  */
  FingerHand(double finger_width, double hand_outer_diameter, double hand_depth);

  /**
   * \brief Find possible finger placements.
   *
   * Finger placements need to be collision-free and contain at least one point in between the fingers.
   *
   * \param points the points to checked for possible finger placements
   * \param bite how far the robot can be moved into the object
   * \param idx if this is larger than -1, only check the <idx>-th finger placement
   */
  void evaluateFingers(const Eigen::Matrix3Xd& points, double bite, int idx = -1);

  /**
   * \brief Try to move the robot hand as far as possible onto the object.
   * \param points the points that the finger placement is evaluated on, assumed to be rotated into the hand frame and
   * cropped by hand height
   * \param min_depth the minimum depth that the hand can be moved onto the object
   * \param max_depth the maximum depth that the hand can be moved onto the object
   */
  void deepenHand(const Eigen::Matrix3Xd& points, double min_depth, double max_depth);

  /**
   * \brief Compute which of the given points are located in the closing region of the robot hand.
   * \param points the points
   * \return the points that are located in the closing region
   */
  std::vector<int> computePointsInClosingRegion(const Eigen::Matrix3Xd& points);

	/**
   * \brief Locate robot hand configurations by checking which finger placements were feasible.
  */
  void evaluateHand();

  void evaluateHand(int idx);

	/**
	 * \brief Return the depth of the hand.
	 * \return the hand depth
	*/
  double getHandDepth() const
  {
    return hand_depth_;
  }
	
	/**
	 * \brief Return the hand configuration evaluations.
	 * \return the hand configuration evaluations
	*/
  const Eigen::Array<bool, 1, Eigen::Dynamic>& getHand() const
  {
    return hand_;
  }
	
	/**
	 * \brief Return the finger placement evaluations.
	 * \return the hand configuration evaluations
	*/
  const Eigen::Array<bool, 1, Eigen::Dynamic>& getFingers() const
  {
    return fingers_;
  }
	
	/**
	 * \brief Return the width of the object contained in the grasp.
	 * \return the width of the object contained in the grasp
	*/
  double getGraspWidth() const
  {
    return grasp_width_;
  }
	
	/**
	 * \brief Return where the back of the hand is.
	 * \return the back of the hand
	*/
  double getBackOfHand() const
  {
    return back_of_hand_;
  }

  /**
   * \brief Return where the bottom of the hand is.
   * \return the bottom of the hand
  */
  double getBottom() const
  {
    return bottom_;
  }

  /**
   * \brief Return where the left finger is.
   * \return the left finger's location
  */
  double getLeft() const
  {
    return left_;
  }

  /**
   * \brief Return where the right finger is.
   * \return the right finger's location
  */
  double getRight() const
  {
    return right_;
  }

  /**
   * \brief Return where the top of the hand is (where the fingertips are).
   * \return the top of the hand
  */
  double getTop() const
  {
    return top_;
  }

  double getCenter() const
  {
    return center_;
  }

  double getSurface() const
  {
    return surface_;
  }

  int getForwardAxis() const
  {
    return forward_axis_;
  }

  void setForwardAxis(int forward_axis)
  {
    forward_axis_ = forward_axis;
  }

  int getLateralAxis() const
  {
    return lateral_axis_;
  }

  void setLateralAxis(int lateral_axis)
  {
    lateral_axis_ = lateral_axis;
  }

  void setBottom(double bottom)
  {
    bottom_ = bottom;
  }

  void setCenter(double center)
  {
    center_ = center;
  }

  void setLeft(double left)
  {
    left_ = left;
  }

  void setRight(double right)
  {
    right_ = right;
  }

  void setSurface(double surface)
  {
    surface_ = surface;
  }

  void setTop(double top)
  {
    top_ = top;
  }

private:

  int forward_axis_; ///< the index of the horizontal axis in the hand frame (grasp approach direction)
  int lateral_axis_; ///< the index of the vertical axis in the hand frame (closing direction of the robot hand)

  double finger_width_; ///< the width of the robot hand fingers
  double hand_outer_diameter_; ///< the maximum aperture of the robot hand (includes finger width)
  double hand_depth_; ///< the hand depth (finger length)

  double back_of_hand_; ///< where the back of the hand is (distance from back to position between finger tip ends)

  double grasp_width_; ///< the width of the object contained in the grasp

  Eigen::VectorXd finger_spacing_; ///< the possible finger placements
  Eigen::Array<bool, 1, Eigen::Dynamic> fingers_; ///< indicates the feasible finger placements
  Eigen::Array<bool, 1, Eigen::Dynamic> hand_;///< indicates the feasible hand locations
  double bottom_; ///< the base of the hand
  double top_; ///< the top of the hand, where the fingertips are
  double left_; ///< the left side of the gripper bounding box
  double right_; ///< the right side of the gripper bounding box
  double center_; ///< the horizontal center of the gripper bounding box
  double surface_; ///< the corresponding vertical base point of the hand in the point cloud
};

#endif
