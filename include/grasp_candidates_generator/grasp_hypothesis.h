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


#ifndef GRASP_HYPOTHESIS_H_
#define GRASP_HYPOTHESIS_H_


// Boost
#include <boost/lexical_cast.hpp>

// Eigen
#include <Eigen/Dense>

// System
#include <iostream>
#include <fstream>
#include <vector>

// custom
#include <grasp_candidates_generator/finger_hand.h>
#include <grasp_candidates_generator/point_list.h>


/** GraspHypothesis class
 *
 * \brief Grasp hypothesis data structure
 * 
 * This class stores a single grasp hypothesis.
 * 
 */
class GraspHypothesis
{
public:

  /**
   * \brief Default constructor.
   */
  GraspHypothesis() : cam_source_(-1) {}

  GraspHypothesis(const Eigen::Vector3d& sample, const Eigen::Matrix3d& frame, const FingerHand& finger_hand,
    const PointList& point_list_learning);

  GraspHypothesis(const Eigen::Vector3d& sample, const Eigen::Matrix3d& frame, const FingerHand& finger_hand);

  void construct(const Eigen::Matrix3d& frame, const FingerHand& finger_hand);

  void writeHandsToFile(const std::string& filename, const std::vector<GraspHypothesis>& hands);

  /**
   * \brief Print a description of the grasp hypothesis to the systen's standard output.
   */
  void print();

  /**
   * \brief Return the approach vector of the grasp.
   * \return 3x1 grasp approach vector
   */
  const Eigen::Vector3d& getApproach() const
  {
    return approach_;
  }

  /**
   * \brief Return the hand axis of the grasp.
   * \return 3x1 hand axis
   */
  const Eigen::Vector3d& getAxis() const
  {
    return axis_;
  }

  /**
   * \brief Return the binormal of the grasp.
   * \return 3x1 binormal
   */
  const Eigen::Vector3d& getBinormal() const
  {
    return binormal_;
  }

  /**
   * \brief Return whether the grasp is antipodal.
   * \return true if the grasp is antipodal, false otherwise
   */
  bool isFullAntipodal() const
  {
    return full_antipodal_;
  }

  /**
   * \brief Return the the centered grasp position at the base of the robot hand.
   * \return 3x1 grasp position at the base of the robot hand
   */
  const Eigen::Vector3d& getGraspBottom() const
  {
    return grasp_bottom_;
  }

  /**
   * \brief Return the grasp position between the end of the finger tips projected onto the back of the hand.
   * \return 3x1 grasp position between the end of the finger tips projected onto the back of the hand
   */
  const Eigen::Vector3d& getGraspSurface() const
  {
    return grasp_surface_;
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
   * \brief Return whether the grasp is indeterminate.
   * \return true if the grasp is indeterminate, false otherwise
   */
  bool isHalfAntipodal() const
  {
    return half_antipodal_;
  }

  /**
   * \brief Return the points used for training/prediction by the SVM that belong to camera #2.
   * \return the list of points used for training/prediction by the SVM that belong to camera #2
   */
  const Eigen::Matrix3Xd& getPointsForLearning() const
  {
    return points_for_learning_;
  }

  /**
   * \brief Return the normals used for training/prediction by the SVM that belong to camera #2.
   * \return the list of normals used for training/prediction by the SVM that belong to camera #2
   */
  const Eigen::Matrix3Xd& getNormalsForLearning() const
  {
    return normals_for_learning_;
  }

  /**
   * \brief Return the camera source of the sample
   * \return the camera source of the sample
   */
  int getCamSource() const
  {
    return cam_source_;
  }

  /**
   * \brief Set whether the grasp is antipodal.
   * \param b whether the grasp is antipodal
   */
  void setFullAntipodal(bool b)
  {
    full_antipodal_ = b;
  }

  /**
   * \brief Set whether the grasp is indeterminate.
   * \param b whether the grasp is indeterminate
   */
  void setHalfAntipodal(bool b)
  {
    half_antipodal_ = b;
  }

  /**
   * \brief Set the width of the object contained in the grasp.
   * \param w the width of the object contained in the grasp
   */
  void setGraspWidth(double w)
  {
    grasp_width_ = w;
  }

  /**
   * \brief Return the the centered grasp position between the fingertips of the robot hand.
   * \return 3x1 grasp position between the fingertips
   */
  const Eigen::Vector3d& getGraspTop() const
  {
    return grasp_top_;
  }

  void setGraspBottom(const Eigen::Vector3d& grasp_bottom)
  {
    grasp_bottom_ = grasp_bottom;
  }

  void setGraspSurface(const Eigen::Vector3d& grasp_surface)
  {
    grasp_surface_ = grasp_surface;
  }

  void setGraspTop(const Eigen::Vector3d& grasp_top)
  {
    grasp_top_ = grasp_top;
  }

  double getScore() const
  {
    return score_;
  }

  void setScore(double score)
  {
    score_ = score;
  }

  const Eigen::Vector3d& getSample() const
  {
    return sample_;
  }

  const Eigen::Matrix3d& getHandFrame() const
  {
    return hand_frame_;
  }

  void setApproach(const Eigen::Vector3d& approach)
  {
    approach_ = approach;
  }

  void setAxis(const Eigen::Vector3d& axis)
  {
    axis_ = axis;
  }

  void setBinormal(const Eigen::Vector3d& binormal)
  {
    binormal_ = binormal;
  }

  double getCenter() const
  {
    return center_;
  }

  double getBottom() const
  {
    return bottom_;
  }

  double getLeft() const
  {
    return left_;
  }

  double getRight() const
  {
    return right_;
  }

  double getTop() const
  {
    return top_;
  }

  int getFingerPlacementIndex () const
  {
    return finger_placement_index_;
  }


private:

  std::string vectorToString(const Eigen::VectorXd& v);

  void calculateGraspPositions(const FingerHand& finger_hand);


protected:

  int cam_source_; ///< the camera source of the sample
  Eigen::Matrix3d hand_frame_; ///< the local reference frame for this grasp hypothesis
  Eigen::Vector3d sample_; ///< the sample at which the grasp hypothesis was found
  Eigen::Vector3d axis_; ///< the hand axis
  Eigen::Vector3d approach_; ///< the grasp approach vector (orthogonal to the hand axis)
  Eigen::Vector3d binormal_; ///< the binormal (orthogonal to the hand axis and the approach vector)
  Eigen::Vector3d grasp_surface_; ///< the centered grasp position on the object surface
  Eigen::Vector3d grasp_bottom_; ///< the centered grasp position at the base of the robot hand
  Eigen::Vector3d grasp_top_; ///< the centered grasp position between the fingertips of the robot hand
  double grasp_width_; ///< the width of object enclosed by the fingers
  double score_; ///< the score given by the classifier
  bool full_antipodal_; ///< whether the grasp hypothesis is antipodal
  bool half_antipodal_; ///< whether the grasp hypothesis is indeterminate
  Eigen::Matrix3Xd points_for_learning_; ///< the points used by the classifier
  Eigen::Matrix3Xd normals_for_learning_; ///< the normals used by the classifier
  Eigen::MatrixXi camera_source_for_learning_; ///< the camera source for each point used by the classifier
  double center_, left_, right_, top_, bottom_; // positions with respect to hand frame
  int finger_placement_index_; // index of the finger placement that resuled in this grasp
};

#endif /* GRASP_HYPOTHESIS_H_ */
