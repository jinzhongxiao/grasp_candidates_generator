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


#ifndef HYPOTHESIS_SET_H_
#define HYPOTHESIS_SET_H_

// System
#include <vector>

// Boost
#include <boost/functional/hash.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/unordered_set.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

// Eigen
#include <Eigen/Dense>

// Custom
#include <grasp_candidates_generator/antipodal.h>
#include <grasp_candidates_generator/finger_hand.h>
#include <grasp_candidates_generator/grasp_hypothesis.h>
#include <grasp_candidates_generator/local_frame.h>
#include <grasp_candidates_generator/point_list.h>


// The operator and the hash function below are necessary for boost's unordered set.

//bool operator ==(const Eigen::Vector3i& a, const Eigen::Vector3i& b)
//{
//  if (a(0) < b(0)) return true;
//
//  if (b(0) < a(0)) return false;
//
//  if (a(1) < b(1)) return true;
//
//  if (b(1) < a(1)) return false;
//
//  return a(2) < b(2);
//}


namespace boost
{
template <>
struct hash<Eigen::Vector3i>
{
    size_t operator()(Eigen::Vector3i const& v) const
    {
      std::size_t seed = 0;

      for (int i = 0; i < v.size(); i++)
      {
        boost::hash_combine(seed, v(i));
      }

      return seed;
    }
};
}


typedef boost::unordered_set<Eigen::Vector3i, boost::hash<Eigen::Vector3i> > Vector3iSet;


class HypothesisSet
{
  public:

    HypothesisSet();

    HypothesisSet(double finger_width, double hand_outer_diameter, double hand_depth, double hand_height,
      double init_bite, int rotation_axis)
      : finger_width_(finger_width), hand_outer_diameter_(hand_outer_diameter), hand_depth_(hand_depth),
        hand_height_(hand_height), init_bite_(init_bite), rotation_axis_(rotation_axis) { }

    void evaluateHypotheses(const PointList& point_list, const LocalFrame& local_frame, const Eigen::VectorXd& angles);

    Eigen::Matrix3Xd calculateShadow(const PointList& point_list, double shadow_length) const;

    const std::vector<GraspHypothesis>& getHypotheses() const
    {
      return hands_;
    }

    void setHands(const std::vector<GraspHypothesis>& hands)
    {
      hands_ = hands;
    }

    const Eigen::Vector3d& getSample() const
    {
      return sample_;
    }

    void setSample(const Eigen::Vector3d& sample)
    {
      sample_ = sample;
    }

    const Eigen::Array<bool, 1, Eigen::Dynamic>& getIsValid() const
    {
      return is_valid_;
    }

    void setIsValid(const Eigen::Array<bool, 1, Eigen::Dynamic>& isValid)
    {
      is_valid_ = isValid;
    }

    void setIsValidWithIndex(int idx, bool val)
    {
      is_valid_[idx] = val;
    }


  private:

    Vector3iSet calculateVoxelizedShadow(const PointList& point_list, const Eigen::Vector3d& shadow_vec,
      int num_shadow_points, double voxel_grid_size) const;

    Eigen::VectorXi floorVector(const Eigen::VectorXd& a) const;

    GraspHypothesis createHypothesis(const Eigen::Vector3d& sample, const PointList& point_list,
      const std::vector<int>& indices_learning, const Eigen::Matrix3d& hand_frame, const FingerHand& finger_hand) const;

    void labelHypothesis(const PointList& point_list, const FingerHand& finger_hand, GraspHypothesis& hand) const;

    Eigen::Vector3d sample_;
    std::vector<GraspHypothesis> hands_;
    Eigen::Array<bool, 1, Eigen::Dynamic> is_valid_;

    /** robot hand geometry */
    double finger_width_; ///< the width of the robot hand fingers
    double hand_outer_diameter_; ///< the maximum robot hand aperture
    double hand_depth_; ///< the hand depth (length of fingers)
    double hand_height_; ///< the hand extends plus/minus this value along the hand axis
    double init_bite_; ///< the minimum object height

    int rotation_axis_; ///< the axis about which the hand frame is rotated to generate different orientations

    /** constants for rotation axis */
    static const int ROTATION_AXIS_NORMAL; ///< normal axis of local reference frame
    static const int ROTATION_AXIS_BINORMAL; ///< binormal axis of local reference frame
    static const int ROTATION_AXIS_CURVATURE_AXIS; ///< curvature axis of local reference frame
};

#endif /* HYPOTHESIS_SET_H_ */
