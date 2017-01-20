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


#ifndef GRASP_SET_H_
#define GRASP_SET_H_

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
#include <grasp_candidates_generator/grasp.h>
#include <grasp_candidates_generator/local_frame.h>
#include <grasp_candidates_generator/point_list.h>


// The hash function below is necessary for boost's unordered set.
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


class GraspSet
{
  public:

    /** robot hand geometry */
    struct HandGeometry
    {
      double finger_width_; ///< the width of the robot hand fingers
      double outer_diameter_; ///< the maximum robot hand aperture
      double depth_; ///< the hand depth (length of fingers)
      double height_; ///< the hand extends plus/minus this value along the hand axis
      double init_bite_; ///< the minimum object height

      HandGeometry() : finger_width_(0.0), outer_diameter_(0.0), depth_(0.0), height_(0.0), init_bite_(0.0) { }

      HandGeometry(double finger_width, double outer_diameter, double hand_depth, double hand_height,
        double init_bite)
        : finger_width_(finger_width), outer_diameter_(outer_diameter), depth_(hand_depth),
          height_(hand_height), init_bite_(init_bite) {  }
    };

    GraspSet();

    GraspSet(const HandGeometry& hand_geometry, const Eigen::VectorXd& angles, int rotation_axis)
      : hand_geometry_(hand_geometry), angles_(angles), rotation_axis_(rotation_axis) { }

    void evaluateHypotheses(const PointList& point_list, const LocalFrame& local_frame);

    Eigen::Matrix3Xd calculateShadow(const PointList& point_list, double shadow_length) const;

    const std::vector<Grasp>& getHypotheses() const
    {
      return hands_;
    }

    void setHands(const std::vector<Grasp>& hands)
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

    Grasp createHypothesis(const Eigen::Vector3d& sample, const PointList& point_list,
      const std::vector<int>& indices_learning, const Eigen::Matrix3d& hand_frame, const FingerHand& finger_hand) const;

    void labelHypothesis(const PointList& point_list, const FingerHand& finger_hand, Grasp& hand) const;

    Eigen::Vector3d sample_;
    std::vector<Grasp> hands_;
    Eigen::Array<bool, 1, Eigen::Dynamic> is_valid_;
    Eigen::VectorXd angles_; ///< the hand orientations to consider in the local search

    HandGeometry hand_geometry_; ///< the robot hand geometry
    int rotation_axis_; ///< the axis about which the hand frame is rotated to generate different orientations

    /** constants for rotation axis */
    static const int ROTATION_AXIS_NORMAL; ///< normal axis of local reference frame
    static const int ROTATION_AXIS_BINORMAL; ///< binormal axis of local reference frame
    static const int ROTATION_AXIS_CURVATURE_AXIS; ///< curvature axis of local reference frame
};

#endif /* GRASP_SET_H_ */
