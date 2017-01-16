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


#ifndef HAND_SEARCH_H
#define HAND_SEARCH_H


#include <Eigen/Dense>

#include <pcl/filters/random_sample.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_cloud.h>

#include <omp.h>

#include <grasp_candidates_generator/antipodal.h>
#include <grasp_candidates_generator/cloud_camera.h>
#include <grasp_candidates_generator/finger_hand.h>
#include <grasp_candidates_generator/frame_estimator.h>
#include <grasp_candidates_generator/grasp_hypothesis.h>
#include <grasp_candidates_generator/local_frame.h>
#include <grasp_candidates_generator/plot.h>
#include <grasp_candidates_generator/point_list.h>


typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;


/** HandSearch class
 *
 * \brief Search for grasp hypotheses.
 * 
 * This class searches for grasp hypotheses in a point cloud by first calculating a local reference frame for a small
 * point neighborhood, and then finding geometrically feasible grasp hypotheses for a larger point neighborhood. It
 * can also estimate whether the grasp is antipodal from the normals of the point neighborhood.
 * 
 */
class HandSearch
{
public:

  /**
   * \brief Parameters for the robot hand model.
   */
  struct Parameters
  {
    /** local reference frame estimation parameters */
    double nn_radius_frames_; ///< local reference frame radius for point neighborhood search

    /** grasp hypotheses generation */
    int num_threads_; ///< the number of CPU threads to be used for the hypothesis generation
    int num_samples_; ///< the number of samples drawn from the point clouds;
    Eigen::Matrix4d cam_tf_left_; ///< pose of the left camera
    Eigen::Matrix4d cam_tf_right_; ///< pose of the right camera
    int num_orientations_; ///< number of hand orientations to evaluate
    int rotation_axis_; ///< the rotation axis about which different hand orientations are generated

    /** robot hand geometry */
    double finger_width_; ///< the width of the robot hand fingers
    double hand_outer_diameter_; ///< the maximum robot hand aperture
    double hand_depth_; ///< the hand depth (length of fingers)
    double hand_height_; ///< the hand extends plus/minus this value along the hand axis
    double init_bite_; ///< the minimum object height
  };

  HandSearch() : plots_samples_(false), plots_local_axes_(false), plots_camera_sources_(false)
  { }

  HandSearch(Parameters params) : params_(params), plots_samples_(false), plots_local_axes_(false),
    plots_camera_sources_(false)
  { }

  std::vector<GraspHypothesis> generateHypotheses(const CloudCamera& cloud_cam, int antipodal_mode, bool use_samples,
    bool forces_PSD = false, bool plots_normals = false, bool plots_samples = false) const;

  std::vector<GraspHypothesis> reevaluateHypotheses(const CloudCamera& cloud_cam,
    const std::vector<GraspHypothesis>& grasps, bool plot_samples = false) const;

  void setParameters(const Parameters& params)
  {
    params_ = params;
  }


private:

  std::vector<GraspHypothesis> evaluateHands(const CloudCamera& cloud_cam, const std::vector<LocalFrame>& frames,
    const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree) const;

  std::vector<GraspHypothesis> evaluateHand(const pcl::PointXYZRGBA& sample, const PointList& point_list,
    const LocalFrame& local_frame, const Eigen::VectorXd& angles) const;

  bool reevaluateHypothesis(const PointList& point_list, const GraspHypothesis& hand, FingerHand& finger_hand,
    PointList& point_list_cropped) const;

  int labelHypothesis(const PointList& point_list, FingerHand& finger_hand) const;

  PointList cropByHandHeight(const PointList& points_in, double height, int dim = 2) const;

  pcl::PointXYZRGBA eigenVectorToPcl(const Eigen::Vector3d& v) const;

  GraspHypothesis createGraspHypothesis(const Eigen::Vector3d& sample, const PointList& point_list,
    const std::vector<int>& indices_learning, const Eigen::Matrix3d& hand_frame, const FingerHand& finger_hand) const;

  Parameters params_; ///< parameters for the hand search

  Eigen::Matrix3Xd cloud_normals_; ///< a 3xn matrix containing the normals for points in the point cloud
  Plot plot_; ///< plot object for visualization of search results

  /** plotting parameters (optional, not read in from config file) **/
  bool plots_samples_; ///< are the samples drawn from the point cloud plotted?
  bool plots_camera_sources_; ///< is the camera source for each point in the point cloud plotted?
  bool plots_local_axes_; ///< are the local axes estimated for each point neighborhood plotted?

  /** constants for rotation axis */
  static const int ROTATION_AXIS_NORMAL; ///< normal axis of local reference frame
  static const int ROTATION_AXIS_BINORMAL; ///< binormal axis of local reference frame
  static const int ROTATION_AXIS_CURVATURE_AXIS; ///< curvature axis of local reference frame
};

#endif /* HAND_SEARCH_H */ 
