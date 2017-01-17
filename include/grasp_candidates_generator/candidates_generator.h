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


#ifndef GRASP_CANDIDATES_GENERATOR_H
#define GRASP_CANDIDATES_GENERATOR_H


// System
#include <vector>


// PCL
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Custom
#include <grasp_candidates_generator/cloud_camera.h>
#include <grasp_candidates_generator/grasp.h>
#include <grasp_candidates_generator/grasp_set.h>
#include <grasp_candidates_generator/hand_search.h>
#include <grasp_candidates_generator/plot.h>


/** CandidatesGenerator class
 *
 * \brief Generate grasp candidates.
 *
 * This class generates grasp candidates by searching for possible robot hand placements in a point cloud.
 *
 */
class CandidatesGenerator
{
  public:

    struct Parameters
    {
      bool plot_normals_;
      bool plot_grasps_;
      bool remove_statistical_outliers_;
      bool voxelize_;
      int num_samples_;
      int num_threads_;
      std::vector<double> workspace_;
    };

    CandidatesGenerator(const Parameters& params, const HandSearch::Parameters& hand_search_params);

    ~CandidatesGenerator()
    {
      delete hand_search_;
    }

    void preprocessPointCloud(CloudCamera& cloud_cam);

    std::vector<Grasp> generateGraspCandidates(const CloudCamera& cloud_cam, bool use_samples = false);

    std::vector<GraspSet> generateGraspCandidateSets(const CloudCamera& cloud_cam, bool use_samples = false);

    void setNumSamples(int num_samples)
    {
      params_.num_samples_ = num_samples;
    }


  private:

    HandSearch* hand_search_;
    Plot plotter_;

    Parameters params_;
};

#endif /* GRASP_CANDIDATES_GENERATOR_H */
