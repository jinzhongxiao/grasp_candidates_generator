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

#ifndef FRAME_ESTIMATOR_H
#define FRAME_ESTIMATOR_H


#include <vector>

#include <Eigen/Dense>

#include <pcl/kdtree/kdtree.h>

#include <omp.h>

#include <grasp_candidates_generator/cloud_camera.h>
#include <grasp_candidates_generator/local_frame.h>


typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;


class FrameEstimator
{
  public:

    FrameEstimator(int num_threads) : num_threads_(num_threads) { }

    std::vector<LocalFrame> calculateLocalFrames(const CloudCamera& cloud_cam, const std::vector<int>& indices,
      double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree) const;

    std::vector<LocalFrame> calculateLocalFrames(const CloudCamera& cloud_cam, const Eigen::Matrix3Xd& samples,
      double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree) const;

    LocalFrame* calculateFrame(const Eigen::Matrix3Xd& normals, const Eigen::Vector3d& sample, double radius,
      const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree) const;


  private:

    pcl::PointXYZRGBA eigenVectorToPcl(const Eigen::Vector3d& v) const;

    int num_threads_;
};

#endif /* FRAME_ESTIMATOR_H */
