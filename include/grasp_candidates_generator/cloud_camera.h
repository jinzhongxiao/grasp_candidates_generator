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


#ifndef CLOUD_CAMERA_H_
#define CLOUD_CAMERA_H_

#include <algorithm>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include <Eigen/Dense>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <grasp_candidates_generator/eigen_utils.h>


typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudNormal;


/** CloudCamera class
 *
 * \brief A point cloud with camera sources and surface normals
 * 
 * This class stores a point cloud with camera sources and surface normals for each point in the point cloud and the
 * view point of the camera from which the cloud was observed.
 * 
*/
class CloudCamera
{
public:

  /**
   * \brief Comparator for checking uniqueness of two 3D-vectors.
  */
  struct UniqueVectorComparator
  {
    /**
     * \brief Compares two 3D-vectors for uniqueness.
     * \param a the first 3D-vector
     * \param b the second 3D-vector
     * \return true if they differ in at least one element, false if all elements are equal
    */
    bool operator ()(const Eigen::Vector3i& a, const Eigen::Vector3i& b)
    {
//      return a(0) < b(0) && a(1) < b(1) && a(2) < b(2);

      for (int i = 0; i < a.size(); i++)
      {
        if (a(i) != b(i))
        {
          return a(i) < b(i);
        }
      }

      return false;
    }
  };

  struct UniqueVectorFirstThreeElementsComparator
  {
    /**
     * \brief Compares two 3D-vectors for uniqueness (ignores the last element).
     * \param a the first 3D-vector
     * \param b the second 3D-vector
     * \return true if they differ in at least one element, false if all elements are equal
    */
    bool operator ()(const Eigen::Vector4i& a, const Eigen::Vector4i& b)
    {
      for (int i = 0; i < a.size() - 1; i++)
      {
        if (a(i) != b(i))
        {
          return true;
        }
      }

      return false;
    }
  };

  CloudCamera();

  CloudCamera(const PointCloudRGB::Ptr& cloud, const Eigen::MatrixXi& camera_source,
    const Eigen::Matrix3Xd& view_points);

  CloudCamera(const PointCloudNormal::Ptr& cloud, const Eigen::MatrixXi& camera_source,
    const Eigen::Matrix3Xd& view_points);

  CloudCamera(const PointCloudNormal::Ptr& cloud, int size_left_cloud, const Eigen::Matrix3Xd& view_points);

  CloudCamera(const PointCloudRGB::Ptr& cloud, int size_left_cloud, const Eigen::Matrix3Xd& view_points);

  CloudCamera(const std::string& filename, const Eigen::Matrix3Xd& view_points);

  CloudCamera(const std::string& filename_left, const std::string& filename_right,
    const Eigen::Matrix3Xd& view_points);

  /**
   * \brief Filter out points in the point cloud that lie outside the workspace dimensions.
   * \param[in] workspace a 6-D vector containing the workspace limits: [minX, maxX, minY, maxY, minZ, maxZ]
  */
  void filterWorkspace(const std::vector<double>& workspace);

  void filterSamples(const std::vector<double>& workspace);

  /**
   * \brief Voxelize the point cloud and keep track of the camera source for each voxel.
   * \param[in] cell_size the size of each voxel
  */
  void voxelizeCloud(double cell_size);

  /**
   * \brief Subsample the point cloud according to the uniform distribution.
   * \param[in] num_samples the number of samples to draw from the point cloud
  */
  void subsampleUniformly(int num_samples);

  void subsampleSamples(int num_samples);

  void calculateNormals(int num_threads);

  void setNormalsFromFile(const std::string& filename);

  void writeNormalsToFile(const std::string& filename, const Eigen::Matrix3Xd& normals);

  const Eigen::MatrixXi& getCameraSource() const
  {
    return camera_source_;
  }

  const PointCloudRGB::Ptr& getCloudProcessed() const
  {
    return cloud_processed_;
  }

  const PointCloudRGB::Ptr& getCloudOriginal() const
  {
    return cloud_original_;
  }

  const std::vector<int>& getSampleIndices() const
  {
    return sample_indices_;
  }

  const Eigen::Matrix3Xd& getNormals() const
  {
    return normals_;
  }

  const Eigen::Matrix3Xd& getSamples() const
  {
    return samples_;
  }

  void setSampleIndices(const std::vector<int>& sampleIndices)
  {
    sample_indices_ = sampleIndices;
  }

  void setSamples(const Eigen::Matrix3Xd& samples);

  void setNormals (const Eigen::Matrix3Xd& normals)
  {
    normals_ = normals;
  }

  const Eigen::Matrix3Xd& getViewPoints() const
  {
    return view_points_;
  }

  void setViewPoints(const Eigen::Matrix3Xd& view_points)
  {
    view_points_ = view_points;
  }


private:

  PointCloudRGB::Ptr loadPointCloudFromFile(const std::string& filename) const;

  PointCloudRGB::Ptr cloud_processed_; ///< the (processed) point cloud
  PointCloudRGB::Ptr cloud_original_; ///< the original point cloud
  Eigen::MatrixXi camera_source_; ///< binary matrix: (i,j) = 1 if point j is seen by camera i
  Eigen::Matrix3Xd normals_; ///< the surface normal for each point in the point cloud
  std::vector<int> sample_indices_; ///< the indices into the point cloud used to sample grasp hypotheses
  Eigen::Matrix3Xd samples_; ///< the samples used for finding grasp hypotheses
  Eigen::Matrix3Xd view_points_; ///< the viewpoints of the camera on the cloud
};

#endif /* CLOUD_CAMERA_H_ */
