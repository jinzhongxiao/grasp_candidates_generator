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


#ifndef POINT_LIST_H_
#define POINT_LIST_H_


#include <Eigen/Dense>

#include <vector>


class PointList
{
  public:

    PointList() { }

    PointList(const Eigen::Matrix3Xd& points, const Eigen::Matrix3Xd& normals, const Eigen::MatrixXi& cam_source,
      const Eigen::Matrix3Xd& view_points)
      : points_(points), normals_(normals), cam_source_(cam_source), view_points_(view_points) { }

    PointList(int size, int num_cams);

    PointList sliceMatrix(const std::vector<int>& indices) const;

    PointList createPermutation(const Eigen::Vector3i& order) const;

    Eigen::Matrix3Xd sliceMatrix(const Eigen::Matrix3Xd& mat, const std::vector<int>& indices) const;

    Eigen::MatrixXi sliceMatrix(const Eigen::MatrixXi& mat, const std::vector<int>& indices) const;

    PointList transformToHandFrame(const Eigen::Vector3d& centroid, const Eigen::Matrix3d& rotation) const;

    const Eigen::MatrixXi& getCamSource() const
    {
      return cam_source_;
    }

    void setCamSource(const Eigen::MatrixXi& cam_source)
    {
      cam_source_ = cam_source;
    }

    const Eigen::Matrix3Xd& getNormals() const
    {
      return normals_;
    }

    void setNormals(const Eigen::Matrix3Xd& normals)
    {
      normals_ = normals;
    }

    const Eigen::Matrix3Xd& getPoints() const
    {
      return points_;
    }

    void setPoints(const Eigen::Matrix3Xd& points)
    {
      points_ = points;
    }

    int size() const
    {
      return points_.cols();
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

    Eigen::Matrix3Xd points_;
    Eigen::Matrix3Xd normals_;
    Eigen::MatrixXi cam_source_;
    Eigen::Matrix3Xd view_points_;
};


#endif /* POINT_LIST_H_ */
