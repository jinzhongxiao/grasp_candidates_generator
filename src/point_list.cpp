#include <grasp_candidates_generator/point_list.h>


PointList::PointList(int size, int num_cams)
{
  points_.resize(3, size);
  normals_.resize(3, size);
  cam_source_.resize(num_cams, size);
  view_points_.resize(3, num_cams);
}


PointList PointList::slice(const std::vector<int>& indices) const
{
  Eigen::Matrix3Xd points_out = sliceMatrix(points_, indices);
  Eigen::Matrix3Xd normals_out = sliceMatrix(normals_, indices);
  Eigen::MatrixXi cam_source_out = sliceMatrix(cam_source_, indices);

  return PointList(points_out, normals_out, cam_source_out, view_points_);
}


Eigen::Matrix3Xd PointList::sliceMatrix(const Eigen::Matrix3Xd& mat, const std::vector<int>& indices) const
{
  Eigen::Matrix3Xd mat_out(3, indices.size());

  for (int j = 0; j < indices.size(); j++)
  {
    mat_out.col(j) = mat.col(indices[j]);
  }

  return mat_out;
}


Eigen::MatrixXi PointList::sliceMatrix(const Eigen::MatrixXi& mat, const std::vector<int>& indices) const
{
  Eigen::MatrixXi mat_out(mat.rows(), indices.size());

  for (int j = 0; j < indices.size(); j++)
  {
    mat_out.col(j) = mat.col(indices[j]);
  }

  return mat_out;
}


PointList PointList::transformToHandFrame(const Eigen::Vector3d& centroid, const Eigen::Matrix3d& rotation) const
{
  Eigen::Matrix3Xd points_centered = points_ - centroid.replicate(1, size());
  points_centered = rotation * points_centered;
  Eigen::Matrix3Xd normals(3, points_centered.cols());
  normals.noalias() = rotation * normals_;

  return PointList(points_centered, normals, cam_source_, view_points_);
}


PointList PointList::rotatePointList(const Eigen::Matrix3d& rotation) const
{
  Eigen::Matrix3Xd points(3, points_.cols());
  Eigen::Matrix3Xd normals(3, points_.cols());
  points.noalias() = rotation * points_;
  normals.noalias() = rotation * normals_;

  return PointList(points, normals, cam_source_, view_points_);
}


PointList PointList::cropByHandHeight(double height, int dim) const
{
  std::vector<int> indices(size());
  int k = 0;
  for (int i = 0; i < size(); i++)
  {
    if (points_(dim, i) > -1.0 * height && points_(dim, i) < height)
    {
      indices[k] = i;
      k++;
    }
  }

  return slice(indices);
}
