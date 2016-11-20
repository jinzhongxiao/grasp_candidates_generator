#include <grasp_candidates_generator/point_list.h>


PointList::PointList(int size, int num_cams)
{
  points_.resize(3, size);
  normals_.resize(3, size);
  cam_source_.resize(num_cams, size);
}


PointList PointList::sliceMatrix(const std::vector<int>& indices) const
{
  Eigen::Matrix3Xd points_out = sliceMatrix(points_, indices);
  Eigen::Matrix3Xd normals_out = sliceMatrix(normals_, indices);
  Eigen::MatrixXi cam_source_out = sliceMatrix(cam_source_, indices);
  Eigen::Matrix3Xd view_points_out = sliceMatrix(view_points_, indices);

  return PointList(points_out, normals_out, cam_source_out, view_points_out);
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


PointList PointList::createPermutation(const Eigen::Vector3i& order)
{
  Eigen::Matrix3Xd points_out(3, size());
  Eigen::Matrix3Xd normals_out(3, size());

  for (int i = 0; i < 3; i++)
  {
    points_out.row(i) = points_.row(order(i));
    normals_out.row(i) = normals_.row(order(i));
  }

  return PointList(points_out, normals_out, cam_source_, view_points_);
}
