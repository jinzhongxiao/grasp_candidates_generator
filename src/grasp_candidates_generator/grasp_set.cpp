#include <grasp_candidates_generator/grasp_set.h>


const int GraspSet::ROTATION_AXIS_NORMAL = 0;
const int GraspSet::ROTATION_AXIS_BINORMAL = 1;
const int GraspSet::ROTATION_AXIS_CURVATURE_AXIS = 2;


GraspSet::GraspSet() : rotation_axis_(-1)
{
  sample_.setZero();
  hands_.resize(0);
  is_valid_.resize(0);
  angles_.resize(0);
}


void GraspSet::evaluateHypotheses(const PointList& point_list, const LocalFrame& local_frame)
{
  hands_.resize(angles_.size());
  sample_ = local_frame.getSample();
  is_valid_ = Eigen::Array<bool, 1, Eigen::Dynamic>::Constant(1, angles_.size(), false);

  FingerHand finger_hand(hand_geometry_.finger_width_, hand_geometry_.outer_diameter_, hand_geometry_.depth_);
  Eigen::Matrix3d rot_binormal;

  // Set the lateral and forward axis of the robot hand frame (closing direction and grasp approach direction).
  if (rotation_axis_ == ROTATION_AXIS_CURVATURE_AXIS)
  {
    finger_hand.setLateralAxis(1);
    finger_hand.setForwardAxis(0);

    // Rotation about binormal by 180 degrees (reverses direction of normal)
    rot_binormal <<  -1.0,  0.0,  0.0,
                      0.0,  1.0,  0.0,
                      0.0,  0.0, -1.0;
  }

  // Local reference frame
  Eigen::Matrix3d local_frame_mat;
  local_frame_mat << local_frame.getNormal(), local_frame.getBinormal(), local_frame.getCurvatureAxis();

  // Evaluate grasp at each hand orientation.
  for (int i = 0; i < angles_.rows(); i++)
  {
    // Rotation about curvature axis by <angles_(i)> radians
    Eigen::Matrix3d rot;
    rot <<  cos(angles_(i)),  -1.0 * sin(angles_(i)),   0.0,
            sin(angles_(i)),  cos(angles_(i)),          0.0,
            0.0,              0.0,                      1.0;

    // Rotate points into this hand orientation.
    Eigen::Matrix3d frame_rot;
    frame_rot.noalias() = local_frame_mat * rot_binormal * rot;
    PointList point_list_frame = point_list.rotatePointList(frame_rot.transpose());

    // Crop points on hand height.
    PointList point_list_cropped = point_list_frame.cropByHandHeight(hand_geometry_.height_);

    // Evaluate finger placements for this orientation.
    finger_hand.evaluateFingers(point_list_cropped.getPoints(), hand_geometry_.init_bite_);

    // Check that there is at least one feasible 2-finger placement.
    finger_hand.evaluateHand();

    if (finger_hand.getHand().any())
    {
      // Try to move the hand as deep as possible onto the object.
      int finger_idx = finger_hand.deepenHand(point_list_cropped.getPoints(), hand_geometry_.init_bite_, hand_geometry_.depth_);

      // Calculate points in the closing region of the hand.
      std::vector<int> indices_closing = finger_hand.computePointsInClosingRegion(point_list_cropped.getPoints(), finger_idx);
      if (indices_closing.size() == 0)
      {
        continue;
      }

      // create the grasp hypothesis
      Grasp hand = createHypothesis(local_frame.getSample(), point_list_cropped, indices_closing, frame_rot,
        finger_hand);
      hands_[i] = hand;
      is_valid_[i] = true;
    }
  }
}


Eigen::Matrix3Xd GraspSet::calculateShadow(const PointList& point_list, double shadow_length) const
{
  bool measure_time = false;

  double voxel_grid_size = 0.003; // voxel size for points that fill occluded region

  double num_shadow_points = floor(shadow_length / voxel_grid_size); // number of points along each shadow vector

  // Calculate the set of cameras which see the points.
  Eigen::VectorXi camera_set = point_list.getCamSource().rowwise().sum();

  // Calculate the center point of the point neighborhood.
  Eigen::Vector3d center = point_list.getPoints().rowwise().sum();
  center /= (double) point_list.size();

  // Stores the list of all bins of the voxelized, occluded points.
  std::vector< Vector3iSet > shadows;
  shadows.resize(camera_set.rows());

  for (int i = 0; i < camera_set.rows(); i++)
  {
    if (camera_set(i) >= 1)
    {
      double t0_if = omp_get_wtime();

      // Calculate the unit vector that points from the camera position to the center of the point neighborhood.
      Eigen::Vector3d shadow_vec = center - point_list.getViewPoints().col(i);

      // Scale that vector by the shadow length.
      shadow_vec = shadow_length * shadow_vec / shadow_vec.norm();

      // Calculate occluded points.
      shadows[i] = calculateVoxelizedShadowVectorized(point_list, shadow_vec, num_shadow_points, voxel_grid_size);
    }
  }

  // Find the intersection of all sets of occluded points.
  double t0_copy= omp_get_wtime();
  Vector3iSet bins_all;

  // only one camera, no intersection
  if (shadows.size() == 1)
  {
    bins_all = shadows[0];
  }
  // more than one camera
  else
  {
    // brute force intersection (TODO: speed this up)
    double t0_intersection = omp_get_wtime();

    bins_all = shadows[0];

    for (int i = 1; i < shadows.size(); ++i)
    {
      Vector3iSet::const_iterator it;

      for (it = shadows[i].begin(); it != shadows[i].end(); it++)
      {
        bins_all.insert(*it);
      }
    }
    if (measure_time)
      std::cout << "intersection runtime: " << omp_get_wtime() - t0_intersection << "s\n";
  }

  // Convert voxels back to points.
  double t0_voxels = omp_get_wtime();
  boost::mt19937 *rng = new boost::mt19937();
  rng->seed(time(NULL));
  boost::normal_distribution<> distribution(0.0, 1.0);
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > generator(*rng, distribution);
  int i = 0;
  Vector3iSet::const_iterator it;
  Eigen::Matrix3Xd shadow_out(3, bins_all.size());

  for (it = bins_all.begin(); it != bins_all.end(); it++)
  {
    shadow_out.col(i) = (*it).cast<double>() * voxel_grid_size;
    shadow_out.col(i)(0) += generator() * voxel_grid_size * 0.3;
    shadow_out.col(i)(1) += generator() * voxel_grid_size * 0.3;
    shadow_out.col(i)(2) += generator() * voxel_grid_size * 0.3;
    i++;
  }
  if (measure_time)
    std::cout << "voxels-to-points runtime: " << omp_get_wtime() - t0_voxels << "s\n";

  return shadow_out;
}


Vector3iSet GraspSet::calculateVoxelizedShadowVectorized(const PointList& point_list, const Eigen::Vector3d& shadow_vec,
  int num_shadow_points, double voxel_grid_size) const
{
//  double t0_shadow = omp_get_wtime();

  Eigen::internal::scalar_normal_dist_op<double> rand_uni; // Uniform functor
  Eigen::internal::scalar_normal_dist_op<double>::rng.seed(42u); // Seed the rng

//  Eigen::MatrixXd X = point_list.getPoints().replicate(num_shadow_points,1);
//  Eigen::MatrixXd V = shadow_vec.replicate(num_shadow_points, X.cols());
//  Eigen::VectorXd r = Eigen::VectorXd::NullaryExpr(num_shadow_points * X.cols(), rand_uni);
//  Eigen::MatrixXi S = ((X + V * r.asDiagonal()) / voxel_grid_size).unaryExpr(std::ptr_fun(floor)).cast<int>();

  bool measure_time = false;

  // works but slower than code below
  bool skip = true;

  if (skip==false)
  {
    double t0_mat = omp_get_wtime();
    Eigen::MatrixXd X = point_list.getPoints().replicate(num_shadow_points,1);
    Eigen::MatrixXd V = shadow_vec.replicate(num_shadow_points, X.cols());
    Eigen::Matrix3Xd r = Eigen::RowVectorXd::NullaryExpr(num_shadow_points * X.cols(), rand_uni).replicate(3,1);
    Eigen::MatrixXd R = Eigen::Map<Eigen::MatrixXd>(r.data(), num_shadow_points*3, X.cols());

    Eigen::MatrixXi S = ((X + R.cwiseProduct(V)) / voxel_grid_size).unaryExpr(std::ptr_fun(floor)).cast<int>();
    std::cout << "matrix calc runtime: " << omp_get_wtime() - t0_mat << "\n";

  //
  //  std::cout << "#pts: " << point_list.getPoints().cols() << "\n";
  //  std::cout << "X: " << X.rows() << " x " <<  X.cols() << "\n";
  //  std::cout << "V: " << V.rows() << " x " <<  V.cols() << "\n";
  //  std::cout << "r: " << r.rows() << " x " <<  r.cols() << "\n";
  ////  std::cout << "R: " << R.rows() << " x " <<  R.cols() << "\n";
  //  std::cout << "------------\n";
  //  std::cout << point_list.getPoints().col(0) << "\n";
  //
    double t0_set = omp_get_wtime();
    Vector3iSet shadow_set;
    shadow_set.reserve(point_list.size() * num_shadow_points);

    for(int i = 0; i < point_list.size() * num_shadow_points; i++)
    {
      int row = i % num_shadow_points;
      int col = i / num_shadow_points;
      shadow_set.insert(S.block(row, col, 3, 1));
    }
    std::cout << "set constr runtime: " << omp_get_wtime() - t0_set << "\n";
    std::cout << "-----------------------------------------------------\n";
  }

  // 40% faster than loop-based version
  double t0_shadow = omp_get_wtime();
  Eigen::Matrix3Xi shadows_mat(3, point_list.size() * num_shadow_points);
  const Eigen::Matrix3Xd shadow_vec_mat = shadow_vec.replicate(1,num_shadow_points);

  for(int i = 0; i < point_list.size(); i++)
  {
    const Eigen::VectorXd r = Eigen::VectorXd::NullaryExpr(num_shadow_points, rand_uni);
    shadows_mat.block(0, i*num_shadow_points , 3, num_shadow_points)
          = (((point_list.getPoints().col(i).replicate(1,num_shadow_points)
//              + Eigen::RowVectorXd::NullaryExpr(num_shadow_points, rand_uni).replicate(3,1).cwiseProduct(shadow_vec.replicate(1, num_shadow_points))
//              + shadow_vec.replicate(1,num_shadow_points) * r.asDiagonal()
                + shadow_vec_mat * r.asDiagonal()
              ) / voxel_grid_size).unaryViewExpr(std::ptr_fun(floor))).cast<int>();
  }
  if (measure_time)
    std::cout << "Matrix calculation runtime: " << omp_get_wtime() - t0_shadow << "\n";

  double t0_set = omp_get_wtime();
  Vector3iSet shadow_set;
//  shadow_set.clear();
//  shadow_set.reserve(shadows_mat.cols()); // seems to take up a lot of time

  for(int i = 0; i < shadows_mat.cols(); i++)
  {
    shadow_set.insert(shadows_mat.col(i));
  }
  if (measure_time)
    std::cout << "Set construction runtime: " << omp_get_wtime() - t0_set << "\n";

//  (shadows_mat.data()row(0)).data();
//  std::vector<int> x(shadows_mat.row(0).data(), shadows_mat.row(0).data() + shadows_mat.cols());
//  std::vector<int> y(shadows_mat.row(1).data(), shadows_mat.row(1).data() + shadows_mat.cols());
//  std::vector<int> z(shadows_mat.row(2).data(), shadows_mat.row(2).data() + shadows_mat.cols());

//  std::set<int> x(shadows_mat.row(0).data(), shadows_mat.row(0).data() + shadows_mat.cols());
//  std::set<int> y(shadows_mat.row(1).data(), shadows_mat.row(1).data() + shadows_mat.cols());
//  std::set<int> z(shadows_mat.row(2).data(), shadows_mat.row(2).data() + shadows_mat.cols());

  if (skip == false)
  {
    t0_set = omp_get_wtime();
    boost::unordered_set<int> x(shadows_mat.row(0).data(), shadows_mat.row(0).data() + shadows_mat.cols());
    boost::unordered_set<int> y(shadows_mat.row(1).data(), shadows_mat.row(1).data() + shadows_mat.cols());
    boost::unordered_set<int> z(shadows_mat.row(2).data(), shadows_mat.row(2).data() + shadows_mat.cols());
    std::cout << "Set construction runtime NEW: " << omp_get_wtime() - t0_set << "\n";
  }

//  const Eigen::Matrix3Xd shadow_vec_mat = shadow_vec.replicate(1, num_shadow_points);

//  for(int i = 0; i < point_list.size(); i++)
//  {
//    const Eigen::Vector3d& p = point_list.getPoints().col(i);
//    Eigen::Matrix3Xd uni = ((Eigen::VectorXd::Random(num_shadow_points) + Eigen::VectorXd::Ones(num_shadow_points)) / 2.0).replicate(3,1);
//    Eigen::Matrix3Xd shadow = p.replicate(1, num_shadow_points) + uni.cwiseProduct(shadow_vec.replicate(1, num_shadow_points));
//    shadows_mat.block(0, i*num_shadow_points , 3, num_shadow_points) = (shadow / voxel_grid_size).unaryExpr(std::ptr_fun(floor)).cast<int>();

//    const Eigen::Vector3d& p = point_list.getPoints().col(i);
//    shadows_mat.block(0, i*num_shadow_points , 3, num_shadow_points)
//      = (((point_list.getPoints().col(i).replicate(1,num_shadow_points)
//          + Eigen::RowVectorXd::NullaryExpr(num_shadow_points, rand_uni).replicate(3,1).cwiseProduct(shadow_vec.replicate(1, num_shadow_points))
//          ) / voxel_grid_size).unaryExpr(std::ptr_fun(floor))).cast<int>();
//        + ((Eigen::VectorXd::Random(num_shadow_points) + Eigen::VectorXd::Ones(num_shadow_points)) / 2.0).replicate(3,1).cwiseProduct(shadow_vec.replicate(1, num_shadow_points))
//        + (0.5 * Eigen::VectorXd::Random(num_shadow_points) + 0.5 * Eigen::VectorXd::Ones(num_shadow_points)).replicate(3,1).cwiseProduct(shadow_vec.replicate(1, num_shadow_points))
//        + Eigen::VectorXd::NullaryExpr(num_shadow_points, rand_uni).transpose().replicate(3,1).cwiseProduct(shadow_vec.replicate(1, num_shadow_points))

//        + Eigen::RowVectorXd::NullaryExpr(num_shadow_points, rand_uni).replicate(3,1).cwiseProduct(shadow_vec_mat)
//        + (Eigen::VectorXd::Random(num_shadow_points) * 0.5 + Eigen::VectorXd::Constant(0.5, num_shadow_points)).replicate(3,1).cwiseProduct(shadow_vec.replicate(1, num_shadow_points))

//    temp.unaryExpr(std::ptr_fun(floor));
//    shadows_mat.block(0, i*num_shadow_points , 3, num_shadow_points) = temp;
//  }
//  std::cout << "Shadow calculation runtime: " << omp_get_wtime() - t0_shadow << "\n";

  return shadow_set;
}


Vector3iSet GraspSet::calculateVoxelizedShadowLoop(const PointList& point_list, const Eigen::Vector3d& shadow_vec,
  int num_shadow_points, double voxel_grid_size) const
{
  double t0_shadow = omp_get_wtime();

  // Create generator for uniform random numbers.
  boost::mt11213b generator(42u);
  boost::uniform_real<> uni_dist(0.0, 1.0);
  boost::variate_generator<boost::mt11213b&, boost::uniform_real<> > uni(generator, uni_dist);

  Vector3iSet shadow;
  shadow.reserve(point_list.size() * num_shadow_points);

  for(int i = 0; i < point_list.size(); i++)
  {
    for(int j = 0; j < num_shadow_points; j++)
    {
      Eigen::Vector3i shadow_point = floorVector((point_list.getPoints().col(i) + uni() * shadow_vec) / voxel_grid_size);
      shadow.insert(shadow_point);
    }
  }

//  std::cout << "Shadow calculation runtime: " << omp_get_wtime() - t0_shadow << "\n";

  return shadow;
}


Eigen::VectorXi GraspSet::floorVector(const Eigen::VectorXd& a) const
{
  Eigen::VectorXi b(a.size());

  for (int i = 0; i < b.size(); i++)
  {
    b(i) = floor((double) a(i));
  }

  return b;
}


Grasp GraspSet::createHypothesis(const Eigen::Vector3d& sample, const PointList& point_list,
  const std::vector<int>& indices_learning, const Eigen::Matrix3d& hand_frame, const FingerHand& finger_hand) const
{
  // extract data for classification
  PointList point_list_learning = point_list.slice(indices_learning);

  // calculate grasp width (hand opening width)
  double width = point_list_learning.getPoints().row(0).maxCoeff() - point_list_learning.getPoints().row(0).minCoeff();

  Grasp hand(sample, hand_frame, finger_hand, width);

  // evaluate if the grasp is antipodal
  labelHypothesis(point_list, finger_hand, hand);

  return hand;
}


void GraspSet::labelHypothesis(const PointList& point_list, const FingerHand& finger_hand, Grasp& hand)
  const
{
  Antipodal antipodal;
  int label = antipodal.evaluateGrasp(point_list, 0.003, finger_hand.getLateralAxis(), finger_hand.getForwardAxis(),
    rotation_axis_);
  hand.setHalfAntipodal(label == Antipodal::HALF_GRASP || label == Antipodal::FULL_GRASP);
  hand.setFullAntipodal(label == Antipodal::FULL_GRASP);
}
