#include <grasp_candidates_generator/hand_search.h>


const int HandSearch::ROTATION_AXIS_NORMAL = 0;
const int HandSearch::ROTATION_AXIS_BINORMAL = 1;
const int HandSearch::ROTATION_AXIS_CURVATURE_AXIS = 2;


std::vector<GraspHypothesis> HandSearch::generateHypotheses(const CloudCamera& cloud_cam, int antipodal_mode,
  bool use_samples, bool forces_PSD, bool plots_normals, bool plots_samples)
{
  if (rotation_axis_ < 0 || rotation_axis_ > 2)
  {
    std::cout << "Parameter <rotation_axis> is not set!\n";
    std::vector<GraspHypothesis> empty(0);
    return empty;
  }

  double t0_total = omp_get_wtime();

  // create KdTree for neighborhood search
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud);

  cloud_normals_.resize(3, cloud->size());
  cloud_normals_.setZero(3, cloud->size());

  // 1. Calculate surface normals for all points (optional).
  bool has_normals = false;
  cloud_normals_ = cloud_cam.getNormals();
  if (plots_normals)
  {
    std::cout << "Plotting normals ...\n";
    plot_.plotNormals(cloud, cloud_normals_);
  }

  if (plots_samples)
  {
    std::cout << "Plotting samples ...\n";
    plot_.plotSamples(cloud_cam.getSampleIndices(), cloud_cam.getCloudProcessed());
  }

  // 2. Estimate local reference frames.
  std::cout << "Estimating local reference frames ...\n";
  std::vector<LocalFrame> frames;
  FrameEstimator frame_estimator(num_threads_);
  if (use_samples)
    frames = frame_estimator.calculateLocalFrames(cloud_cam, cloud_cam.getSamples(), nn_radius_taubin_, kdtree);
  else
    frames = frame_estimator.calculateLocalFrames(cloud_cam, cloud_cam.getSampleIndices(), nn_radius_taubin_, kdtree);
  if (plots_local_axes_)
    plot_.plotLocalAxes(frames, cloud_cam.getCloudOriginal());

  // 3. Evaluate possible hand placements.
  std::cout << "Finding hand poses ...\n";
  std::vector<GraspHypothesis> hypotheses = evaluateHands(cloud_cam, frames, kdtree);

  std::cout << "====> HAND SEARCH TIME: " << omp_get_wtime() - t0_total << std::endl;

  return hypotheses;
}


std::vector<GraspHypothesis> HandSearch::reevaluateHypotheses(const CloudCamera& cloud_cam,
  const std::vector<GraspHypothesis>& grasps, bool plot_samples)
{
  // calculate radius for points in closing region of robot hand
  Eigen::Vector3d radius_options;
  radius_options << hand_outer_diameter_ - finger_width_, hand_depth_, hand_height_/2.0;
  double radius = radius_options.maxCoeff();

  // create KdTree for neighborhood search
  const Eigen::MatrixXi& camera_source = cloud_cam.getCameraSource();
  const Eigen::Matrix3Xd& cloud_normals = cloud_cam.getNormals();
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud);

  Plot plotter;
  Eigen::Matrix3Xd samples(3, grasps.size());
  for (int i = 0; i < grasps.size(); i++)
  {
    samples.col(i) = grasps[i].getSample();
  }
  if (plot_samples)
  {
    plotter.plotSamples(samples, cloud);
  }

  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  Eigen::Matrix3Xd points = cloud->getMatrixXfMap().block(0, 0, 3, cloud->size()).cast<double>();
  PointList point_list(points, cloud_normals, camera_source);
  PointList nn_points;
  std::vector<int> labels(grasps.size()); // -1: not feasible, 0: feasible, >0: see Antipodal class

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for private(nn_indices, nn_dists, nn_points) num_threads(num_threads_)
#endif
  for (int i = 0; i < grasps.size(); i++)
  {
    labels[i] = 0;
    pcl::PointXYZRGBA sample = eigenVectorToPcl(grasps[i].getSample());
//    std::cout << i << ", sample: " << grasps[i].getSample().transpose() << "\n";

    if (kdtree.radiusSearch(sample, radius, nn_indices, nn_dists) > 0)
    {
      nn_points = point_list.sliceMatrix(nn_indices);
      nn_points.setPoints(nn_points.getPoints() - grasps[i].getSample().replicate(1, nn_points.size()));
      PointList nn_points_frame;
      FingerHand finger_hand(finger_width_, hand_outer_diameter_, hand_depth_);

      // set the lateral and forward axes of the robot hand frame (closing direction and grasp approach direction)
      if (rotation_axis_ == ROTATION_AXIS_CURVATURE_AXIS)
      {
        finger_hand.setLateralAxis(1);
        finger_hand.setForwardAxis(0);
      }

      // check if the grasp is feasible (collision-free and contains at least one point)
      if (reevaluateHypothesis(nn_points, grasps[i], finger_hand, nn_points_frame))
      {
        // label the grasp
        labels[i] = labelHypothesis(nn_points_frame, finger_hand);
      }
    }
  }

  // remove empty list elements
  std::vector<GraspHypothesis> grasps_out;
  for (std::size_t i = 0; i < labels.size(); i++)
  {
    grasps_out.push_back(grasps[i]);
    bool is_half_grasp = labels[i] == Antipodal::FULL_GRASP || labels[i] == Antipodal::HALF_GRASP;
    grasps_out[grasps_out.size()-1].setHalfAntipodal(is_half_grasp);
    grasps_out[grasps_out.size()-1].setFullAntipodal(labels[i] == Antipodal::FULL_GRASP);
  }

  return grasps_out;
}


pcl::PointXYZRGBA HandSearch::eigenVectorToPcl(const Eigen::Vector3d& v)
{
  pcl::PointXYZRGBA p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}


void HandSearch::setParameters(const Parameters& params)
{
  finger_width_ = params.finger_width_;
  hand_outer_diameter_= params.hand_outer_diameter_;
  hand_depth_ = params.hand_depth_;
  hand_height_ = params.hand_height_;
  init_bite_ = params.init_bite_;

  num_threads_ = params.num_threads_;
  num_samples_ = params.num_samples_;
  nn_radius_taubin_ = params.nn_radius_taubin_;
  num_orientations_ = params.num_orientations_;
  rotation_axis_ = params.rotation_axis_;

  cam_tf_left_ = params.cam_tf_left_;
  cam_tf_right_ = params.cam_tf_right_;
}


std::vector<GraspHypothesis> HandSearch::evaluateHands(const CloudCamera& cloud_cam,
  const std::vector<LocalFrame>& frames, const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree)
{
  double t1 = omp_get_wtime();

  // calculate radius for points in closing region of robot hand
  Eigen::Vector3d radius_options;
  radius_options << hand_outer_diameter_ - finger_width_, hand_depth_, hand_height_/2.0;
  double radius = radius_options.maxCoeff();

  // possible angles used for hand orientations
  Eigen::VectorXd angles0 = Eigen::VectorXd::LinSpaced(num_orientations_ + 1, -1.0 * M_PI/2.0, M_PI/2.0);
  Eigen::VectorXd angles = angles0.block(0, 0, num_orientations_, 1);

  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();
  Eigen::Matrix3Xd points = cloud->getMatrixXfMap().block(0, 0, 3, cloud->size()).cast<double>();
  std::vector< std::vector<GraspHypothesis> > hand_lists(frames.size(), std::vector<GraspHypothesis>(0));
  PointList point_list(points, cloud_cam.getNormals(), cloud_cam.getCameraSource());
  PointList nn_points;

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for private(nn_indices, nn_dists, nn_points) num_threads(num_threads_)
#endif
  for (std::size_t i = 0; i < frames.size(); i++)
  {
    pcl::PointXYZRGBA sample = eigenVectorToPcl(frames[i].getSample());

    if (kdtree.radiusSearch(sample, radius, nn_indices, nn_dists) > 0)
    {
      nn_points = point_list.sliceMatrix(nn_indices);
      nn_points.setPoints(nn_points.getPoints() - frames[i].getSample().replicate(1, nn_points.size()));

      std::vector<GraspHypothesis> hands = evaluateHand(sample, nn_points, frames[i], angles);
      if (hands.size() > 0)
        hand_lists[i] = hands;
    }
  }

  // concatenate the grasp lists
  std::vector<GraspHypothesis> hypotheses;
  for (std::size_t i = 0; i < hand_lists.size(); i++)
  {
    if (hand_lists[i].size() > 0)
      hypotheses.insert(hypotheses.end(), hand_lists[i].begin(), hand_lists[i].end());
  }

  double t2 = omp_get_wtime();
  std::cout << " Found " << hypotheses.size() << " robot hand poses in " << t2 - t1 << " sec.\n";

  return hypotheses;
}


std::vector<GraspHypothesis> HandSearch::evaluateHand(const pcl::PointXYZRGBA& sample, const PointList& point_list,
  const LocalFrame& local_frame, const Eigen::VectorXd& angles)
{
  FingerHand finger_hand(finger_width_, hand_outer_diameter_, hand_depth_);

  // set the lateral and forward axes of the robot hand frame (closing direction and grasp approach direction)
  if (rotation_axis_ == ROTATION_AXIS_CURVATURE_AXIS)
  {
    finger_hand.setLateralAxis(1);
    finger_hand.setForwardAxis(0);
  }

  // rotation about binormal by 180 degrees (reverses direction of normal)
  Eigen::Matrix3d rot_binormal;
  rot_binormal <<  -1.0,  0.0,  0.0,
                    0.0,  1.0,  0.0,
                    0.0,  0.0, -1.0;

  // local reference frame
  Eigen::Matrix3d local_frame_mat;
  local_frame_mat << local_frame.getNormal(), local_frame.getBinormal(), local_frame.getCurvatureAxis();

//  std::cout << "local_frame_mat:\n" << local_frame_mat << "\n";

  // evaluate grasp at each hand orientation
  std::vector<GraspHypothesis> hand_list;
  for (int i = 0; i < angles.rows(); i++)
  {
    // rotation about curvature axis by <angles(i)> radians
    Eigen::Matrix3d rot;
    rot <<  cos(angles(i)),  -1.0 * sin(angles(i)),  0.0,
            sin(angles(i)),  cos(angles(i)),         0.0,
            0.0,             0.0,                    1.0;

    // rotate points into this hand orientation
    Eigen::Matrix3d frame_rot = local_frame_mat * rot_binormal * rot;
    Eigen::Matrix3Xd points_frame = frame_rot.transpose() * point_list.getPoints();
    Eigen::Matrix3Xd normals_frame = frame_rot.transpose() * point_list.getNormals();
    PointList point_list_frame(points_frame, normals_frame, point_list.getCamSource());

    // crop points based on hand height
    PointList point_list_cropped = cropByHandHeight(point_list_frame, hand_height_);

    // evaluate finger locations for this orientation
    finger_hand.evaluateFingers(point_list_cropped.getPoints(), init_bite_);

    // check that there are at least two corresponding finger placements
    if (finger_hand.getFingers().cast<int>().sum() >= 2)
    {
      finger_hand.evaluateHand();

      if (finger_hand.getHand().cast<int>().sum() > 0)
      {
        // try to move the hand as deep as possible onto the object
        finger_hand.deepenHand(point_list_cropped.getPoints(), init_bite_, hand_depth_);

        // calculate points in the closing region of the hand
        std::vector<int> indices_learning = finger_hand.computePointsInClosingRegion(point_list_cropped.getPoints());
        if (indices_learning.size() == 0)
        {
//          std::cout << "#point_list_cropped: " << point_list_cropped.size() << ", #indices_learning: "
//            << indices_learning.size() << "\n";
          continue;
        }

        GraspHypothesis hand = createGraspHypothesis(local_frame.getSample(), point_list_cropped, indices_learning,
          frame_rot, finger_hand);
        hand_list.push_back(hand);
      }
    }
  }

  return hand_list;
}


bool HandSearch::reevaluateHypothesis(const PointList& point_list, const GraspHypothesis& hand,
  FingerHand& finger_hand, PointList& point_list_cropped)
{
  // transform points into hand frame and crop them on <hand_height>
  const Eigen::Matrix3d& frame = hand.getHandFrame();
  Eigen::Matrix3Xd points_frame = frame.transpose() * point_list.getPoints();
  Eigen::Matrix3Xd normals_frame = frame.transpose() * point_list.getNormals();
  PointList point_list_frame(points_frame, normals_frame, point_list.getCamSource());
  point_list_cropped = cropByHandHeight(point_list_frame, hand_height_);

  // evaluate finger location for this grasp
  finger_hand.evaluateFingers(point_list_cropped.getPoints(), hand.getTop(), hand.getFingerPlacementIndex());

  // check that the finger placement is feasible
  if (finger_hand.getFingers().cast<int>().sum() >= 2)
  {
    finger_hand.evaluateHand(hand.getFingerPlacementIndex());

    if (finger_hand.getHand().cast<int>().sum() > 0)
    {
      return true;
    }
  }

  return false;
}


int HandSearch::labelHypothesis(const PointList& point_list, FingerHand& finger_hand)
{
  std::vector<int> indices_learning = finger_hand.computePointsInClosingRegion(point_list.getPoints());
  if (indices_learning.size() == 0)
  {
//    std::cout << "#pts_rot: " << point_list.getPoints().size() << ", #indices_learning: " << indices_learning.size()
//      << "\n";
    return Antipodal::NO_GRASP;
  }

  // extract data for classification
  PointList point_list_learning = point_list.sliceMatrix(indices_learning);

  // calculate grasp width
  const Eigen::VectorXd& lateral_pts = point_list_learning.getPoints().row(finger_hand.getLateralAxis());
  double grasp_width = lateral_pts.maxCoeff() - lateral_pts.minCoeff();

  // evaluate if the grasp is antipodal
  Antipodal antipodal;
  //      std::cout << " points_in_box: " << points_learning.cols() << " normals_in_box: " << normals_in_box.cols() << std::endl;
  int antipodal_result = antipodal.evaluateGrasp(point_list_learning, 0.003, finger_hand.getLateralAxis(),
    finger_hand.getForwardAxis(), rotation_axis_);
  //      std::cout << " antipodal_result: " << antipodal_result << std::endl;

  return antipodal_result;
}


PointList HandSearch::cropByHandHeight(const PointList& points_in, double height, int dim)
{
  std::vector<int> indices(points_in.size());
  int k = 0;
  for (int i = 0; i < points_in.size(); i++)
  {
    if (points_in.getPoints()(dim, i) > -1.0 * height && points_in.getPoints()(dim, i) < height)
    {
      indices[k] = i;
      k++;
    }
  }

  Eigen::Matrix3Xd points_out(3, k);
  Eigen::Matrix3Xd normals_out(3, k);
  Eigen::MatrixXi cam_source_out(points_in.getCamSource().rows(), k);
  for (int i = 0; i < k; i++)
  {
    points_out.col(i) = points_in.getPoints().col(indices[i]);
    normals_out.col(i) = points_in.getNormals().col(indices[i]);
    cam_source_out.col(i) = points_in.getCamSource().col(indices[i]);
  }

  return PointList(points_out, normals_out, cam_source_out);
}


GraspHypothesis HandSearch::createGraspHypothesis(const Eigen::Vector3d& sample, const PointList& point_list,
  const std::vector<int>& indices_learning, const Eigen::Matrix3d& hand_frame, const FingerHand& finger_hand)
{
  // extract data for classification
  PointList point_list_learning = point_list.sliceMatrix(indices_learning);

  // evaluate if the grasp is antipodal
  Antipodal antipodal;
  int antipodal_result = antipodal.evaluateGrasp(point_list_learning, 0.003, finger_hand.getLateralAxis(),
    finger_hand.getForwardAxis(), rotation_axis_);

  GraspHypothesis hand(sample, hand_frame, finger_hand, point_list_learning);
  hand.setHalfAntipodal(antipodal_result == Antipodal::HALF_GRASP || antipodal_result == Antipodal::FULL_GRASP);
  hand.setFullAntipodal(antipodal_result == Antipodal::FULL_GRASP);

  return hand;
}


void HandSearch::cropPointsAndNormals(const Eigen::Matrix3Xd& points, const Eigen::Matrix3Xd& normals,
  const Eigen::MatrixXi& cam_source, double height, Eigen::Matrix3Xd& points_out, Eigen::Matrix3Xd& normals_out,
  Eigen::MatrixXi& cam_source_out)
{
  std::vector<int> indices(points.cols());
  int k = 0;
  for (int i = 0; i < points.cols(); i++)
  {
    if (points(2, i) > -1.0 * height && points(2, i) < height)
    {
      indices[k] = i;
      k++;
    }
  }

  points_out.resize(3, k);
  normals_out.resize(3, k);
  cam_source_out.resize(cam_source.rows(), k);
  for (int i = 0; i < k; i++)
  {
    points_out.col(i) = points.col(indices[i]);
    normals_out.col(i) = normals.col(indices[i]);
    cam_source_out.col(i) = cam_source.col(indices[i]);
  }
}