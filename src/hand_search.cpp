#include <grasp_candidates_generator/hand_search.h>


const int HandSearch::ROTATION_AXIS_NORMAL = 0;
const int HandSearch::ROTATION_AXIS_BINORMAL = 1;
const int HandSearch::ROTATION_AXIS_CURVATURE_AXIS = 2;


std::vector<GraspHypothesis> HandSearch::generateHypotheses(const CloudCamera& cloud_cam, int antipodal_mode,
  bool use_samples, bool forces_PSD, bool plots_normals, bool plots_samples) const
{
  if (params_.rotation_axis_ < 0 || params_.rotation_axis_ > 2)
  {
    std::cout << "Parameter <rotation_axis> is not set correctly!\n";
    std::vector<GraspHypothesis> empty(0);
    return empty;
  }

  double t0_total = omp_get_wtime();

  // Create KdTree for neighborhood search.
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud);

  // Plot normals and/or samples if desired.
  if (plots_normals)
  {
    std::cout << "Plotting normals ...\n";
    plot_.plotNormals(cloud_cam.getNormals(), cloud_normals_);
  }

  if (plots_samples)
  {
    if (cloud_cam.getSampleIndices().size() > 0)
    {
      std::cout << "Plotting sample indices ...\n";
      plot_.plotSamples(cloud_cam.getSampleIndices(), cloud_cam.getCloudProcessed());
    }
    else if (cloud_cam.getSamples().cols() > 0)
    {
      std::cout << "Plotting samples ...\n";
      plot_.plotSamples(cloud_cam.getSamples(), cloud_cam.getCloudProcessed());
    }
  }

  // 1. Estimate local reference frames.
  std::cout << "Estimating local reference frames ...\n";
  std::vector<LocalFrame> frames;
  FrameEstimator frame_estimator(params_.num_threads_);
  if (use_samples)
    frames = frame_estimator.calculateLocalFrames(cloud_cam, cloud_cam.getSamples(), params_.nn_radius_frames_, kdtree);
  else
    frames = frame_estimator.calculateLocalFrames(cloud_cam, cloud_cam.getSampleIndices(), params_.nn_radius_frames_, kdtree);
  if (plots_local_axes_)
    plot_.plotLocalAxes(frames, cloud_cam.getCloudOriginal());

  // 2. Evaluate possible hand placements.
  std::cout << "Finding hand poses ...\n";
  std::vector<GraspHypothesis> hypotheses = evaluateHands(cloud_cam, frames, kdtree);

  std::cout << "====> HAND SEARCH TIME: " << omp_get_wtime() - t0_total << std::endl;

  return hypotheses;
}


std::vector<GraspHypothesis> HandSearch::reevaluateHypotheses(const CloudCamera& cloud_cam,
  const std::vector<GraspHypothesis>& grasps, bool plot_samples) const
{
  // calculate radius for points in closing region of robot hand
  Eigen::Vector3d radius_options;
  radius_options << params_.hand_outer_diameter_ - params_.finger_width_, params_.hand_depth_, params_.hand_height_/2.0;
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
  PointList point_list(points, cloud_normals, camera_source, cloud_cam.getViewPoints());
  PointList nn_points;
  std::vector<int> labels(grasps.size()); // -1: not feasible, 0: feasible, >0: see Antipodal class

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for private(nn_indices, nn_dists, nn_points) num_threads(params_.num_threads_)
#endif
  for (int i = 0; i < grasps.size(); i++)
  {
    labels[i] = 0;
    pcl::PointXYZRGBA sample = eigenVectorToPcl(grasps[i].getSample());

    if (kdtree.radiusSearch(sample, radius, nn_indices, nn_dists) > 0)
    {
      nn_points = point_list.sliceMatrix(nn_indices);
      nn_points.setPoints(nn_points.getPoints() - grasps[i].getSample().replicate(1, nn_points.size()));
      PointList nn_points_frame;
      FingerHand finger_hand(params_.finger_width_, params_.hand_outer_diameter_, params_.hand_depth_);

      // set the lateral and forward axes of the robot hand frame (closing direction and grasp approach direction)
      if (params_.rotation_axis_ == ROTATION_AXIS_CURVATURE_AXIS)
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


pcl::PointXYZRGBA HandSearch::eigenVectorToPcl(const Eigen::Vector3d& v) const
{
  pcl::PointXYZRGBA p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}


std::vector<GraspHypothesis> HandSearch::evaluateHands(const CloudCamera& cloud_cam,
  const std::vector<LocalFrame>& frames, const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree) const
{
  double t1 = omp_get_wtime();

  // calculate radius for points in closing region of robot hand
  Eigen::Vector3d radius_options;
  radius_options << params_.hand_outer_diameter_ - params_.finger_width_, params_.hand_depth_, params_.hand_height_/2.0;
  double radius = radius_options.maxCoeff();

  // possible angles used for hand orientations
  Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(params_.num_orientations_ + 1, -1.0 * M_PI/2.0, M_PI/2.0);
  angles = angles.block(0, 0, params_.num_orientations_, 1);

  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();
  Eigen::Matrix3Xd points = cloud->getMatrixXfMap().block(0, 0, 3, cloud->size()).cast<double>();
  std::vector< std::vector<GraspHypothesis> > hand_lists(frames.size(), std::vector<GraspHypothesis>(0));
  PointList point_list(points, cloud_cam.getNormals(), cloud_cam.getCameraSource(), cloud_cam.getViewPoints());
  PointList nn_points;

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for private(nn_indices, nn_dists, nn_points) num_threads(params_.num_threads_)
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
  const LocalFrame& local_frame, const Eigen::VectorXd& angles) const
{
  FingerHand finger_hand(params_.finger_width_, params_.hand_outer_diameter_, params_.hand_depth_);

  // set the lateral and forward axes of the robot hand frame (closing direction and grasp approach direction)
  if (params_.rotation_axis_ == ROTATION_AXIS_CURVATURE_AXIS)
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
    PointList point_list_frame = point_list.rotatePointList(frame_rot.transpose());

    // crop points based on hand height
    PointList point_list_cropped = cropByHandHeight(point_list_frame, params_.hand_height_);

    // evaluate finger locations for this orientation
    finger_hand.evaluateFingers(point_list_cropped.getPoints(), params_.init_bite_);

    // check that there are at least two corresponding finger placements
    if (finger_hand.getFingers().cast<int>().sum() >= 2)
    {
      finger_hand.evaluateHand();

      if (finger_hand.getHand().cast<int>().sum() > 0)
      {
        // try to move the hand as deep as possible onto the object
        finger_hand.deepenHand(point_list_cropped.getPoints(), params_.init_bite_, params_.hand_depth_);

        // calculate points in the closing region of the hand
        std::vector<int> indices_learning = finger_hand.computePointsInClosingRegion(point_list_cropped.getPoints());
        if (indices_learning.size() == 0)
        {
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
  FingerHand& finger_hand, PointList& point_list_cropped) const
{
  // Transform points into hand frame and crop them on <hand_height>.
  PointList point_list_frame = point_list.rotatePointList(hand.getFrame().transpose());
  point_list_cropped = cropByHandHeight(point_list_frame, params_.hand_height_);

  // Evaluate finger location for this grasp.
  finger_hand.evaluateFingers(point_list_cropped.getPoints(), hand.getTop(), hand.getFingerPlacementIndex());

  // Check that the finger placement is possible.
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


int HandSearch::labelHypothesis(const PointList& point_list, FingerHand& finger_hand) const
{
  std::vector<int> indices_learning = finger_hand.computePointsInClosingRegion(point_list.getPoints());
  if (indices_learning.size() == 0)
  {
    return Antipodal::NO_GRASP;
  }

  // extract data for classification
  PointList point_list_learning = point_list.sliceMatrix(indices_learning);

  // evaluate if the grasp is antipodal
  Antipodal antipodal;
  int antipodal_result = antipodal.evaluateGrasp(point_list_learning, 0.003, finger_hand.getLateralAxis(),
    finger_hand.getForwardAxis(), params_.rotation_axis_);

  return antipodal_result;
}


PointList HandSearch::cropByHandHeight(const PointList& points_in, double height, int dim) const
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

  return PointList(points_out, normals_out, cam_source_out, points_in.getViewPoints());
}


GraspHypothesis HandSearch::createGraspHypothesis(const Eigen::Vector3d& sample, const PointList& point_list,
  const std::vector<int>& indices_learning, const Eigen::Matrix3d& hand_frame, const FingerHand& finger_hand) const
{
  // extract data for classification
  PointList point_list_learning = point_list.sliceMatrix(indices_learning);

  // evaluate if the grasp is antipodal
  Antipodal antipodal;
  int antipodal_result = antipodal.evaluateGrasp(point_list_learning, 0.003, finger_hand.getLateralAxis(),
    finger_hand.getForwardAxis(), params_.rotation_axis_);

  // calculate grasp width (hand opening width)
  double width = point_list_learning.getPoints().row(0).maxCoeff() - point_list_learning.getPoints().row(0).minCoeff();

  GraspHypothesis hand(sample, hand_frame, finger_hand, width);
  hand.setHalfAntipodal(antipodal_result == Antipodal::HALF_GRASP || antipodal_result == Antipodal::FULL_GRASP);
  hand.setFullAntipodal(antipodal_result == Antipodal::FULL_GRASP);

  return hand;
}
