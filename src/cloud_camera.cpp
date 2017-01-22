#include <grasp_candidates_generator/cloud_camera.h>


CloudCamera::CloudCamera()
  : cloud_original_(new PointCloudRGB), cloud_processed_(new PointCloudRGB)
{
  view_points_.resize(3,1);
  view_points_ << 0.0, 0.0, 0.0;
}


CloudCamera::CloudCamera(const PointCloudNormal::Ptr& cloud, int size_left_cloud, const Eigen::Matrix3Xd& view_points)
  : cloud_processed_(new PointCloudRGB), cloud_original_(new PointCloudRGB), view_points_(view_points)
{
  sample_indices_.resize(0);
  samples_.resize(3,0);

  pcl::copyPointCloud(*cloud, *cloud_original_);
  *cloud_processed_ = *cloud_original_;

  // set the camera source matrix: (i,j) = 1 if point j is seen by camera i
  if (size_left_cloud == cloud->size()) // one camera
  {
    camera_source_ = Eigen::MatrixXi::Zero(1, cloud->size());
  }
  else // two cameras
  {
    int size_right_cloud = cloud->size() - size_left_cloud;
    camera_source_ = Eigen::MatrixXi::Zero(2, cloud->size());
    camera_source_.block(0,0,1,size_left_cloud) = Eigen::MatrixXi::Ones(1, size_left_cloud);
    camera_source_.block(1,size_left_cloud,1,size_right_cloud) = Eigen::MatrixXi::Ones(1, size_right_cloud);
  }

  normals_.resize(3, cloud->size());
  for (int i = 0; i < cloud->size(); i++)
  {
    normals_.col(i) << cloud->points[i].normal_x, cloud->points[i].normal_y, cloud->points[i].normal_z;
  }
}


CloudCamera::CloudCamera(const PointCloudRGB::Ptr& cloud, int size_left_cloud, const Eigen::Matrix3Xd& view_points)
  : cloud_processed_(cloud), cloud_original_(cloud), view_points_(view_points)
{
  sample_indices_.resize(0);

  // set the camera source matrix: (i,j) = 1 if point j is seen by camera i
  if (size_left_cloud == cloud->size()) // one camera
  {
    camera_source_ = Eigen::MatrixXi::Zero(1, cloud->size());
  }
  else // two cameras
  {
    int size_right_cloud = cloud->size() - size_left_cloud;
    camera_source_ = Eigen::MatrixXi::Zero(2, cloud->size());
    camera_source_.block(0,0,1,size_left_cloud) = Eigen::MatrixXi::Ones(1, size_left_cloud);
    camera_source_.block(1,size_left_cloud,1,size_right_cloud) = Eigen::MatrixXi::Ones(1, size_right_cloud);
  }
}


CloudCamera::CloudCamera(const std::string& filename, const Eigen::Matrix3Xd& view_points)
  : cloud_processed_(new PointCloudRGB), view_points_(view_points)
{
  sample_indices_.resize(0);
  cloud_processed_ = loadPointCloudFromFile(filename);
  cloud_original_ = cloud_processed_;
  camera_source_ = Eigen::MatrixXi::Ones(1, cloud_processed_->size());
  std::cout << "Loaded point cloud with " << camera_source_.cols() << " points \n";
}


CloudCamera::CloudCamera(const std::string& filename_left, const std::string& filename_right,
    const Eigen::Matrix3Xd& view_points) : cloud_processed_(new PointCloudRGB), view_points_(view_points)
{
  sample_indices_.resize(0);

  // load and combine the two point clouds
  std::cout << "Loading point clouds ...\n";
  PointCloudRGB::Ptr cloud_left(new PointCloudRGB), cloud_right(new PointCloudRGB);
  cloud_left = loadPointCloudFromFile(filename_left);
  cloud_right = loadPointCloudFromFile(filename_right);

  std::cout << "Concatenating point clouds ...\n";
  *cloud_processed_ = *cloud_left + *cloud_right;
  cloud_original_ = cloud_processed_;

  std::cout << "Loaded left point cloud with " << cloud_left->size() << " points \n";
  std::cout << "Loaded right point cloud with " << cloud_right->size() << " points \n";

  // set the camera source matrix: (i,j) = 1 if point j is seen by camera i
  camera_source_ = Eigen::MatrixXi::Zero(2, cloud_processed_->size());
  camera_source_.block(0,0,1,cloud_left->size()) = Eigen::MatrixXi::Ones(1, cloud_left->size());
  camera_source_.block(1,cloud_left->size(),1,cloud_right->size()) = Eigen::MatrixXi::Ones(1, cloud_right->size());
}


void CloudCamera::filterWorkspace(const std::vector<double>& workspace)
{
  std::vector<int> indices;
  for (int i = 0; i < cloud_processed_->size(); i++)
  {
    const pcl::PointXYZRGBA& p = cloud_processed_->points[i];
    if (p.x > workspace[0] && p.x < workspace[1] && p.y > workspace[2] && p.y < workspace[3]
        && p.z > workspace[4] && p.z < workspace[5])
    {
      indices.push_back(i);
    }
  }

  Eigen::MatrixXi camera_source(camera_source_.rows(), indices.size());
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  cloud->points.resize(indices.size());
  for (int i = 0; i < indices.size(); i++)
  {
    camera_source(i) = camera_source_(indices[i]);
    cloud->points[i] = cloud_processed_->points[indices[i]];
  }
  if (normals_.cols() > 0)
  {
    Eigen::Matrix3Xd normals(3, indices.size());
    for (int i = 0; i < indices.size(); i++)
    {
      normals.col(i) = normals_.col(indices[i]);
    }
    normals_ = normals;
  }
  cloud_processed_ = cloud;
  camera_source_ = camera_source;
}


void CloudCamera::filterSamples(const std::vector<double>& workspace)
{
  std::vector<int> indices;
  for (int i = 0; i < samples_.size(); i++)
  {
    if (samples_(0,i) > workspace[0] && samples_(0,i) < workspace[1]
        && samples_(1,i) > workspace[2] && samples_(1,i) < workspace[3]
        && samples_(2,i) > workspace[4] && samples_(2,i) < workspace[5])
    {
      indices.push_back(i);
    }
  }

  Eigen::Matrix3Xd filtered_samples(3, indices.size());
  for (int i = 0; i < indices.size(); i++)
  {
    filtered_samples.col(i) = samples_.col(i);
  }
  samples_ = filtered_samples;
}


void CloudCamera::voxelizeCloud(double cell_size)
{
  Eigen::MatrixXf pts = cloud_processed_->getMatrixXfMap();
  Eigen::Vector3f min_xyz;
  min_xyz << pts.row(0).minCoeff(), pts.row(1).minCoeff(), pts.row(2).minCoeff();

  // find the cell that each point falls into
  std::set< Eigen::Vector4i, CloudCamera::UniqueVectorFirstThreeElementsComparator> bins;
  std::vector<Eigen::Vector3d> avg_normals;
  avg_normals.resize(pts.cols());
  std::vector<int> counts;
  counts.resize(pts.cols());

  for (int i = 0; i < pts.cols(); i++)
  {
    Eigen::Vector3f p;
    p << pts.col(i)(0), pts.col(i)(1), pts.col(i)(2);
    Eigen::Vector3i v = EigenUtils::floorVector((p - min_xyz) / cell_size);
    Eigen::Vector4i v4;
    v4 << v(0), v(1), v(2), i;
    std::pair< std::set<Eigen::Vector4i, CloudCamera::UniqueVectorFirstThreeElementsComparator>::iterator, bool> res = bins.insert(v4);

    if (res.second && normals_.cols() > 0)
    {
      avg_normals[i] = normals_.col(i);
      counts[i] = 1;
    }
    else if (normals_.cols() > 0)
    {
      const int& idx = (*res.first)(3);
      avg_normals[idx] += normals_.col(i);
      counts[idx]++;
    }
  }

  // Calculate the point value and the average surface normal for each cell, and set the camera source for each point.
  Eigen::Matrix3Xf voxels(3, bins.size());
  Eigen::Matrix3Xd normals(3, bins.size());
  Eigen::MatrixXi camera_source(camera_source_.rows(), bins.size());
  int i = 0;
  std::set<Eigen::Vector4i, CloudCamera::UniqueVectorFirstThreeElementsComparator>::iterator it;

  for (it = bins.begin(); it != bins.end(); it++)
  {
    voxels.col(i) = (*it).block(0,0,3,1).cast<float>();
    const int& idx = (*it)(3);

    for (int j = 0; j < camera_source_.rows(); j++)
    {
      camera_source(j,i) = (camera_source_(j, idx) == 1) ? 1 : 0;
    }
    if (normals_.cols() > 0)
    {
      normals.col(i) = avg_normals[idx] / (double) counts[idx];
    }
    i++;
  }

  voxels.row(0) = voxels.row(0) * cell_size + Eigen::VectorXf::Ones(voxels.cols()) * min_xyz(0);
  voxels.row(1) = voxels.row(1) * cell_size + Eigen::VectorXf::Ones(voxels.cols()) * min_xyz(1);
  voxels.row(2) = voxels.row(2) * cell_size + Eigen::VectorXf::Ones(voxels.cols()) * min_xyz(2);

  // Copy the voxels into the point cloud.
  cloud_processed_->points.resize(voxels.cols());
  for(int i=0; i < voxels.cols(); i++)
  {
    cloud_processed_->points[i].getVector3fMap() = voxels.col(i).cast<float>();
  }

  camera_source_ = camera_source;

  if (normals_.cols() > 0)
    normals_ = normals;
}


void CloudCamera::subsampleUniformly(int num_samples)
{
  sample_indices_.resize(num_samples);
  pcl::RandomSample<pcl::PointXYZRGBA> random_sample;
  random_sample.setInputCloud(cloud_processed_);
  random_sample.setSample(num_samples);
  random_sample.filter(sample_indices_);
}


void CloudCamera::subsampleSamples(int num_samples)
{
  // use all incoming samples
  if (num_samples == 0 || num_samples >= samples_.cols())
  {
    std::cout << "Using all " << samples_.cols() << " samples.\n";
  }
  // subsample the incoming samples
  else
  {
    std::vector<int> seq(samples_.size());
    for (int i = 0; i < seq.size(); i++)
    {
      seq[i] = i;
    }
    std::random_shuffle(seq.begin(), seq.end());

    Eigen::Matrix3Xd subsamples(3, num_samples);
    for (int i = 0; i < samples_.cols(); i++)
    {
      subsamples.col(i) = samples_.col(seq[i]);
    }
    samples_ = subsamples;

    std::cout << "Subsampled " << samples_.cols() << " samples at random uniformly.\n";
  }
}


void CloudCamera::writeNormalsToFile(const std::string& filename, const Eigen::Matrix3Xd& normals)
{
  std::ofstream myfile;
  myfile.open (filename.c_str());

  for (int i = 0; i < normals.cols(); i++)
  {
    myfile << boost::lexical_cast<std::string>(normals(0,i)) << "," << boost::lexical_cast<std::string>(normals(1,i)) << "," << boost::lexical_cast<std::string>(normals(2,i)) << "\n";
  }

  myfile.close();
}


void CloudCamera::calculateNormals(int num_threads)
{
  double t0 = omp_get_wtime();
  std::cout << "Calculating surface normals";

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  if (cloud_processed_->isOrganized())
  {
    std::cout << " using integral images ...\n";
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud(cloud_processed_);
    ne.setViewPoint(view_points_(0,0), view_points_(1,0), view_points_(2,0));
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setNormalSmoothingSize(20.0f);
    ne.compute(*cloud_normals);
  }
  else
  {
    std::cout << " using normal estimation OMP ...\n";
    pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> estimator(num_threads);
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_ptr(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    estimator.setInputCloud(cloud_processed_);
    estimator.setViewPoint(view_points_(0,0), view_points_(1,0), view_points_(2,0));
    estimator.setSearchMethod(tree_ptr);
    estimator.setRadiusSearch(0.03);
//    estimator.setKSearch(30);
    estimator.compute(*cloud_normals);
  }

  normals_ = cloud_normals->getMatrixXfMap().cast<double>();

  std::cout << " runtime: " << omp_get_wtime() - t0 << "\n";
}


void CloudCamera::setNormalsFromFile(const std::string& filename)
{
  std::ifstream in;
  in.open(filename.c_str());
  std::string line;
  normals_.resize(3, cloud_original_->size());
  int i = 0;

  while(std::getline(in, line))
  {
    std::stringstream lineStream(line);
    std::string cell;
    int j = 0;

    while(std::getline(lineStream, cell, ','))
    {
      normals_(i,j) = boost::lexical_cast<double>(cell);
      j++;
    }

    i++;
  }
}


PointCloudRGB::Ptr CloudCamera::loadPointCloudFromFile(const std::string& filename) const
{
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename, *cloud) == -1)
  {
    std::cout << "Couldn't read .pcd file: " << filename << "\n";
    cloud->points.resize(0);
  }
  return cloud;
}


void CloudCamera::setSamples(const Eigen::Matrix3Xd& samples)
{
  samples_ = samples;
}
