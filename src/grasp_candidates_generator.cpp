#include <grasp_candidates_generator/grasp_candidates_generator.h>


GraspCandidatesGenerator::GraspCandidatesGenerator(const Parameters& params,
  const HandSearch::Parameters& hand_search_params) : params_(params)
{
  hand_search_.setParameters(hand_search_params);
}


void GraspCandidatesGenerator::preprocessPointCloud(CloudCamera& cloud_cam)
{
  const double VOXEL_SIZE = 0.003;

  std::cout << "Processing cloud with: " << cloud_cam.getCloudOriginal()->size() << " points.\n";

  // perform statistical outlier removal
  if (params_.remove_statistical_outliers_)
  {
    //    Plot plotter;
    //    plotter.drawCloud(cloud_cam.getCloudProcessed(), "before");

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud_cam.getCloudProcessed());
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_cam.getCloudProcessed());
    std::cout << "Cloud after removing statistical outliers: " << cloud_cam.getCloudProcessed()->size() << std::endl;
    //    plotter.drawCloud(cloud_cam.getCloudProcessed(), "after");
  }

  // no indices into point cloud given
  if (cloud_cam.getSampleIndices().size() == 0)
  {
    // 1. Workspace filtering
    cloud_cam.filterWorkspace(params_.workspace_);
    std::cout << "After workspace filtering: " << cloud_cam.getCloudProcessed()->size() << " points left.\n";

    // 2. Voxelization
    if (params_.voxelize_)
    {
      cloud_cam.voxelizeCloud(VOXEL_SIZE);
      std::cout << "After voxelization: " << cloud_cam.getCloudProcessed()->size() << " points left.\n";
    }

    // 3. Subsampling
    if (cloud_cam.getSamples().cols() > 0)
    {
      // remove samples outside of the workspace
      cloud_cam.filterSamples(params_.workspace_);

      // subsample the remaining samples
      cloud_cam.subsampleSamples(params_.num_samples_);
    }

    // 3. Subsampling
    if (params_.num_samples_ > cloud_cam.getCloudProcessed()->size())
    {
      std::vector<int> indices_all(cloud_cam.getCloudProcessed()->size());
      for (int i=0; i < cloud_cam.getCloudProcessed()->size(); i++)
        indices_all[i] = i;
      cloud_cam.setSampleIndices(indices_all);
      std::cout << "Cloud is smaller than num_samples. Subsampled all " << cloud_cam.getCloudProcessed()->size()
        << " points.\n";
    }
    else
    {
      cloud_cam.subsampleUniformly(params_.num_samples_);
      std::cout << "Subsampled " << params_.num_samples_ << " at random uniformly.\n";
    }
  }
  // indices into point cloud given
  else
  {
    if (params_.num_samples_ > 0 && params_.num_samples_ < cloud_cam.getSampleIndices().size())
    {
      std::vector<int> indices_rand(params_.num_samples_);
      for (int i=0; i < params_.num_samples_; i++)
        indices_rand[i] = cloud_cam.getSampleIndices()[rand() % cloud_cam.getSampleIndices().size()];
      cloud_cam.setSampleIndices(indices_rand);
      std::cout << "Subsampled " << indices_rand.size() << " indices.\n";
    }
    else
    {
      std::cout << "Using all " << cloud_cam.getSampleIndices().size() << " indices.\n";
    }
  }

  // 4. Calculate surface normals.
  if (cloud_cam.getNormals().cols() == 0)
  {
    std::cout << "Calculating surface normals with " << params_.num_threads_ << " CPU threads.\n";
    cloud_cam.calculateNormals(params_.num_threads_);
    std::cout << "  done\n";
  }
}


std::vector<GraspHypothesis> GraspCandidatesGenerator::generateGraspCandidates(const CloudCamera& cloud_cam,
  bool use_samples)
{
  std::vector<GraspHypothesis> candidates = hand_search_.generateHypotheses(cloud_cam, 0, use_samples);
  std::cout << "Generated " << candidates.size() << " grasp candidates.\n";

  if (params_.plot_grasps_)
  {
    plotter_.plotFingers(candidates, cloud_cam.getCloudOriginal(), "Grasp Candidates");
  }

  return candidates;
}
