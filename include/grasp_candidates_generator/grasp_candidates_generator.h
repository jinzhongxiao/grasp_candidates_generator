#ifndef GRASP_CANDIDATES_GENERATOR_H
#define GRASP_CANDIDATES_GENERATOR_H


// System
#include <vector>


// PCL
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Custom
#include <grasp_candidates_generator/cloud_camera.h>
#include <grasp_candidates_generator/hand_search.h>
#include <grasp_candidates_generator/plot.h>


class GraspCandidatesGenerator
{
  public:

    struct Parameters
    {
      bool plot_grasps_;
      bool remove_statistical_outliers_;
      bool voxelize_;
      int num_samples_;
      int num_threads_;
      std::vector<double> workspace_;
    };

    GraspCandidatesGenerator(const Parameters& params, const HandSearch::Parameters& hand_search_params);

    void preprocessPointCloud(CloudCamera& cloud_cam);

    std::vector<GraspHypothesis> generateGraspCandidates(const CloudCamera& cloud_cam);


  private:

    HandSearch hand_search_;
    Plot plotter_;

    Parameters params_;
};

#endif /* GRASP_CANDIDATES_GENERATOR_H */
