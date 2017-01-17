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
#include <grasp_candidates_generator/grasp_hypothesis.h>
#include <grasp_candidates_generator/hypothesis_set.h>
#include <grasp_candidates_generator/hand_search.h>
#include <grasp_candidates_generator/plot.h>


class GraspCandidatesGenerator
{
  public:

    struct Parameters
    {
      bool plot_normals_;
      bool plot_grasps_;
      bool remove_statistical_outliers_;
      bool voxelize_;
      int num_samples_;
      int num_threads_;
      std::vector<double> workspace_;
    };

    GraspCandidatesGenerator(const Parameters& params, const HandSearch::Parameters& hand_search_params);

    ~GraspCandidatesGenerator()
    {
      delete hand_search_;
    }

    void preprocessPointCloud(CloudCamera& cloud_cam);

    std::vector<GraspHypothesis> generateGraspCandidates(const CloudCamera& cloud_cam, bool use_samples = false);

    std::vector<HypothesisSet> generateGraspCandidateSets(const CloudCamera& cloud_cam, bool use_samples = false);

    void setNumSamples(int num_samples)
    {
      params_.num_samples_ = num_samples;
    }


  private:

    HandSearch* hand_search_;
    Plot plotter_;

    Parameters params_;
};

#endif /* GRASP_CANDIDATES_GENERATOR_H */
