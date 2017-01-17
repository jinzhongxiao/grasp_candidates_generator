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


#ifndef PLOT_H
#define PLOT_H


#include <pcl/visualization/pcl_visualizer.h>

#include <grasp_candidates_generator/grasp.h>
#include <grasp_candidates_generator/grasp_set.h>
#include <grasp_candidates_generator/local_frame.h>


typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudNormal;


/** Plot class
 *
 * This class provides a set of visualization methods that visualize the output of the algorithm and
 * some intermediate steps. The visualization is done using PCL Visualizer.
 *
 */
class Plot
{
  public:

    /**
     * \brief Plot a list of grasp sets.
     * \param hand_set_list the list of grasp sets
     * \param cloud the point cloud to be plotted
     * \param str the title of the plot window
     * \param outer_diameter the width of the drawn grasps
     */
    void plotFingers(const std::vector<GraspSet>& hand_set_list, const PointCloudRGBA::Ptr& cloud,
      std::string str, double outer_diameter = 0.09) const;

    /**
     * \brief Plot a list of grasps.
     * \param hand_list the list of grasps
     * \param cloud the point cloud to be plotted
     * \param str the title of the plot window
     * \param outer_diameter the width of the drawn grasps
     */
    void plotFingers(const std::vector<Grasp>& hand_list, const PointCloudRGBA::Ptr& cloud, std::string str,
      double outer_diameter = 0.09) const;

    /**
     * \brief Plot a list of samples.
     * \param index_list the list of samples (indices into the point cloud)
     * \param cloud the point cloud to be plotted
     */
    void plotSamples(const std::vector<int>& index_list, const PointCloudRGBA::Ptr& cloud) const;

    /**
     * \brief Plot a list of samples.
     * \param samples the list of samples (indices into the point cloud)
     * \param cloud the point cloud to be plotted
     */
    void plotSamples(const Eigen::Matrix3Xd& samples, const PointCloudRGBA::Ptr& cloud) const;

    /**
     * \brief Plot a point cloud that contains samples.
     * \param samples_cloud the point cloud that contains the samples
     * \param cloud the point cloud to be plotted
     */
    void plotSamples(const PointCloudRGBA::Ptr& samples_cloud, const PointCloudRGBA::Ptr& cloud) const;

    /** 
     * \brief Plot a list of normals.
     * \param cloud the point cloud to be plotted
     * \param normals the normals to be plotted
     */
    void plotNormals(const PointCloudRGBA::Ptr& cloud, const Eigen::Matrix3Xd& normals) const;

    /**
     * \brief Plot a list of points and their normals.
     * \param pts the list of points to be plotted
     * \param normals the normals to be plotted
     */
    void plotNormals(const Eigen::Matrix3Xd& pts, const Eigen::Matrix3Xd& normals) const;

    /**
     * \brief Plot a list of local reference frames.
     * \param frame_list the list of frames to be plotted
     * \param cloud the point cloud to be plotted
     */
    void plotLocalAxes(const std::vector<LocalFrame>& frame_list, const PointCloudRGBA::Ptr& cloud) const;

    /**
     * \brief Plot the camera source for each point in the point cloud.
     * \param pts_cam_source_in the camera source for each point in the point cloud
     * \param cloud the point cloud to be plotted
     */
    void plotCameraSource(const Eigen::VectorXi& pts_cam_source_in, const PointCloudRGBA::Ptr& cloud) const;

    /**
     * \brief Plot a point cloud.
     * \param cloud_rgb the point cloud to be plotted
     * \param str the title of the plot window
     */
    void plotCloud(const PointCloudRGBA::Ptr& cloud_rgb, const std::string& title) const;


  private:

    /**
     * \brief Create a point cloud that stores the visual representations of the grasps.
     * \param hand_list the list of grasps to be be stored in the point cloud
     * \param outer_diameter the outer diameter of the visual grasp representation
     */
    PointCloudRGBA::Ptr createFingersCloud(const std::vector<Grasp>& hand_list, double outer_diameter) const;

    /**
     * \brief Convert an Eigen vector to a PCL point.
     * \param v the Eigen vector to be converted
     */
    pcl::PointXYZRGBA eigenVector3dToPointXYZRGBA(const Eigen::Vector3d& v) const;

    /**
     * \brief Set the color of a point.
     * \param hand the grasp that the point belongs to
     * \param p the point for which the color is set
     */
    void setPointColor(const Grasp& hand, pcl::PointXYZRGBA& p) const;

    /**
     * \brief Add a point cloud with normals to a PCL visualizer.
     * \param viewer the PCL visualizer that the cloud is added to
     * \param cloud the cloud to be added
     * \param line_width the line width for drawing normals
     * \param color_cloud the color that is used to draw the cloud
     * \param color_normals the color that is used to draw the normals
     * \param cloud_name an identifier string for the cloud
     * \param normals_name an identifier string for the normals
     */
    void addCloudNormalsToViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,
      const PointCloudNormal::Ptr& cloud, double line_width, double* color_cloud, double* color_normals,
      const std::string& cloud_name, const std::string& normals_name) const;

    /**
     * \brief Run/show a PCL visualizer until an escape key is hit.
     * \param viewer the PCL visualizer to be shown
     */
    void runViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer) const;

    /**
     * \brief Create a PCL visualizer.
     * \param title the title of the visualization window
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer(std::string title) const;

    double marker_lifetime_; ///< max time that markers are visualized in Rviz
};

#endif /* PLOT_H */ 
