#ifndef REMOVER_UTILS_H
#define REMOVER_UTILS_H

#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <cmath>
#include <string>
#include <map>
#include <iostream>
#include <vector>
#include <memory>

#include <ros/ros.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/copy_point.h>

#include <remover/node.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "signal.h"
#include <queue>

namespace remover_utils
{

    sensor_msgs::PointCloud2 cloud2msg(
        const pcl::PointCloud<pcl::PointXYZI> &cloud);

    int load_pcd(
        std::string pcd_name, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> dst);

    Eigen::Matrix4d poseToEigenTransform(
        const geometry_msgs::Pose &pose);

    geometry_msgs::Pose eigenMatrixToPose(
        const Eigen::Matrix4d &eigen_matrix);

    void separate_static_and_dynamic_pts(
        const pcl::PointCloud<pcl::PointXYZI> &cloudIn, pcl::PointCloud<pcl::PointXYZI> &dynamic_pts,
        pcl::PointCloud<pcl::PointXYZI> &static_pts);

    void voxelize_preserving_labels(
        pcl::PointCloud<pcl::PointXYZI>::Ptr src,
        pcl::PointCloud<pcl::PointXYZI> &dst, float leaf_size);

}
#endif
