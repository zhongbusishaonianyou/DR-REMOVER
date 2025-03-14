#include "tools/remover_utils.hpp"

std::vector<int> DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259};
// std::vector<int> DYNAMIC_CLASSES = {4,5,6};
namespace remover_utils
{
    sensor_msgs::PointCloud2 cloud2msg(const pcl::PointCloud<pcl::PointXYZI> &cloud)
    {
        sensor_msgs::PointCloud2 cloud_ROS;
        pcl::toROSMsg(cloud, cloud_ROS);
        cloud_ROS.header.frame_id = "map";
        return cloud_ROS;
    }

    int load_pcd(std::string pcd_name, pcl::PointCloud<pcl::PointXYZI>::Ptr dst)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name, *dst) == -1)
        {
            return (-1);
        }
        
        return 0;
    }

    geometry_msgs::Pose eigenMatrixToPose(const Eigen::Matrix4d &eigen_matrix)
    {
        geometry_msgs::Pose pose;

        // 提取平移信息
        pose.position.x = eigen_matrix(0, 3);
        pose.position.y = eigen_matrix(1, 3);
        pose.position.z = eigen_matrix(2, 3);

        // 从旋转矩阵中提取四元数
        Eigen::Quaterniond quat(eigen_matrix.block<3, 3>(0, 0));
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();

        return pose;
    }
    Eigen::Matrix4d poseToEigenTransform(const geometry_msgs::Pose &pose)
    {
        Eigen::Vector3d position_eigen(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaterniond quaternion_eigen(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        Eigen::Matrix3d rotation_matrix = quaternion_eigen.toRotationMatrix();
        Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
        transform_matrix.block<3, 3>(0, 0) = rotation_matrix;
        transform_matrix.block<3, 1>(0, 3) = position_eigen;
        return transform_matrix;
    }

    void separate_static_and_dynamic_pts(
        const pcl::PointCloud<pcl::PointXYZI> &cloudIn, pcl::PointCloud<pcl::PointXYZI> &dynamic_pts,
        pcl::PointCloud<pcl::PointXYZI> &static_pts)
    {
        dynamic_pts.clear();
        static_pts.clear();

        for (const auto &pt : cloudIn.points)
        {
            uint32_t float2int = static_cast<uint32_t>(pt.intensity);
            uint32_t semantic_label = float2int & 0xFFFF;
            bool is_static = true;
            for (int class_num : DYNAMIC_CLASSES)
            {
                if (semantic_label == class_num)
                {
                    dynamic_pts.push_back(pt);
                    is_static = false;
                    break;
                }
            }
            if (is_static)
            {
                static_pts.push_back(pt);
            }
        }
    }

    void voxelize_preserving_labels(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI> &dst, float leaf_size)
    {
        /**< IMPORTANT
         * Because PCL voxlizaiton just does average the intensity of point cloud,
         * so this function is to conduct voxelization followed by nearest points search to re-assign the label of each point */
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_reassigned(new pcl::PointCloud<pcl::PointXYZI>);

        // 1. Voxelization
        static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(src);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*ptr_voxelized);

        // 2. Find nearest point to update intensity (index and id)
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(src);

        ptr_reassigned->points.reserve(ptr_voxelized->points.size());

        int K = 1;

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        // Set dst <- output
        for (const auto &pt : ptr_voxelized->points)
        {
            if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                auto updated = pt;
                // Update meaned intensity to original intensity
                updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
                ptr_reassigned->points.emplace_back(updated);
            }
        }
        dst = *ptr_reassigned;
    }

}