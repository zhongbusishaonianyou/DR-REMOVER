#ifndef OFFLINEMAPUPDATER_H
#define OFFLINEMAPUPDATER_H
#include "remover.h"

#define NUM_PTS_LARGE_ENOUGH 200000
#define NUM_PTS_LARGE_ENOUGH_FOR_MAP 5000000
namespace remover
{
    class OfflineMapUpdater
    {
    private:
        double query_voxel_size_;
        double map_voxel_size_;

        int removal_interval_;
        int num_pts_map;
        bool is_initialized_ = false;

        double L_max_;
        double min_h_;
        double max_h_;

        double submap_size_;
        float submap_center_x_;
        float submap_center_y_;

        std::string dataset_name_;
        std::string sequence_name_;
        std::string map_name_;
        std::string save_path_;
        
        unique_ptr<DR_REMOVER> remover_;

        ros::NodeHandle nh;

        ros::Subscriber sub_node_;
        ros::Subscriber sub_flag_;

        ros::Publisher pub_path_;
        ros::Publisher pub_preserved_static_pts; 
        ros::Publisher pub_preserved_dynamic_pts;  
        ros::Publisher pub_map_ground; 
        ros::Publisher pub_curr_scan;
        ros::Publisher pub_removed_static_pts;
        
        //global_map=submap_arranged+submap_complement+total_map_rejected_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr global_map;
        pcl::PointCloud<pcl::PointXYZI>::Ptr submap_arranged;
        pcl::PointCloud<pcl::PointXYZI>::Ptr submap_complement;
        pcl::PointCloud<pcl::PointXYZI>::Ptr total_map_rejected_;
       
        pcl::PointCloud<pcl::PointXYZI>::Ptr query_voi_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_ground_to_viz;

        //submap_arranged=map_voi_+map_voi_outskirts_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_voi_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_voi_outskirts_;
        
        //map_voi_=map_filtered_body+curr_map_rejected:
        pcl::PointCloud<pcl::PointXYZI>::Ptr curr_map_rejected;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered_body;
        
        nav_msgs::Path path_;

        vector<float> tf_lidar2body;
        
        Eigen::Matrix4f tf_lidar2body_;
        Eigen::Matrix4f tf_body2origin_;
        
        void set_initial_parameters();

        void initialize_global_ptrs();

        void load_global_map();

        void clear(pcl::PointCloud<pcl::PointXYZI> &pt_cloud);

        void callback_node(const remover::node::ConstPtr &msg);

        void callback_flag(const std_msgs::Float32::ConstPtr &msg);

        void reassign_submap(float pose_x, float pose_y);

        void obtain_submap_from_map(
            const pcl::PointCloud<pcl::PointXYZI> &map_global,
            pcl::PointCloud<pcl::PointXYZI> &submap,
            pcl::PointCloud<pcl::PointXYZI> &submap_complement,
            float x, float y);

        void extract_map_voi(
            float curr_x, float curr_y,
            pcl::PointCloud<pcl::PointXYZI> &map_voi_body, pcl::PointCloud<pcl::PointXYZI> &outskirts);

        void visualize_path(
            nav_msgs::Path &path, std::string mode,
            const remover::node &node, const Eigen::Matrix4d &poses);

        void publish(
            const pcl::PointCloud<pcl::PointXYZI> &map,
            const ros::Publisher &publisher);
       
        void visualize_points_cloud();

    public:
        OfflineMapUpdater();

        ~OfflineMapUpdater();

        void save_global_map(float voxel_size);
        void Run_Dynamic_Objects_Remover();

    };

}

#endif
