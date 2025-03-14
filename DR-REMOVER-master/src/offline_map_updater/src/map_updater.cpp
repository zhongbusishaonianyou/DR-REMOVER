#include "DR_remover/MapUpdater.h"

using namespace remover;
using namespace std;
OfflineMapUpdater::OfflineMapUpdater()
{
    sub_node_ = nh.subscribe<remover::node>("/node/combined/optimized", 2000, &OfflineMapUpdater::callback_node, this);
    sub_flag_ = nh.subscribe<std_msgs::Float32>("/saveflag", 10, &OfflineMapUpdater::callback_flag, this);

    pub_path_ = nh.advertise<nav_msgs::Path>("/MapUpdater/path_corrected", 100);
    pub_curr_scan = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/curr_scan", 100);
    pub_preserved_dynamic_pts = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/dynamic", 100);
    pub_preserved_static_pts = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/static", 100);

    pub_map_ground = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/ground", 100);

    pub_removed_static_pts = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/map_rejected", 100);

    initialize_global_ptrs();

    set_initial_parameters();

    load_global_map();

    remover_.reset(new DR_REMOVER(&nh));
}

OfflineMapUpdater::~OfflineMapUpdater()
{
}

void OfflineMapUpdater::initialize_global_ptrs()
{

    global_map.reset(new pcl::PointCloud<pcl::PointXYZI>());
    submap_arranged.reset(new pcl::PointCloud<pcl::PointXYZI>());
    submap_complement.reset(new pcl::PointCloud<pcl::PointXYZI>());

    query_voi_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_ground_to_viz.reset(new pcl::PointCloud<pcl::PointXYZI>());

    map_voi_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_voi_outskirts_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    map_filtered_body.reset(new pcl::PointCloud<pcl::PointXYZI>());

    curr_map_rejected.reset(new pcl::PointCloud<pcl::PointXYZI>());
    total_map_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void OfflineMapUpdater::set_initial_parameters()
{

    nh = ros::NodeHandle("~");

    nh.param("/MapUpdater/query_voxel_size", query_voxel_size_, 0.05);
    nh.param("/MapUpdater/map_voxel_size", map_voxel_size_, 0.05);

    nh.param("/MapUpdater/removal_interval", removal_interval_, 2);
    nh.param<std::string>("/MapUpdater/dataset_name", dataset_name_, "05");
    nh.param<std::string>("/MapUpdater/sequence_name", sequence_name_, "05");
    nh.param<std::string>("/MapUpdater/initial_map_path", map_name_, "/");
    nh.param<std::string>("/MapUpdater/save_path", save_path_, "/");

    nh.param("/large_scale/submap_size", submap_size_, 200.0);

    nh.param("/remover/L_max", L_max_, 60.0);
    nh.param("/remover/max_h", max_h_, 3.0);
    nh.param("/remover/min_h", min_h_, 0.0);
    nh.param<vector<float>>("/tf/tf_lidar2body", tf_lidar2body, vector<float>());

    tf_lidar2body_ = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(tf_lidar2body.data(), 4, 4);
}

void OfflineMapUpdater::load_global_map()
{

    cout << "Loading the global map..." << endl;

    global_map->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);

    int flag = remover_utils::load_pcd(map_name_, global_map);

    if (flag == -1)
    {
        throw invalid_argument("Read map file failed!");
    }
    else
    {
        cout << "Successfully loaded the global map!" << endl;
    }

    num_pts_map = global_map->points.size();
}

void OfflineMapUpdater::callback_flag(const std_msgs::Float32::ConstPtr &msg)
{
    save_global_map(msg->data);
}

void OfflineMapUpdater::save_global_map(float voxel_size)
{
    string static_map_name = save_path_ + "/" + dataset_name_ + "_" + sequence_name_ + "_result.pcd";
    string dynamic_map_name = save_path_ + "/" + dataset_name_ + "_" + sequence_name_ + "_dynamic.pcd";

    pcl::PointCloud<pcl::PointXYZI>::Ptr global_static_map(new pcl::PointCloud<pcl::PointXYZI>);
    global_static_map->reserve(num_pts_map);

    *global_static_map = *submap_arranged + *submap_complement;
    pcl::io::savePCDFileASCII(static_map_name, *global_static_map);
    cout << "\033[1;32mSuccessfully saved the global static map \033[0m" << endl;

    pcl::io::savePCDFileASCII(dynamic_map_name, *total_map_rejected_);
    cout << "\033[1;32mSuccessfully saved the removed points\033[0m" << endl;
}

void OfflineMapUpdater::callback_node(const remover::node::ConstPtr &msg)
{
    static int stack_count = 0;

    if ((stack_count++) % removal_interval_ == 0)
    {
        ROS_INFO_STREAM("\033[01;33m" << msg->header.seq << "th frame\033[0m");

        tf_body2origin_ = remover_utils::poseToEigenTransform(msg->odom).cast<float>();

        visualize_path(path_, "corrected", *msg, remover_utils::poseToEigenTransform(msg->odom));

        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query(new pcl::PointCloud<pcl::PointXYZI>);
        ptr_query->reserve(NUM_PTS_LARGE_ENOUGH);

        pcl::fromROSMsg(msg->lidar, *ptr_query);

        pcl::transformPointCloud(*ptr_query, *query_voi_, tf_lidar2body_);

        Run_Dynamic_Objects_Remover();
    }
    else
    {
        ROS_INFO_STREAM("\033[1;32mskip " << msg->header.seq << "th frame\033[0m");
    }
}

void OfflineMapUpdater::Run_Dynamic_Objects_Remover()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered_origin(new pcl::PointCloud<pcl::PointXYZI>);
    map_filtered_origin->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);

    float x_curr = tf_body2origin_(0, 3);
    float y_curr = tf_body2origin_(1, 3);

    auto start = ros::Time::now().toSec();

    reassign_submap(x_curr, y_curr);
    
    extract_map_voi(x_curr, y_curr, *map_voi_, *map_voi_outskirts_);
   
    remover_->Create_Zoning_Model(*map_voi_, *query_voi_);

    remover_->Group_Dynamic_Region_Detection();

    remover_->Cell_Dynamic_Object_Removal(*map_filtered_body);

    pcl::transformPointCloud(*map_filtered_body, *map_filtered_origin, tf_body2origin_);

    *submap_arranged = *map_filtered_origin + *map_voi_outskirts_;

    auto end = ros::Time::now().toSec();

    ROS_INFO_STREAM("\033[1;33m" << setw(22) << "DR-REMOVER takes " << end - start << "s to remove " << (*curr_map_rejected).size() << " points\033[0m");

    /*** Just for debugging and visualizing***/
    visualize_points_cloud();
}
void OfflineMapUpdater::reassign_submap(float pose_x, float pose_y)
{
    if (is_initialized_)
    {
        float diff_x = abs(submap_center_x_ - pose_x);
        float diff_y = abs(submap_center_y_ - pose_y);
        static float half_size = submap_size_ / 2.0;

        if ((diff_x > half_size) || (diff_y > half_size))
        {
            global_map.reset(new pcl::PointCloud<pcl::PointXYZI>());
            global_map->reserve(num_pts_map);

            *global_map = *submap_arranged + *submap_complement;
            obtain_submap_from_map(*global_map, *submap_arranged, *submap_complement, pose_x, pose_y);
            submap_center_x_ = pose_x;
            submap_center_y_ = pose_y;
        }
    }
    else
    {
        obtain_submap_from_map(*global_map, *submap_arranged, *submap_complement, pose_x, pose_y);
        submap_center_x_ = pose_x;
        submap_center_y_ = pose_y;
        is_initialized_ = true;
    }
}

void OfflineMapUpdater::obtain_submap_from_map(
    const pcl::PointCloud<pcl::PointXYZI> &map_global, pcl::PointCloud<pcl::PointXYZI> &submap,
    pcl::PointCloud<pcl::PointXYZI> &submap_complement,
    float x, float y)
{

    submap.clear();
    submap_complement.clear();
    submap.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    submap_complement.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);

    for (const auto pt : map_global.points)
    {
        float diff_x = abs(x - pt.x);
        float diff_y = abs(y - pt.y);
        if (diff_x < submap_size_ && diff_y < submap_size_)
        {
            submap.emplace_back(pt);
        }
        else
        {
            submap_complement.emplace_back(pt);
        }
    }
}
void OfflineMapUpdater::clear(pcl::PointCloud<pcl::PointXYZI> &pt_cloud)
{
    if (!pt_cloud.empty())
        pt_cloud.clear();
}
void OfflineMapUpdater::extract_map_voi(
    float curr_x, float curr_y, pcl::PointCloud<pcl::PointXYZI> &map_voi_body,
    pcl::PointCloud<pcl::PointXYZI> &outskirts)
{
    clear(map_voi_body);
    clear(outskirts);

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_voi_origin(new pcl::PointCloud<pcl::PointXYZI>);
    map_voi_origin->reserve(NUM_PTS_LARGE_ENOUGH);

    for (auto const &pt : submap_arranged->points)
    {
        float dis_x = abs(pt.x - curr_x);
        float dis_y = abs(pt.y - curr_y);

        if (dis_x < L_max_ && dis_y < L_max_)
        {
            map_voi_origin->points.emplace_back(pt);
        }
        else
        {
            outskirts.emplace_back(pt);
        }
    }
    pcl::transformPointCloud(*map_voi_origin, map_voi_body, tf_body2origin_.inverse());
}

void OfflineMapUpdater::visualize_path(
    nav_msgs::Path &path, std::string mode,
    const remover::node &node, const Eigen::Matrix4d &poses)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = node.header;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose = remover_utils::eigenMatrixToPose(poses);
    path.header = pose_stamped.header;
    path.poses.push_back(pose_stamped);
}

void OfflineMapUpdater::publish(
    const pcl::PointCloud<pcl::PointXYZI> &map,
    const ros::Publisher &publisher)
{
    sensor_msgs::PointCloud2 Msg;
    pcl::toROSMsg(map, Msg);
    Msg.header.frame_id = "map";
    publisher.publish(Msg);
}
void OfflineMapUpdater::visualize_points_cloud()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_viz(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_dynamic_pts(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_static_pts(new pcl::PointCloud<pcl::PointXYZI>);
    submap_dynamic_pts->reserve(500000);
    submap_static_pts->reserve(2000000);
    ptr_query_viz->reserve((*query_voi_).size());

    pcl::PointCloud<pcl::PointXYZI> removed_dynamic_pts;
    pcl::PointCloud<pcl::PointXYZI> removed_static_pts;

    remover_->visualize_outliers(*curr_map_rejected, *map_ground_to_viz);
    pcl::transformPointCloud(*map_ground_to_viz, *map_ground_to_viz, tf_body2origin_);

    publish(*map_ground_to_viz, pub_map_ground);

    remover_utils::separate_static_and_dynamic_pts(*submap_arranged, *submap_dynamic_pts, *submap_static_pts);
    publish(*submap_static_pts, pub_preserved_static_pts);
    publish(*submap_dynamic_pts, pub_preserved_dynamic_pts);

    pcl::transformPointCloud(*query_voi_, *ptr_query_viz, tf_body2origin_);
    publish(*ptr_query_viz, pub_curr_scan);

    pub_path_.publish(path_);

    pcl::transformPointCloud(*curr_map_rejected, *curr_map_rejected, tf_body2origin_);
    remover_utils::separate_static_and_dynamic_pts(*curr_map_rejected, removed_dynamic_pts, removed_static_pts);
    publish(removed_static_pts, pub_removed_static_pts);
    *total_map_rejected_ += *curr_map_rejected;
}