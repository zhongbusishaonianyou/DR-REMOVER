#include "tools/remover_utils.hpp"

#define INF 10000000000000.0
#define PI 3.1415926535
#define ENOUGH_NUM 8000

#define NOT_ASSIGNED 0
#define DYNAMIC 1
#define STATIC 2
#define is_divided 3

using namespace std;
using namespace Eigen;

struct GROUP
{
    float max_h;
    float min_h;
    float x;
    float y;
    int status;
    pcl::PointCloud<pcl::PointXYZI> points;
};
typedef vector<vector<GROUP>> CART_ROG;
typedef vector<GROUP> Strip;
class DR_REMOVER
{
public:
    DR_REMOVER(ros::NodeHandle *nodehandler) : nh(*nodehandler)
    {
        nh.param("/remover/L_max", max_r, 60.0);
        nh.param("/remover/max_h", max_h, 3.0);
        nh.param("/remover/min_h", min_h, 0.0);

        nh.param("/remover/N_x", Nx, 30);
        nh.param("/remover/N_y", Ny, 60);
        nh.param("/remover/N_c", Nc, 240);

        nh.param("/remover/minimum_num_pts", minimum_num_pts, 5);
        nh.param("/remover/Thres", Thres, 0.15);
        nh.param("/remover/Kappa", Kappa, 5.0);

        nh.param("/remover/gf_dist_thr", gf_dist_thr, 0.05);
        nh.param("/remover/gf_iter_times", gf_iter_times, 3);
        nh.param("/remover/gf_mini_num_points", gf_num_pts, 10);
        nh.param("/remover/gf_seeds_heigt_thr", gf_thres_seeds, 0.5);

        cout << "-------\033[47;36m Dual-resolution Zoning Model Parameters of DR-REMOVER\033[0m-----" << endl;
        cout << "The region of interest range :\033[1;34m " << max_r << "m\033[0m" << endl;
        cout << "The minimum height of the region of interest:\033[1;34m" << min_h << "m\033[0m" << endl;
        cout << "The maximum height of the region of interest:\033[1;34m" << max_h << "m\033[0m" << endl;
        cout << "Number of Groups divided in the X-axis:\033[1;34m" << Nx << "\033[0m" << endl;
        cout << "Number of Groups divided in the Y-axis:\033[1;34m" << Ny << "\033[0m" << endl;
        cout << "Number of Cells divided in the XY-axis:\033[1;34m" << Nc << "\033[0m" << endl;
        cout << "-------------------------------------------------------------------" << endl;

        init_zoning_model_parms();

        init_ROG(CART_ROG_MAP, Nx, Ny);
        init_ROG(CART_ROC_MAP, Nc, Nc);
        init_ROG(CART_ROG_SCAN, Nx, Ny);
    }
    DR_REMOVER();

    ~DR_REMOVER();

    void Create_Zoning_Model(
        const pcl::PointCloud<pcl::PointXYZI> &map_voi,
        const pcl::PointCloud<pcl::PointXYZI> &query_voi);

    void Group_Dynamic_Region_Detection();

    void Cell_Dynamic_Object_Removal(
        pcl::PointCloud<pcl::PointXYZI> &preserved_submap_pts);

    void visualize_outliers(
        pcl::PointCloud<pcl::PointXYZI> &map_rejected,
        pcl::PointCloud<pcl::PointXYZI> &map_ground);

    pcl::PointCloud<pcl::PointXYZI> viz_ground_pts;
    pcl::PointCloud<pcl::PointXYZI> debug_map_rejected;
    pcl::PointCloud<pcl::PointXYZI> map_voi_complement;

    CART_ROG CART_ROG_MAP;
    CART_ROG CART_ROC_MAP;
    CART_ROG CART_ROG_SCAN;

private:
    vector<int> DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259};
    ros::NodeHandle nh;

    // Zoning model parameters
    int Nx, Ny, Nc;
    int N_cx, N_cy;

    float group_length;
    float group_width;
    float cell_size;

    double max_r, min_h, max_h;
    // used for Group-based dynamic region detection
    int minimum_num_pts;
    double Thres;
    double Kappa;

    VectorXf object_pos;
    MatrixXi GROUP_STATUS;
    // used for ground fitting
    bool gf_success;

    int gf_iter_times;
    int gf_num_pts;

    double gf_dist_thr;
    double gf_thres_seeds;

    float plane_dis_thr;
    float plane_cofficent_d;

    // used for removing dynamic points
    float adjacent_height, ground_level;

    MatrixXf plane_normal;

    void init_zoning_model_parms();

    void init_ROG(
        CART_ROG &CART_ROG,
        const int num_x, const int num_y);

    void dis_and_yaw(
        const float &x, const float &y,
        float &dis, int &yaw_idx);

    void clear_group(
        CART_ROG &CART_ROG,
        const int num_x, const int num_y);

    void clear(pcl::PointCloud<pcl::PointXYZI> &pt_cloud);

    void clear(CART_ROG &CART_ROG);

    void VOI_to_CART_ROG(const pcl::PointXYZI &pt, GROUP &group);

    void Pointcloud_to_CART_ROG(
        const pcl::PointCloud<pcl::PointXYZI> &src,
        CART_ROG &CART_ROG);

    void Pointcloud_to_CART_ROC(
        GROUP &group, CART_ROG &CART_ROC);

    void Pointcloud_to_CART_ROG(
        const pcl::PointCloud<pcl::PointXYZI> &src,
        CART_ROG &CART_ROG,
        pcl::PointCloud<pcl::PointXYZI> &complement);

    void Occupancy_Height_Ratio(
        const GROUP group_map, const GROUP group_curr, bool &OHR_FLAG);

    void Fully_Occlusion_Distance(
        const GROUP group_map, const GROUP group_curr, bool &FOD_FLAG);

    void estimate_ground_plane(
        const pcl::PointCloud<pcl::PointXYZI> &map_ground);

    void estimate_adjacent_height(
        const int x_idx, const int y_idx);

    void Compare_points_distribution(
        const int x_idx, const int y_idx, const float edge_h, bool &overlap_flag);

    void estimate_ground_height(
        const int x_idx, const int y_idx);

    void spatial_distribution_test(
        const pcl::PointCloud<pcl::PointXYZI> &none_ground,
        pcl::PointCloud<pcl::PointXYZI> &dynamic_points,
        pcl::PointCloud<pcl::PointXYZI> &revert_ground_pts);

    void pointcloud_segmentation(
        const pcl::PointCloud<pcl::PointXYZI> &input,
        pcl::PointCloud<pcl::PointXYZI> &output1,
        pcl::PointCloud<pcl::PointXYZI> &output2,
        const float h);

    void remove_aggregate_points(
        const pcl::PointCloud<pcl::PointXYZI> &low_interval,
        pcl::PointCloud<pcl::PointXYZI> &remove_points,
        pcl::PointCloud<pcl::PointXYZI> &revert_points);

    void extract_initial_seeds_(
        const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
        pcl::PointCloud<pcl::PointXYZI> &init_seeds);

    void extract_ground(
        const pcl::PointCloud<pcl::PointXYZI> &src,
        pcl::PointCloud<pcl::PointXYZI> &fitting_ground,
        pcl::PointCloud<pcl::PointXYZI> &non_fitting);
    
   /*  void remove_sparse_scanning(
        const int x_idx, const int y_idx,
        const MatrixXi distribution, const double scan_max_h, bool &AOR_FLAG);
    */
   
    //void CART_ROG_to_pointcloud(
    //    const CART_ROG &ROG, pcl::PointCloud<pcl::PointXYZI> &map_points);

    /* used method in paper
   void Get_Area_Occupancy_Ratio(
       const int x_idx,const int y_idx,
      Group & cell_map,Eigen::MatrixXi cell_status);

   void analysis_occupy_cell(
      const int x_idx,const int y_idx,const double map_h,
      Eigen::MatrixXi &cell_status);

   void revert_multiple_object_misdeletion(
      const int x_idx,const int y_idx,Eigen::MatrixXi cell_status);

   void seek_same_object(
      const int x_idx,const int y_idx,int & occupy_cell,
      Eigen::MatrixXi  occupy_block);

   void merge_near_group(
      const int x_idx,const int y_idx,Eigen::MatrixXi & merge_groups);
  */
};
