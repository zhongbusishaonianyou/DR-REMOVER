#include "DR_remover/remover.h"

using namespace std;
using namespace Eigen;
DR_REMOVER::DR_REMOVER()
{
}

DR_REMOVER::~DR_REMOVER()
{
}

void DR_REMOVER::dis_and_yaw(const float &x, const float &y, float &dis, int &yaw_idx)
{
    dis = sqrt(pow(x, 2) + pow(y, 2));
    float yaw = atan2(y, x) * 180.0 / M_PI + 180;
    yaw_idx = min(static_cast<int>(yaw / 3.0), 119);
}
void DR_REMOVER::clear(pcl::PointCloud<pcl::PointXYZI> &pt_cloud)
{
    if (!pt_cloud.empty())
    {
        pt_cloud.clear();
    }
}
void DR_REMOVER::clear(CART_ROG &CART_ROG)
{
    if (!CART_ROG.empty())
    {
        CART_ROG.clear();
    }
}
void DR_REMOVER::init_ROG(CART_ROG &CART_ROG, const int num_x, const int num_y)
{
    clear(CART_ROG);
    Strip strip;
    GROUP group = {-INF, INF, 0, 0, static_cast<bool>(NOT_ASSIGNED)};
    group.points.reserve(ENOUGH_NUM);
    for (int i = 0; i < num_y; i++)
    {
        strip.emplace_back(group);
    }
    for (int j = 0; j < num_x; j++)
    {
        CART_ROG.emplace_back(strip);
    }
}

void DR_REMOVER::init_zoning_model_parms()
{
    N_cx = Nc / Nx;
    N_cy = Nc / Ny;
    group_length = 2 * max_r / Nx;
    group_width = 2 * max_r / Ny;
    cell_size = 2 * max_r / Nc;
}

void DR_REMOVER::clear_group(CART_ROG &CART_ROG, const int num_x, const int num_y)
{
    for (int x_idx = 0; x_idx < num_x; ++x_idx)
    {
        for (int y_idx = 0; y_idx < num_y; ++y_idx)
        {
            GROUP &group = CART_ROG[x_idx][y_idx];
            group.max_h = -INF;
            group.min_h = INF;
            group.x = 0;
            group.y = 0;
            group.status = NOT_ASSIGNED;
            clear(group.points);
        }
    }
}

void DR_REMOVER::Create_Zoning_Model(
    const pcl::PointCloud<pcl::PointXYZI> &map_voi,
    const pcl::PointCloud<pcl::PointXYZI> &query_voi)
{

    clear(debug_map_rejected);
    clear(viz_ground_pts);
    clear(map_voi_complement);

    object_pos = max_r * VectorXf::Ones(120, 1);
    GROUP_STATUS = Eigen::MatrixXi::Zero(Nc, Ny);

    clear_group(CART_ROG_MAP, Nx, Ny);
    clear_group(CART_ROG_SCAN, Nx, Ny);
    clear_group(CART_ROC_MAP, Nc, Nc);

    Pointcloud_to_CART_ROG(query_voi, CART_ROG_SCAN);
    Pointcloud_to_CART_ROG(map_voi, CART_ROG_MAP, map_voi_complement);
}

bool points_compare(pcl::PointXYZI a, pcl::PointXYZI b)
{
    return a.z < b.z;
}

void DR_REMOVER::VOI_to_CART_ROG(const pcl::PointXYZI &pt, GROUP &group)
{
    group.points.push_back(pt);

    if (pt.z >= group.max_h)
    {
        group.max_h = pt.z;
        group.x = pt.x;
        group.y = pt.y;
    }
    if (pt.z <= group.min_h)
    {
        group.min_h = pt.z;
    }
}
void DR_REMOVER::Pointcloud_to_CART_ROG(const pcl::PointCloud<pcl::PointXYZI> &src, CART_ROG &CART_ROG)
{
    for (auto const &pt : src.points)
    {
        if (pt.z < max_h && pt.z > min_h & abs(pt.x) < max_r && (abs(pt.y) < max_r))
        {
            int x_idx = floor((pt.x + max_r) / group_length);
            int y_idx = floor((pt.y + max_r) / group_width);

            if (pt.z >= Kappa)
            {
                int idx;
                float dis;
                dis_and_yaw(pt.x, pt.y, dis, idx);
                if (dis > 2)
                    object_pos(idx) = min(object_pos(idx), dis);
            }
            VOI_to_CART_ROG(pt, CART_ROG.at(x_idx).at(y_idx));
        }
    }
}
void DR_REMOVER::Pointcloud_to_CART_ROC(GROUP &group, CART_ROG &CART_ROC)
{
    if (group.status == is_divided)
        return;
    for (auto const &pt : group.points)
    {

        int x_idx = floor((pt.x + max_r) / cell_size);
        int y_idx = floor((pt.y + max_r) / cell_size);

        VOI_to_CART_ROG(pt, CART_ROC.at(x_idx).at(y_idx));
    }
    group.status = is_divided;
}
void DR_REMOVER::Pointcloud_to_CART_ROG(const pcl::PointCloud<pcl::PointXYZI> &src,
                                        CART_ROG &CART_ROG, pcl::PointCloud<pcl::PointXYZI> &complement)
{
    for (auto const &pt : src.points)
    {
        if (pt.z < max_h && pt.z > min_h && abs(pt.x) < max_r && abs(pt.y) < max_r)
        {

            int x_idx = floor((pt.x + max_r) / group_length);
            int y_idx = floor((pt.y + max_r) / group_width);

            VOI_to_CART_ROG(pt, CART_ROG.at(x_idx).at(y_idx));
        }
        else
        {
            complement.push_back(pt);
        }
    }
}

void DR_REMOVER::estimate_ground_plane(const pcl::PointCloud<pcl::PointXYZI> &inliers)
{
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;

    pcl::computeMeanAndCovarianceMatrix(inliers, cov, pc_mean);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    plane_normal = (svd.matrixU().col(2));
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
    // according to normal.T*[x,y,z] = -d
    plane_cofficent_d = -(plane_normal.transpose() * seeds_mean)(0, 0);
    // set distance threhold to `th_dist - d`
    plane_dis_thr = gf_dist_thr - plane_cofficent_d;
}

void DR_REMOVER::extract_initial_seeds_(
    const pcl::PointCloud<pcl::PointXYZI> &p_sorted, pcl::PointCloud<pcl::PointXYZI> &init_seeds)
{

    float ave_height = 0;
    for (int i = 0; i < gf_num_pts; i++)
    {
        ave_height += p_sorted.points[i].z / gf_num_pts;
    }

    clear(init_seeds);
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < ave_height + gf_thres_seeds)
        {
            init_seeds.push_back(p_sorted.points[i]);
        }
    }
}
void DR_REMOVER::extract_ground(const pcl::PointCloud<pcl::PointXYZI> &src,
                                pcl::PointCloud<pcl::PointXYZI> &fitting_ground,
                                pcl::PointCloud<pcl::PointXYZI> &non_fitting)
{

    clear(non_fitting);
    clear(fitting_ground);

    pcl::PointCloud<pcl::PointXYZI> outliers;
    pcl::PointCloud<pcl::PointXYZI> inliers;

    pointcloud_segmentation(src, outliers, inliers, ground_level);
    sort(inliers.points.begin(), inliers.points.end(), points_compare);

    if (inliers.points.size() < gf_num_pts)
    {
        non_fitting = inliers;
        fitting_ground = outliers;
    }
    else
    {
        auto points = inliers.getMatrixXfMap(3, 8, 0).transpose();
        extract_initial_seeds_(inliers, fitting_ground);
        for (int i = 0; i < gf_iter_times; i++)
        {
            estimate_ground_plane(fitting_ground);
            clear(fitting_ground);
            Eigen::VectorXf result = points * plane_normal;
            for (int r = 0; r < result.rows(); r++)
            {
                if (result[r] < plane_dis_thr)
                {
                    fitting_ground.push_back(inliers[r]);
                }
                else
                {
                    if (i == (gf_iter_times - 1))
                    {
                        non_fitting.push_back(inliers[r]);
                    }
                }
            }
        }

        if (plane_normal(2, 0) > 0.9)
        {
            fitting_ground += outliers;
            gf_success = true;
        }
        else
        {
            non_fitting = inliers;
            fitting_ground = outliers;
        }
    }
}

void DR_REMOVER::Occupancy_Height_Ratio(
    const GROUP group_map, const GROUP group_curr, bool &OHR_FLAG)
{
    float Delta_M = (group_map.max_h - group_map.min_h);
    float Delta_Q = (group_curr.max_h - group_curr.min_h);
    float Beta_h = abs(group_map.min_h - group_curr.min_h);
   
    float factor = 1.0 / (1.0 + Beta_h);
    float OHR = factor * (Delta_M / Delta_Q);

    if (OHR > Thres && Beta_h < 0.5 && Delta_M > 0.5)
    {
        OHR_FLAG = true;
    }
}

void DR_REMOVER::Fully_Occlusion_Distance(
    const GROUP group_map, const GROUP group_curr, bool &FOD_FLAG)
{

    float dis;
    int idx;

    dis_and_yaw(group_map.x, group_map.y, dis, idx);

    float FOD = object_pos(idx) - dis - 2 * max_r / Nx;

    if (FOD > 0 && dis < max_r)
        FOD_FLAG = true;
}

void DR_REMOVER::estimate_adjacent_height(const int x_idx, const int y_idx)
{
    float h_left, h_right, h_front, h_back;
    int N_cx = Nc / Nx;
    int N_cy = Nc / Ny;
    VectorXf surrounding_h = max_h * VectorXf::Ones(4, 1);

    Pointcloud_to_CART_ROC(CART_ROG_MAP[x_idx][y_idx + 1], CART_ROC_MAP);
    Pointcloud_to_CART_ROC(CART_ROG_MAP[x_idx][y_idx - 1], CART_ROC_MAP);

    for (int i = 0; i < N_cx; i++)
    {
        h_left = CART_ROC_MAP[N_cx * x_idx + i][N_cy * (y_idx + 1)].max_h;
        h_right = CART_ROC_MAP[N_cx * x_idx + i][N_cy * y_idx - 1].max_h;

        if (h_left != -INF)
            surrounding_h(0) = min(surrounding_h(0), h_left);
        if (h_right != -INF)
            surrounding_h(2) = min(surrounding_h(2), h_right);
    }

    Pointcloud_to_CART_ROC(CART_ROG_MAP[x_idx + 1][y_idx], CART_ROC_MAP);
    Pointcloud_to_CART_ROC(CART_ROG_MAP[x_idx - 1][y_idx], CART_ROC_MAP);

    for (int j = 0; j < N_cy; j++)
    {
        h_front = CART_ROC_MAP[N_cx * (x_idx + 1)][N_cy * y_idx + j].max_h;
        h_back = CART_ROC_MAP[N_cx * x_idx - 1][N_cy * y_idx + j].max_h;

        if (h_front != -INF)
            surrounding_h(1) = min(surrounding_h(1), h_front);
        if (h_back != -INF)
            surrounding_h(3) = min(surrounding_h(3), h_back);
    }

    sort(surrounding_h.data(), surrounding_h.data() + surrounding_h.size());
    adjacent_height = surrounding_h(1);
}

void DR_REMOVER::Compare_points_distribution(
    const int x_idx, const int y_idx, const float group_h, bool &overlap_flag)
{
    MatrixXi scan_distribution = MatrixXi::Zero(N_cx, N_cy);
    MatrixXi map_distribution = MatrixXi::Zero(N_cx, N_cy);

    for (int i = 0; i < N_cx; i++)
    {
        for (int j = 0; j < N_cy; j++)
        {
            float cell_h = CART_ROC_MAP[(N_cx * x_idx + i)][(N_cy * y_idx + j)].max_h;

            float ratio_h = (group_h - cell_h) / group_h;

            if (ratio_h < 0.5)
                map_distribution(i, j) = 1;
        }
    }

    GROUP group_scan = CART_ROG_SCAN[x_idx][y_idx];
    for (auto const &pt : group_scan.points)
    {
        int x_pos = fmod(floor((pt.x + max_r) / cell_size), N_cx);
        int y_pos = fmod(floor((pt.y + max_r) / cell_size), N_cy);
        scan_distribution(x_pos, y_pos) = 1;
    }

    MatrixXi overlap = map_distribution.cwiseProduct(scan_distribution);

    if (overlap.sum() != 0)
        overlap_flag = true;
}
/*
void DR_REMOVER::remove_sparse_scanning(
    const int x_idx, const int y_idx, const MatrixXi distribution, const double scan_max_h, bool &AOR_FLAG)
{
    VectorXi::Index maxRow;
    VectorXi num_occupancy = VectorXi::Zero(4, 1);
    MatrixXf cell_max_h = MatrixXf::Zero(N_cx, N_cy);

    num_occupancy(0) = distribution.topRows(N_cx / 2).sum();
    num_occupancy(1) = distribution.bottomRows(N_cx / 2).sum();
    num_occupancy(2) = distribution.leftCols(N_cy / 2).sum();
    num_occupancy(3) = distribution.rightCols(N_cy / 2).sum();

    double max_occupancy = (double)num_occupancy.maxCoeff(&maxRow) / distribution.sum();

    int dir_x[4] = {-1, 1, 0, 0};
    int dir_y[4] = {0, 0, -1, 1};

    GROUP group_scan = CART_ROG_SCAN[x_idx + dir_x[maxRow]][y_idx + dir_y[maxRow]];

    if (max_occupancy > 0.8 && (group_scan.points.size()==0))
    {
        AOR_FLAG = false;
    }
    else
    {
        AOR_FLAG = true;
    }
}
*/
void DR_REMOVER::estimate_ground_height(const int x_idx, const int y_idx)
{
    int static_area_num = 0;
    ground_level = 0;

    Pointcloud_to_CART_ROC(CART_ROG_MAP[x_idx][y_idx], CART_ROC_MAP);

    for (int i = 0; i < N_cx; i++)
    {
        for (int j = 0; j < N_cy; j++)
        {
            GROUP cell_map = CART_ROC_MAP[(N_cx * x_idx + i)][(N_cy * y_idx + j)];

            if (cell_map.max_h > min_h && cell_map.max_h < adjacent_height)
            {
                static_area_num++;
                ground_level += cell_map.max_h;
            }
        }
    }

    if (static_area_num != 0)
    {
        ground_level = ground_level / static_area_num;
    }
    else
    {
        ground_level = adjacent_height;
    }
}
void DR_REMOVER::remove_aggregate_points(const pcl::PointCloud<pcl::PointXYZI> &low_interval,
                                         pcl::PointCloud<pcl::PointXYZI> &remove_points,
                                         pcl::PointCloud<pcl::PointXYZI> &revert_points)
{
    clear(remove_points);
    clear(revert_points);

    // step1: set input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(low_interval.size());
    pcl::copyPointCloud(low_interval, *cloud);

    // step2: build kd-tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // step3: set cluster object
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ground_cluster;
    ground_cluster.setClusterTolerance(0.3);
    ground_cluster.setMinClusterSize(10);
    ground_cluster.setMaxClusterSize(2500);
    ground_cluster.setSearchMethod(tree);
    ground_cluster.setInputCloud(cloud);
    ground_cluster.extract(cluster_indices);

    // step4: find max cluster
    int max_cluster_size = 0;
    int max_cluster_index = -1;
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        if (cluster_indices[i].indices.size() > max_cluster_size)
        {
            max_cluster_size = cluster_indices[i].indices.size();
            max_cluster_index = i;
        }
    }
    if (max_cluster_index == -1)
    {
        remove_points = low_interval;
        return;
    }

    // step5: save max cluseter points as ground points
    for (const auto &idx : cluster_indices[max_cluster_index].indices)
    {
        revert_points.push_back(low_interval.points[idx]);
    }

    // step6: remover other clusters points
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        if (i != max_cluster_index) // jump max cluster
        {
            for (const auto &idx : cluster_indices[i].indices)
            {
                remove_points.push_back(low_interval.points[idx]);
            }
        }
    }
}
void DR_REMOVER::pointcloud_segmentation(const pcl::PointCloud<pcl::PointXYZI> &input,
                                         pcl::PointCloud<pcl::PointXYZI> &output1,
                                         pcl::PointCloud<pcl::PointXYZI> &output2,
                                         const float h)
{
    clear(output1);
    clear(output2);
    for (const auto &pt : input.points)
    {
        if (pt.z < h)
        {
            output1.push_back(pt);
        }
        else
        {
            output2.push_back(pt);
        }
    }
}

void DR_REMOVER::spatial_distribution_test(const pcl::PointCloud<pcl::PointXYZI> &none_ground,
                                           pcl::PointCloud<pcl::PointXYZI> &dynamic_points,
                                           pcl::PointCloud<pcl::PointXYZI> &revert_ground_pts)
{
    clear(dynamic_points);
    clear(revert_ground_pts);

    int num_points = none_ground.size();

    if (num_points < minimum_num_pts)
    {
        dynamic_points = none_ground;
        return;
    }

    pcl::PointCloud<pcl::PointXYZI> revert_part;
    pcl::PointCloud<pcl::PointXYZI> remove_part;

    if (gf_success)
    {
        auto pts_matrix = none_ground.getMatrixXfMap(3, 8, 0).transpose();
        Eigen::VectorXf normal_dis = pts_matrix * plane_normal;

        float pt_min_h = normal_dis.minCoeff();
        float pt_max_h = normal_dis.maxCoeff();
        float interval_thres = (pt_max_h - pt_min_h) / 4;

        for (int r = 0; r < normal_dis.rows(); r++)
        {
            if (normal_dis[r] < pt_min_h + interval_thres)
            {
                revert_part.push_back(none_ground[r]);
            }
            else
            {
                remove_part.push_back(none_ground[r]);
            }
        }
        gf_success = false;
    }
    else
    {
        float pt_min_h = none_ground.points[0].z;
        float pt_max_h = none_ground.points[num_points - 1].z;
        float interval_thres = (pt_max_h - pt_min_h) / 4;

        pointcloud_segmentation(none_ground, revert_part, remove_part, pt_min_h + interval_thres);
    }

    float VOF = revert_part.size() / static_cast<float>(num_points);
    if (VOF > 0.5)
    {
        remove_aggregate_points(revert_part, dynamic_points, revert_ground_pts);
        dynamic_points += remove_part;
    }
    else
    {
        dynamic_points = none_ground;
    }
}

void DR_REMOVER::Group_Dynamic_Region_Detection()
{

    for (int x_idx = 1; x_idx < Nx - 1; x_idx++)
    {
        for (int y_idx = 1; y_idx < Ny - 1; y_idx++)
        {

            GROUP &group_scan = CART_ROG_SCAN[x_idx][y_idx];
            GROUP &group_map = CART_ROG_MAP[x_idx][y_idx];

            if (group_map.points.empty())
                continue;

            if (group_scan.points.size() < minimum_num_pts)
            {
                GROUP_STATUS(x_idx, y_idx) = STATIC;
            }
            else
            {
                bool OHR_FLAG_ = false;
                bool FOD_FLAG_ = false;

                GROUP_STATUS(x_idx, y_idx) = STATIC;
                // OHR dynamic Group judgment
                Occupancy_Height_Ratio(group_map, group_scan, OHR_FLAG_);
                // FOD dynamic Group judgment
                Fully_Occlusion_Distance(group_map, group_scan, FOD_FLAG_);

                if (OHR_FLAG_ && FOD_FLAG_)
                {
                    bool overlap_FLAG_ = false;
                    bool AOR_FLAG_ = false;

                    Pointcloud_to_CART_ROC(group_map, CART_ROC_MAP);
                    // it's not included in our paper
                    Compare_points_distribution(x_idx, y_idx, group_map.max_h, overlap_FLAG_);
                    // AOR dynamic Group judgment (this is different from paper's method)
                    // remove_sparse_scanning(x_idx, y_idx, points_distribution, group_scan.max_h, AOR_FLAG_);
                    if (overlap_FLAG_)
                    {
                        GROUP_STATUS(x_idx, y_idx) = DYNAMIC;
                    }
                }
            }
        }
    }
}

void DR_REMOVER::Cell_Dynamic_Object_Removal(pcl::PointCloud<pcl::PointXYZI> &preserved_submap_pts)
{
    pcl::PointCloud<pcl::PointXYZI> fitted_ground_pts, unfitted_points;
    pcl::PointCloud<pcl::PointXYZI> remove_dynamic_pts, revert_ground_pts;

    clear(preserved_submap_pts);
    preserved_submap_pts.reserve(2000000);

    for (int x_idx = 0; x_idx < Nx; x_idx++)
    {
        for (int y_idx = 0; y_idx < Ny; y_idx++)
        {
            GROUP group_map = CART_ROG_MAP[x_idx][y_idx];

            int OCCUPANCY_STATUS = GROUP_STATUS(x_idx, y_idx);

            if (OCCUPANCY_STATUS == DYNAMIC)
            {
               // cout << "----------------------------------" << endl;
               // cout << "the dynamic position:" << group_map.x << " " << group_map.y << endl;
                // provide ground level
                estimate_adjacent_height(x_idx, y_idx);
                // static cell max height
                estimate_ground_height(x_idx, y_idx);
                // Elevation Filtering before fitting ground plane;
                extract_ground(group_map.points, fitted_ground_pts, unfitted_points);
                // revert ground points
                spatial_distribution_test(unfitted_points, remove_dynamic_pts, revert_ground_pts);

                preserved_submap_pts += (fitted_ground_pts + revert_ground_pts);

                viz_ground_pts += fitted_ground_pts;

                debug_map_rejected += remove_dynamic_pts;

                //cout << "remove_dynamic_pts size:" << remove_dynamic_pts.size() << endl;
            }
            else
            {
                preserved_submap_pts += group_map.points;
            }
        }
    }
    preserved_submap_pts += map_voi_complement;
}

void DR_REMOVER::visualize_outliers(
    pcl::PointCloud<pcl::PointXYZI> &map_rejected, pcl::PointCloud<pcl::PointXYZI> &map_ground)
{
    clear(map_rejected);
    clear(map_ground);

    map_rejected = debug_map_rejected;
    map_ground = viz_ground_pts;
}

/*used method in paper,we simfify this process.
void DR_REMOVER::CART_ROG_to_pointcloud(const CART_ROG &ROG, pcl::PointCloud<pcl::PointXYZI> &map_points)
{

    clear(map_points);

    for (int y_idx = 0; y_idx < Ny; y_idx++)
    {
        for (int x_idx = 0; x_idx < Nx; x_idx++)
        {
            if (ROG[x_idx][y_idx].is_occupied)
            {
                for (auto const &pt : ROG.at(x_idx).at(y_idx).points)
                {
                    map_points.points.push_back(pt);
                }
            }
        }
    }
}

void DR_REMOVER::analysis_occupy_cell(const int x_idx,const int y_idx,const double map_h,Eigen::MatrixXi &cell_status)
{
      double map_s_h;

         for (int i=0;i<8;i++)
            {
              for(int j=0;j<4;j++)
                    {

                    map_s_h = CART_ROC_MAP[(8*x_idx+i)][(4*y_idx+j)].max_h;

                    if(map_h-map_s_h<0.5*map_h)        cell_status(i,j)=1;


                    }
                }

}
void DR_REMOVER::Get_Area_Occupancy_Ratio(const int x_idx,const int y_idx,Group & cell_map,Eigen::MatrixXi cell_status){

     int  occupy_b=cell_status.row(0).sum()+cell_status.row(1).sum()+cell_status.row(2).sum()+cell_status.row(3).sum();
     int  occupy_f=cell_status.row(4).sum()+cell_status.row(5).sum()+cell_status.row(6).sum()+cell_status.row(7).sum();
     int  occupy_r=cell_status.col(0).sum()+cell_status.col(1).sum();
     int  occupy_l=cell_status.col(2).sum()+cell_status.col(3).sum();


        int num_occupy_cell=0;
        int x_i = fmod(floor((cell_map.x+max_r)/0.5),8);
        int y_i = fmod(floor((cell_map.y+max_r)/0.5),4);

         Eigen::MatrixXi near_cell_status1=MatrixXi::Zero(8,4);
         Eigen::MatrixXi near_cell_status2=MatrixXi::Zero(8,4);

         Eigen::MatrixXi merge_groups=MatrixXi::Zero(8,12);

               if(abs(occupy_b-occupy_f)>abs(occupy_l-occupy_r))
               {
                    if(occupy_b>occupy_f&&cell_status.row(0).sum()!=0)
                     {

                     if  (x_i>=4 ||CART_ROG_SCAN[x_idx-1][y_idx].points.size()==0)

                    {   cell_map.is_dynamic=0;return; }

                     merge_near_group(x_idx,y_idx,merge_groups);

                     seek_same_object(x_i+4,y_i+4,num_occupy_cell,merge_groups);

                    }
                    else if(occupy_b<occupy_f&&(cell_status.row(7).sum()!=0))
                     {

                     if  (x_i<4|| CART_ROG_SCAN[x_idx+1][y_idx].points.size()==0)

                     {  cell_map.is_dynamic=0; return;  }

                    merge_near_group(x_idx+1,y_idx,merge_groups);

                    seek_same_object(x_i-4,y_i+4,num_occupy_cell,merge_groups);

                     }
               }
               else
               {
                    if(occupy_r>occupy_l&&cell_status.col(0).sum()!=0)
                    {
                      if  (CART_ROG_SCAN[x_idx][y_idx-1].points.size()==0)

                     {  cell_map.is_dynamic=0; return;  }

                      analysis_occupy_cell(x_idx,y_idx-1,cell_map.max_h,near_cell_status1);
                      analysis_occupy_cell(x_idx,y_idx-2,cell_map.max_h,near_cell_status2);


                       merge_groups<<near_cell_status2,near_cell_status1,cell_status;
                       seek_same_object(x_i,y_i+8,num_occupy_cell,merge_groups);


                    }
                    else if(occupy_r<occupy_l&&cell_status.col(3).sum()!=0)
                    {

                      if  (CART_ROG_SCAN[x_idx][y_idx+1].points.size()==0)

                     {  cell_map.is_dynamic=0; return;  }

                      analysis_occupy_cell(x_idx,y_idx+1,cell_map.max_h,near_cell_status1);
                      analysis_occupy_cell(x_idx,y_idx+2,cell_map.max_h,near_cell_status2);

                      merge_groups<<cell_status, near_cell_status1,near_cell_status1;
                      seek_same_object(x_i,y_i,num_occupy_cell,merge_groups);

                    }
               }

                 if (num_occupy_cell>2*cell_status.sum())  cell_map.is_dynamic=0;

}
void DR_REMOVER::merge_near_group(const int x_idx,const int y_idx,Eigen::MatrixXi & merge_groups)
{

                double map_h=CART_ROG_MAP[x_idx][y_idx].max_h;

                    for (int i=-4;i<4;i++)
                 {
                    for(int j=-4;j<8;j++)
                    {

                    Group &cell_map_s = CART_ROC_MAP[(8*x_idx+i)][(4*y_idx+j)];


                     if(map_h-cell_map_s.max_h<0.5*map_h)
                     {
                        merge_groups(i+4,j+4)=1;

                       if (cell_map_s.max_h-map_h>0.5)
                       {
                          merge_groups(i+4,j+4)=0;
                        }

                     }
                    }
                }

}

void DR_REMOVER::revert_multiple_object_misdeletion(const int x_idx,const int y_idx,Eigen::MatrixXi cell_status)
{
                int num_occupy_cell=cell_status.sum();

                 if (cell_status.row(0).sum()!=0&&(cell_status.row(1).sum()==0||cell_status.row(2).sum()==0))
                 {
                     if(num_occupy_cell>3*cell_status.row(0).sum())
                     {

                         for(int j=0;j<4;j++)
                      {
                       CART_ROC_MAP[(8*x_idx)][(4*y_idx+j)].is_dynamic=1;
                      }


                     }
                 }
                 else if (cell_status.row(7).sum()!=0&&(cell_status.row(6).sum()==0||cell_status.row(5).sum()==0))
                 {
                     if(num_occupy_cell>3*cell_status.row(7).sum())
                     {

                         for(int j=0;j<4;j++)
                      {
                       CART_ROC_MAP[(8*x_idx+7)][(4*y_idx+j)].is_dynamic=1;
                     }

                     }
                 }

}

void DR_REMOVER::seek_same_object(const int x_idx,const int y_idx,int & occupy_cell,Eigen::MatrixXi occupy_block)
{


                std::queue <int> queue_x;
                std::queue <int> queue_y;
                queue_x.push(x_idx);
                queue_y.push(y_idx);

                while (!queue_x.empty()) {
                    int curr_x = queue_x.front();
                    int curr_y = queue_y.front();
                    queue_x.pop();
                    queue_y.pop();
                    if (curr_x < 0 || curr_y < 0 || curr_x == occupy_block.rows() || curr_y == occupy_block.cols()|| occupy_block(curr_x,curr_y) != 1) {
                        continue;
                    }
                    ++occupy_cell;
                    occupy_block(curr_x,curr_y) = 0;
                    int dir_x[4] = {0, 0, 1, -1};
                    int dir_y[4] = {1, -1, 0, 0};
                    for (int index = 0; index != 4; ++index) {
                        int next_x = curr_x + dir_x[index], next_y = curr_y+ dir_y[index];
                        queue_x.push(next_x);
                        queue_y.push(next_y);
                    }
                }

}
*/