#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <tools/remover_utils.hpp>
#include <string>
#include <map>
#include <vector>

ros::Publisher TP_Publisher;
ros::Publisher FP_Publisher;

ros::Publisher TN_Publisher;
ros::Publisher FN_Publisher;

sensor_msgs::PointCloud2 TP;
sensor_msgs::PointCloud2 TN;

sensor_msgs::PointCloud2 FP;
sensor_msgs::PointCloud2 FN;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapviz");
    ros::NodeHandle nodeHandler;

    std::string static_map_name, dynamic_map_name;

    nodeHandler.param<std::string>("/static_map_name", static_map_name, "/");
    nodeHandler.param<std::string>("/dynamic_map_name", dynamic_map_name, "/");

    TP_Publisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/remove_dynamic_pts", 100);
    TN_Publisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/preserved_static_pts", 100);

    FP_Publisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/remove_static_pts", 100);
    FN_Publisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/preserved_dynamic_pts", 100);

    pcl::PointCloud<pcl::PointXYZI>::Ptr build_static_map(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr build_dynamic_map(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI> preserved_dynamic_pts, preserved_static_pts, target_obj;
    pcl::PointCloud<pcl::PointXYZI> remove_dynamic_pts, remove_static_pts;

    remover_utils::load_pcd(static_map_name, build_static_map);
    remover_utils::load_pcd(dynamic_map_name, build_dynamic_map);

    remover_utils::separate_static_and_dynamic_pts(*build_static_map, preserved_dynamic_pts, preserved_static_pts);
    remover_utils::separate_static_and_dynamic_pts(*build_dynamic_map, remove_dynamic_pts, remove_static_pts);

    TN = remover_utils::cloud2msg(preserved_static_pts);
    FN = remover_utils::cloud2msg(preserved_dynamic_pts);

    TP = remover_utils::cloud2msg(remove_dynamic_pts);
    FP = remover_utils::cloud2msg(remove_static_pts);
   
    cout << "\033[47;36m--------------------------DR-REMOVER visualize results--------------------" << "\033[0m" << endl;

    cout << "The number of preserved static points from raw map:\033[1;36m" << preserved_static_pts.size() << "\033[0m" << endl;
    cout << "The number of removed dynamic points from raw map:\033[1;36m" << remove_dynamic_pts.size() << "\033[0m" << endl;
    cout << "The number of preserved dynamic points from raw map:\033[1;36m" << preserved_dynamic_pts.size() << "\033[0m" << endl;
    cout << "The number of removed static points from raw map:\033[1;36m" << remove_static_pts.size() << "\033[0m" << endl;
    cout << "------------------------------------------------"<< endl;
    TP_Publisher.publish(TP);
    TN_Publisher.publish(TN);
    FP_Publisher.publish(FP);
    FN_Publisher.publish(FN);

    return 0;
}
