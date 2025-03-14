#include <ros/ros.h>
#include "DR_remover/MapUpdater.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DR-REMOVER_STATIC_MAP_BUILDING");
    ros::NodeHandle nh;

    remover::OfflineMapUpdater updater = remover::OfflineMapUpdater();

    ros::spin();

    return 0;
}
