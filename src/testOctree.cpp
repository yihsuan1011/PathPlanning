#include "OctreeGen.h"

int main (int argc, char** argv) {
    ros::init(argc, argv, "testOctree");
    ros::NodeHandle nh;
    OctreeGen octreeGen;
    ros::Subscriber sub = nh.subscribe("/points2", 1, &OctreeGen::PointCloudCallback, &octreeGen);
    ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
    ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2>("/diff_cloud", 1);
    ros::Publisher pub4 = nh.advertise<sensor_msgs::PointCloud2>("/ori_cloud", 1);
    ros::Rate rate(30);
    while(ros::ok()){
        pub.publish(octreeGen.octree_msg);
        pub2.publish(octreeGen.cloud_msg);
        pub3.publish(octreeGen.diff_cloud_msg);
        pub4.publish(octreeGen.ori_cloud_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}