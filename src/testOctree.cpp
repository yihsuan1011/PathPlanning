#include "OctreeGen.h"

// #include <ros/ros.h>
// // #include <sensor_msgs/PointCloud2.h>
// // #include <octomap_msgs/Octomap.h>
// // #include <tf/transform_listener.h>
// #include <octomap_ros/conversions.h>
// #include <octomap_msgs/conversions.h>
// #include <pcl_ros/transforms.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl/filters/uniform_sampling.h>
// #include <pcl/filters/voxel_grid.h>

// class OctreeGen{
//     public:
//         OctreeGen(){
//             minBound = octomap::point3d( 0.0, -1.0, -1.5);  // lower bound
//             maxBound = octomap::point3d( 1.5,  1.0,  0.5);  // upper bound
//             tree = new octomap::OcTree(0.05);               // resolution
//             tree->setBBXMin(minBound);
//             tree->setBBXMax(maxBound);
//             tree->useBBXLimit(true);
//             TFlistener = new tf::TransformListener();
//             octree_msg = octomap_msgs::Octomap();
//         };
//         ~OctreeGen(){
//             delete tree;
//             delete TFlistener;
//         };

//         void PointCloud2Octree(const sensor_msgs::PointCloud2ConstPtr& msg){
//             ROS_INFO("PointCloud to Octree");
//             sensor_msgs::PointCloud2 transformed_cloud;
//             tf::StampedTransform transform;
//             try {
//                 TFlistener->lookupTransform("base", msg->header.frame_id, msg->header.stamp, transform);
//                 pcl_ros::transformPointCloud("base", *msg, transformed_cloud, *TFlistener);
//             } catch (tf::TransformException &ex) {
//                 ROS_WARN("Warn during transform from '%s' to 'base': %s", msg->header.frame_id.c_str(), ex.what());
//                 return;
//             }
            
//             sensor_msgs::PointCloud2 processed_cloud;
//             PointCloudProcess(transformed_cloud, processed_cloud);

//             octomap::Pointcloud octomapCloud;
//             octomap::pointCloud2ToOctomap(processed_cloud, octomapCloud);
//             tree->insertPointCloud(octomapCloud, octomap::point3d(0,0,0));

//             octomap_msgs::fullMapToMsg(*tree, octree_msg);
//             octree_msg.header.stamp = transformed_cloud.header.stamp;
//             octree_msg.header.frame_id = transformed_cloud.header.frame_id;
//         };

//         void PointCloudProcess(const sensor_msgs::PointCloud2& msg, sensor_msgs::PointCloud2& Pmsg){
//             ROS_INFO("PointCloud Process");
//             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
//             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
//             pcl::fromROSMsg(msg, *cloud1);

//             pcl::CropBox<pcl::PointXYZ> filter1;
//             filter1.setInputCloud(cloud1);
//             filter1.setMin(Eigen::Vector4f( 0.0, -1.0, -1.5, 1.0));
//             filter1.setMax(Eigen::Vector4f( 1.5,  1.0,  0.5, 1.0));
//             filter1.filter(*cloud2);
//             cloud1->clear();

//             pcl::UniformSampling<pcl::PointXYZ> filter2;
//             filter2.setInputCloud(cloud2);
//             filter2.setRadiusSearch(0.03);
//             filter2.filter(*cloud1);
//             cloud2->clear();

//             pcl::VoxelGrid<pcl::PointXYZ> filter3;
//             filter3.setInputCloud(cloud1);
//             filter3.setLeafSize(0.05, 0.05, 0.05);
//             filter3.filter(*cloud2);
//             cloud = *cloud2;

//             pcl::toROSMsg(cloud, Pmsg);
//             cloud_msg = Pmsg;
//             cloud_msg.header.stamp = msg.header.stamp;
//             cloud_msg.header.frame_id = msg.header.frame_id;
//         };

//         pcl::PointCloud<pcl::PointXYZ> cloud;
//         sensor_msgs::PointCloud2 cloud_msg;
//         octomap_msgs::Octomap octree_msg;

//     private:
//         octomap::OcTree* tree;
//         tf::TransformListener* TFlistener;
//         octomap::point3d minBound;
//         octomap::point3d maxBound;
// };

int main (int argc, char** argv) {
    ros::init(argc, argv, "testOctree");
    ros::NodeHandle nh;
    OctreeGen octreeGen;
    ros::Subscriber sub = nh.subscribe("/points2", 1, &OctreeGen::PointCloud2Octree, &octreeGen);
    ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("/processed_cloud", 1);
    ros::Rate rate(30);
    while(ros::ok()){
        pub.publish(octreeGen.octree_msg);
        pub2.publish(octreeGen.cloud_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}