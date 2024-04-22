#include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <octomap_msgs/Octomap.h>
#include <tf/transform_listener.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class OctreeGen{
    public:
        OctreeGen(){
            // depth_camera_link
            // minBound = octomap::point3d(-1.0, -0.5, 0.0);   // lower bound
            // maxBound = octomap::point3d( 1.0,  1.5, 1.0);   // upper bound
            // base
            minBound = octomap::point3d( 0.0, -1.0, -1.5);  // lower bound
            maxBound = octomap::point3d( 3.0,  1.0,  0.5);  // upper bound
            tree = new octomap::OcTree(0.05);                // resolution
            tree->setBBXMin(minBound);
            tree->setBBXMax(maxBound);
            tree->useBBXLimit(true);
            TFlistener = new tf::TransformListener();
            octree_msg = octomap_msgs::Octomap();
        };
        ~OctreeGen(){
            delete tree;
            delete TFlistener;
        };

        void PointCloud2Octree(const sensor_msgs::PointCloud2ConstPtr& msg){
            ROS_INFO("PointCloud to Octree");
            sensor_msgs::PointCloud2 transformed_cloud;
            tf::StampedTransform transform;
            try {
                // if (!TFlistener->waitForTransform("base", msg->header.frame_id,
                //                                 msg->header.stamp, ros::Duration(1.0))) {
                //     ROS_ERROR("Unable to get pose from TF");
                //     return;
                // }
                TFlistener->lookupTransform("base", msg->header.frame_id, msg->header.stamp, transform);
                pcl_ros::transformPointCloud("base", *msg, transformed_cloud, *TFlistener);
                ROS_INFO("Transformed PointCloud");
            } catch (tf::TransformException &ex) {
                ROS_WARN("Warn during transform from '%s' to 'base': %s", msg->header.frame_id.c_str(), ex.what());
                return;
            }
            octomap::Pointcloud octomapCloud;
            octomap::pointCloud2ToOctomap(transformed_cloud, octomapCloud);
            ROS_INFO("Converted PointCloud to Octomap");
            tree->insertPointCloud(octomapCloud, octomap::point3d(0,0,0));

            octomap_msgs::fullMapToMsg(*tree, octree_msg);
            octree_msg.header.stamp = ros::Time::now();
            octree_msg.header.frame_id = transformed_cloud.header.frame_id;
        };
        octomap_msgs::Octomap octree_msg;

    private:
        octomap::OcTree* tree;
        tf::TransformListener* TFlistener;
        octomap::point3d minBound;
        octomap::point3d maxBound;
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "testOctree");
    ros::NodeHandle nh;
    OctreeGen octreeGen;
    ros::Subscriber sub = nh.subscribe("/points2", 1, &OctreeGen::PointCloud2Octree, &octreeGen);
    ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1);
    ros::Rate rate(30);
    while(ros::ok()){
        pub.publish(octreeGen.octree_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}