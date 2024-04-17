#include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>

class OctreeGen{
    public:
        OctreeGen(){
            tree = new octomap::OcTree(0.1);
            octree_msg = octomap_msgs::Octomap();
        };
        ~OctreeGen(){
            delete tree;
        };
        void PointCloud2Octree(const sensor_msgs::PointCloud2ConstPtr& msg){
            ROS_INFO("PointCloud to Octree");
            octomap::Pointcloud octomapCloud;
            octomap::pointCloud2ToOctomap(*msg, octomapCloud);
            tree->insertPointCloud(octomapCloud, octomap::point3d(0,0,0));
            octomap_msgs::fullMapToMsg(*tree, octree_msg);
            octree_msg.header.stamp = ros::Time::now();
            octree_msg.header.frame_id = "depth_camera_link";
        };

        octomap::OcTree* tree;
        octomap_msgs::Octomap octree_msg;
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