#include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <octomap_msgs/Octomap.h>
// #include <tf/transform_listener.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

class OctreeGen{
    public:
        OctreeGen(){
            minBound = octomap::point3d( 0.0, -1.0, -1.5);  // lower bound
            maxBound = octomap::point3d( 1.5,  1.0,  0.5);  // upper bound
            tree = new octomap::OcTree(0.05);               // resolution
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
                TFlistener->lookupTransform("base", msg->header.frame_id, msg->header.stamp, transform);
                pcl_ros::transformPointCloud("base", *msg, transformed_cloud, *TFlistener);
            } catch (tf::TransformException &ex) {
                ROS_WARN("Warn during transform from '%s' to 'base': %s", msg->header.frame_id.c_str(), ex.what());
                return;
            }
            
            sensor_msgs::PointCloud2 processed_cloud;
            PointCloudProcess(transformed_cloud, processed_cloud);
            cloudDiff();

            octomap::Pointcloud octomapCloud;
            octomap::pointCloud2ToOctomap(processed_cloud, octomapCloud);
            tree->insertPointCloud(octomapCloud, octomap::point3d(0,0,0));

            octomap_msgs::fullMapToMsg(*tree, octree_msg);
            octree_msg.header.stamp = ros::Time::now();
            octree_msg.header.frame_id = transformed_cloud.header.frame_id;
        };

        void PointCloudProcess(const sensor_msgs::PointCloud2& msg, sensor_msgs::PointCloud2& Pmsg){
            ROS_INFO("PointCloud Process");
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(msg, *cloud1);

            pcl::CropBox<pcl::PointXYZ> filter1;
            filter1.setInputCloud(cloud1);
            filter1.setMin(Eigen::Vector4f( 0.0, -1.0, -1.5, 1.0));
            filter1.setMax(Eigen::Vector4f( 1.5,  1.0,  0.5, 1.0));
            filter1.filter(*cloud2);
            cloud1->clear();

            pcl::UniformSampling<pcl::PointXYZ> filter2;
            filter2.setInputCloud(cloud2);
            filter2.setRadiusSearch(0.03);
            filter2.filter(*cloud1);
            cloud2->clear();

            pcl::VoxelGrid<pcl::PointXYZ> filter3;
            filter3.setInputCloud(cloud1);
            filter3.setLeafSize(0.05, 0.05, 0.05);
            filter3.filter(*cloud2);
            cloud = *cloud2;
            if (last_cloud.size() == 0) last_cloud = cloud;

            pcl::toROSMsg(cloud, Pmsg);
            cloud_msg = Pmsg;
            cloud_msg.header.stamp = msg.header.stamp;
            cloud_msg.header.frame_id = msg.header.frame_id;
        };

        void cloudDiff (void) {
            ROS_INFO("Cloud Diff");
            pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(0.5);
            octree.setInputCloud(cloud.makeShared());
            octree.addPointsFromInputCloud();
            octree.switchBuffers();
            octree.setInputCloud(last_cloud.makeShared());
            octree.addPointsFromInputCloud();
            std::vector<int> newPointIdxVector;
            octree.getPointIndicesFromNewVoxels(newPointIdxVector);
            pcl::copyPointCloud(cloud, newPointIdxVector, diff_cloud);
            last_cloud = cloud;

            pcl::toROSMsg(diff_cloud, diff_cloud_msg);
            diff_cloud_msg.header.stamp = cloud_msg.header.stamp;
            diff_cloud_msg.header.frame_id = cloud_msg.header.frame_id;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> last_cloud;
        pcl::PointCloud<pcl::PointXYZ> diff_cloud;
        octomap::OcTree* tree;
        sensor_msgs::PointCloud2 cloud_msg;
        sensor_msgs::PointCloud2 diff_cloud_msg;
        octomap_msgs::Octomap octree_msg;

    private:
        tf::TransformListener* TFlistener;
        octomap::point3d minBound;
        octomap::point3d maxBound;
};