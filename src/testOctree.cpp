#include "OctreeGen.h"

int main (int argc, char** argv) {
    ros::init(argc, argv, "testOctree");
    ros::NodeHandle nh;
    OctreeGen octreeGen;
    // message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/points2", 1);
    // message_filters::Subscriber<airobots_calvin::MotorStatus> motor_sub(nh, "/armR/motorstatus", 1);
    // message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, airobots_calvin::MotorStatus> sync(points_sub, motor_sub, 10);
    // sync.registerCallback(boost::bind(&OctreeGen::PointCloudCallback, &octreeGen, _1, _2));
    ros::Subscriber sub = nh.subscribe("/points2", 1, &OctreeGen::PointCloudCallback, &octreeGen);
    ros::Subscriber sub2 = nh.subscribe("/armR/motorstatus", 1, &OctreeGen::MotorCallback, &octreeGen);
    ros::Subscriber sub3 = nh.subscribe("/ball", 1, &OctreeGen::BallCallback, &octreeGen);
    ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
    ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2>("/diff_cloud", 1);
    ros::Publisher pub4 = nh.advertise<sensor_msgs::PointCloud2>("/ori_cloud", 1);
    ros::Rate rate(30);
    while(ros::ok()){
        pub.publish(octreeGen.octree_msg);
        pub2.publish(octreeGen.curr_cloud_msg);
        pub3.publish(octreeGen.diff_cloud_msg);
        pub4.publish(octreeGen.ori_cloud_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}