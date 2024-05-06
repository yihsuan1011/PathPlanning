#include <ros/ros.h>
#include <airobots_calvin/MotorInfo.h>
#include <airobots_calvin/MotorStatus.h>
#include <deque>
#include <mutex>
// #include <sensor_msgs/PointCloud2.h>
// #include <octomap_msgs/Octomap.h>
// #include <tf/transform_listener.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

class OctreeGen{
    public:
        OctreeGen(){
            static_flag = false;
            minBound = octomap::point3d( 0.0, -1.0, -1.5);  // lower bound
            maxBound = octomap::point3d( 1.5,  1.0,  0.5);  // upper bound
            float resolution = 0.05;
            ori_octree = new octomap::OcTree(resolution);
            ori_octree->setBBXMin(minBound);
            ori_octree->setBBXMax(maxBound);
            ori_octree->useBBXLimit(true);
            diff_octree = new octomap::OcTree(resolution);
            diff_octree->setBBXMin(minBound);
            diff_octree->setBBXMax(maxBound);
            diff_octree->useBBXLimit(true);
            TFlistener = new tf::TransformListener();
            joint_angles = vector<float>(6, 0);
            key_joint_pos = vector<Eigen::Vector3f>(4, Eigen::Vector3f(0, 0, 0));
            octree_msg = octomap_msgs::Octomap();
        };
        ~OctreeGen(){
            delete ori_octree;
            delete diff_octree;
            delete TFlistener;
        };

        void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
            // ROS_INFO("PointCloud to Octree");
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
            PointCloudProcess(transformed_cloud);
            if(!static_flag)
                StaticDectect();
            else {
                cloudDiff(cloud, ori_cloud, diff_cloud);
                PointCloud2Octree(diff_cloud_msg, diff_octree);
                if(ori_octree == NULL)
                    PointCloud2Octree(ori_cloud_msg, ori_octree);
            }
            
            // octomap_msgs::fullMapToMsg(*tree, octree_msg);
            // octree_msg.header.stamp = ros::Time::now();
            // octree_msg.header.frame_id = transformed_cloud.header.frame_id;
        };

        void MotorCallback(const airobots_calvin::MotorStatusConstPtr& msg){
            std::lock_guard<std::mutex> lock(mtx);
            for(auto& motor : msg->status){
                switch(motor.id){
                    case 23:
                        joint_angles[0] = motor.angle;
                        break;
                    case 24:
                        joint_angles[1] = motor.angle;
                        break;
                    case 25:
                        joint_angles[2] = motor.angle;
                        break;
                    case 26:
                        joint_angles[3] = motor.angle;
                        break;
                    case 27:
                        joint_angles[4] = motor.angle;
                        break;
                    case 28:
                        joint_angles[5] = motor.angle;
                        break;
                    case 31:
                        break;
                    default:
                        ROS_ERROR("Invalid id: %i",motor.id);
                        break;
                }
            }
            getJointPosition();
        };

        void getJointPosition() {
            float CORE_LINK_LENGTH_ = 175;
	        float SHOULDER_LINK_LENGTH_ = 122;
	        float UPPER_LINK_LENGTH_ = 257;
	        float END_EFFECTOR_LENGTH_ = 320;
            float J1 = joint_angles[0] * Angle2Rad;
            float J2 = joint_angles[1] * Angle2Rad;
            float J3 = joint_angles[2] * Angle2Rad;
            float J4 = joint_angles[3] * Angle2Rad;
            float J5 = joint_angles[4] * Angle2Rad;
            float J6 = joint_angles[5] * Angle2Rad;
            Eigen::Matrix<float, 4, 4> Tb0;
            Tb0 << 1, 0, 0, 0,
                0, -1, 0, -CORE_LINK_LENGTH_,
                0, 0, -1, 0,
                0, 0, 0, 1;
            Eigen::Matrix<float, 4, 4> T01 = GetTransformMatrix_Craig(0, 0, J1, 0);
            Eigen::Matrix<float, 4, 4> T12 = GetTransformMatrix_Craig(-M_PI_2, 0, J2 - M_PI_2, 0);
            Eigen::Matrix<float, 4, 4> T225 = GetTransformMatrix_Craig(-M_PI_2, 0, -M_PI_2, 0);
            Eigen::Matrix<float, 4, 4> T253 = GetTransformMatrix_Craig(0, SHOULDER_LINK_LENGTH_, J3 + M_PI_2, 0);
            Eigen::Matrix<float, 4, 4> T335 = GetTransformMatrix_Craig(0, UPPER_LINK_LENGTH_, M_PI_2, 0);
            Eigen::Matrix<float, 4, 4> T354 = GetTransformMatrix_Craig(M_PI_2, 0, J4 + M_PI_2, 0);
            Eigen::Matrix<float, 4, 4> T45 = GetTransformMatrix_Craig(-M_PI_2, 0, J5, 0);
            Eigen::Matrix<float, 4, 4> T56 = GetTransformMatrix_Craig(M_PI_2, 0, J6, 0);
            Eigen::Matrix<float, 4, 4> T6E;
            T6E << 0, 0, 1, 0,
                0, -1, 0, 0,
                1, 0, 0, END_EFFECTOR_LENGTH_,
                0, 0, 0, 1;

            /* transform matrix based on the 1st coordination */
            Eigen::Matrix<float, 4, 4> Tb1 = Tb0 * T01;
            Eigen::Matrix<float, 4, 4> Tb2 = Tb1 * T12;
            Eigen::Matrix<float, 4, 4> Tb3 = Tb2 * T225 * T253;
            Eigen::Matrix<float, 4, 4> Tb4 = Tb3 * T335 * T354;
            Eigen::Matrix<float, 4, 4> Tb5 = Tb4 * T45;
            Eigen::Matrix<float, 4, 4> Tb6 = Tb5 * T56;
            Eigen::Matrix<float, 4, 4> TbE = Tb6 * T6E;
            
            /* positions based on the 1st coordination */
            // Eigen::Vector3f Pb0(0, 0, 0);
            Eigen::Vector3f Pb1(Tb1(0, 3), Tb1(1, 3), Tb1(2, 3));
            // Eigen::Vector3f Pb2(Tb2(0, 3), Tb2(1, 3), Tb2(2, 3));
            Eigen::Vector3f Pb3(Tb3(0, 3), Tb3(1, 3), Tb3(2, 3));
            Eigen::Vector3f Pb4(Tb4(0, 3), Tb4(1, 3), Tb4(2, 3));
            // Eigen::Vector3f Pb5(Tb5(0, 3), Tb5(1, 3), Tb5(2, 3));
            // Eigen::Vector3f Pb6(Tb6(0, 3), Tb6(1, 3), Tb6(2, 3));
            Eigen::Vector3f PbE(TbE(0, 3), TbE(1, 3), TbE(2, 3));
            key_joint_pos = {Pb1, Pb3, Pb4, PbE};
        };

        Eigen::Matrix<float, 4, 4> GetTransformMatrix_Craig(const float &alpha_1, const float &a_1, const float &theta, const float &d) {
            Eigen::Matrix<float, 4, 4> transform_matrix;
            transform_matrix << cos(theta), -sin(theta), 0, a_1,
                sin(theta) * cos(alpha_1), cos(theta) * cos(alpha_1), -sin(alpha_1), -d * sin(alpha_1),
                sin(theta) * sin(alpha_1), cos(theta) * sin(alpha_1), cos(alpha_1), cos(alpha_1) * d,
                0, 0, 0, 1;

            return transform_matrix;
        };

        void PointCloudProcess(const sensor_msgs::PointCloud2& msg){
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
            cloud1->clear();

            ArmFiler(cloud2, cloud1);

            cloud = *cloud1;
            if (ori_cloud.size() == 0) ori_cloud = cloud;

            pcl::toROSMsg(cloud, cloud_msg);
            cloud_msg.header.stamp = msg.header.stamp;
            cloud_msg.header.frame_id = msg.header.frame_id;
        };

        void ArmFiler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud) {
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(cloud);
            vector<int> toRemove;

            vector<int> pointIdxRadiusSearch;
            vector<float> pointRadiusSquaredDistance;
            float radius = 0.1;
            if (kdtree.radiusSearch(pcl::PointXYZ(key_joint_pos[0].x(), key_joint_pos[0].y(), key_joint_pos[0].z()), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
                    pcl::PointXYZ point = cloud->points[pointIdxRadiusSearch[i]];
                    if (isArm(point))
                        toRemove.push_back(pointIdxRadiusSearch[i]);
                }
            }
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            inliers->indices = toRemove;
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*filtered_cloud);
        };

        bool isArm(const pcl::PointXYZ& point) {
            Eigen::Vector3f p(point.x, point.y, point.z);
            float d1 = DisPoint2Link(p, key_joint_pos[0], key_joint_pos[1]);
            float d2 = DisPoint2Link(p, key_joint_pos[1], key_joint_pos[2]);
            float d3 = DisPoint2Link(p, key_joint_pos[2], key_joint_pos[3]);
            if (d1 < 0.1 || d2 < 0.1 || d3 < 0.1) return true;
            else return false;
        };

        float DisPoint2Link(const Eigen::Vector3f& P, const Eigen::Vector3f& A, const Eigen::Vector3f& B) {
            Eigen::Vector3f AB = B - A;
            Eigen::Vector3f AP = P - A;
            float t = AP.dot(AB) / AB.squaredNorm();
            if (t < 0) return AP.norm();
            else if (t > 1) return (P - B).norm();
            else return (AP - t * AB).norm();
        };

        void StaticDectect(void) {
            cloudDiff(cloud, ori_cloud, diff_cloud);
            ori_cloud = cloud;
            // ROS_INFO("Diff: %lu", diff_cloud.size());
            diffs.push_back(diff_cloud.size());
            if (diffs.size() > 30) diffs.pop_front();
            ROS_INFO("Size: %lu, Sum of diff: %u", diffs.size(), std::accumulate(diffs.begin(), diffs.end(), 0));
            if (diffs.size() == 30 && std::accumulate(diffs.begin(), diffs.end(), 0) < 100) static_flag = true;
        };

        void cloudDiff(pcl::PointCloud<pcl::PointXYZ>& cloud1, pcl::PointCloud<pcl::PointXYZ>& cloud2, pcl::PointCloud<pcl::PointXYZ>& diff) {
            pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(0.5);
            octree.setInputCloud(cloud1.makeShared());
            octree.addPointsFromInputCloud();
            octree.switchBuffers();
            octree.setInputCloud(cloud2.makeShared());
            octree.addPointsFromInputCloud();
            std::vector<int> newPointIdxVector;
            octree.getPointIndicesFromNewVoxels(newPointIdxVector);
            pcl::copyPointCloud(cloud1, newPointIdxVector, diff);

            pcl::toROSMsg(diff_cloud, diff_cloud_msg);
            diff_cloud_msg.header.stamp = cloud_msg.header.stamp;
            diff_cloud_msg.header.frame_id = cloud_msg.header.frame_id;

            pcl::toROSMsg(ori_cloud, ori_cloud_msg);
            ori_cloud_msg.header.stamp = cloud_msg.header.stamp;
            ori_cloud_msg.header.frame_id = cloud_msg.header.frame_id;
        };

        void PointCloud2Octree(const sensor_msgs::PointCloud2& cloud, octomap::OcTree* tree){
            octomap::Pointcloud octomapCloud;
            octomap::pointCloud2ToOctomap(cloud, octomapCloud);
            tree->insertPointCloud(octomapCloud, octomap::point3d(0,0,0));
        };

        bool static_flag;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> ori_cloud;
        pcl::PointCloud<pcl::PointXYZ> diff_cloud;
        std::deque<int> diffs;
        octomap::OcTree* ori_octree;
        octomap::OcTree* diff_octree;
        sensor_msgs::PointCloud2 cloud_msg;
        sensor_msgs::PointCloud2 ori_cloud_msg;
        sensor_msgs::PointCloud2 diff_cloud_msg;
        octomap_msgs::Octomap octree_msg;

    private:
        tf::TransformListener* TFlistener;
        vector<float> joint_angles;
        vector<Eigen::Vector3f> key_joint_pos;
        std::mutex mtx;
        octomap::point3d minBound;
        octomap::point3d maxBound;
};