#include <stdio.h>
#include <string.h>
#include <stdlib.h>


//ROS

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kdl/frames_io.hpp>

//OPENCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


//ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

//CUSTOM NODES
#include <lwr_controllers/PoseRPY.h>
#include "lar_tools.h"
#include "lar_vision/commons/lar_vision_commons.h"
#include "lar_vision/segmentation/HighMap.h"
#include <dynamic_reconfigure/server.h>
#include <trimod_gripper/VisionConfig.h>

using namespace std;
using namespace lar_vision;
using namespace lar_tools;

//ROS
ros::NodeHandle* nh;


//CLOUDS & VIEWER
pcl::visualization::PCLVisualizer* viewer;
pcl::PointCloud<PointType>::Ptr cloud_full(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_filtered(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_full_filtered(new pcl::PointCloud<PointType>);
pcl::PointCloud<NormalType>::Ptr cloud_full_normals(new pcl::PointCloud<NormalType>);
pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
std::string save_folder;

//PARAMETERS
double slice_size;
double offset;
double reduction;
double filter_leaf;
double min_z;
double max_z;
bool doslice = false;
bool grasp = false;
bool grasp_alpha;
bool grasp_delta;


ros::Subscriber sub_cloud;
ros::Subscriber sub_pose;
image_transport::Subscriber sub_rgb;
image_transport::Subscriber sub_depth;

lwr_controllers::PoseRPY pose;
sensor_msgs::JointState joint_msg;
std::vector<lwr_controllers::PoseRPY> poses;

/** TRANSFORMS */
Eigen::Matrix4f T_0_ROBOT;
Eigen::Matrix4f T_EE;
Eigen::Matrix4f T_0_CAMERA;


void updateSegmentation(){
        //Normals
        compute_normals(cloud_full_filtered, cloud_full_normals);

        //Segmentation
        pcl::PointCloud<PointType>::Ptr planes(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr clusters(new pcl::PointCloud<PointType>());
        std::vector<int> filtered_indices;
        std::vector<int> planes_indices;

        HighMap map(2.0f, slice_size, offset, reduction);
        map.planesCheck(
                cloud_full_filtered,
                cloud_full_normals,
                filtered_indices,
                planes_indices,
                10.0f,
                500
                );
        pcl::copyPointCloud(*cloud_full_filtered, filtered_indices, *clusters);
        pcl::copyPointCloud(*cloud_full_filtered, planes_indices, *planes);


        display_cloud(*viewer, planes, 255,255,255, 1, "planes");

        //CLUSTERIZATION
        Palette palette;
        std::vector<pcl::PointIndices> cluster_indices;
        clusterize(clusters, cluster_indices);
        Eigen::Vector3f gravity;
        gravity << 0, 0, 1;
        for (int i = 0; i < cluster_indices.size(); i++) {
                pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>());
                pcl::copyPointCloud(*clusters, cluster_indices[i].indices, *cluster);

                std::string name = "cluster_" + boost::lexical_cast<std::string>(i);
                Eigen::Vector3i color = palette.getColor();
                display_cloud(*viewer, cluster, color[0], color[1], color[2], 1, name);

                name = "cluster_rf_" + boost::lexical_cast<std::string>(i);
                Eigen::Vector4f centroid;
                Eigen::Vector3f center;
                pcl::ReferenceFrame rf;
                pcl::ReferenceFrame rf_oriented;
                pcl::compute3DCentroid(*cluster, centroid);
                center << centroid[0], centroid[1], centroid[2];
                compute_centroid_local_rf(cluster, rf);
                compute_centroid_local_rf(cluster, rf_oriented, gravity);

                Eigen::Matrix4f fake_rf;
                rotation_matrix_4x4('x', 0, fake_rf);
                convert_rf_to_eigen_4x4(rf_oriented, fake_rf, true);

                draw_reference_frame(*viewer, center, rf_oriented, 0.1f, name);
                draw_text_3D(*viewer,name,center,255,255,255,0.01f,name+"text");
        }
}

void
cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud);

        pcl::transformPointCloud(*cloud, *cloud_trans, T_0_CAMERA);

        // Create the filtering object
        double leaf =0.01;
        nh->param<double>("filter_leaf", leaf, 0.01);

        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud (cloud_trans);
        sor.setLeafSize (leaf,leaf,leaf);
        sor.filter (*cloud_trans_filtered);

        (*cloud_full) += (*cloud_trans_filtered);
        sor.setInputCloud (cloud_full);
        sor.setLeafSize (leaf,leaf,leaf);
        sor.filter (*cloud_full_filtered);


        pcl::PassThrough<PointType> pass;
        pass.setInputCloud (cloud_full_filtered);
        pass.setFilterFieldName ("z");
      
        std::cout << "Filtering: "<<min_z<<" / "<<max_z<<std::endl;
        pass.setFilterLimits (min_z, max_z);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_full_filtered);

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        updateSegmentation();
        //lar_vision::display_cloud(*viewer, cloud_full_filtered, 0, 255, 0, 3.0f, "view");
        //viewer->addPointCloud(cloud, "scene");
}

cv::Mat current_rgb;
cv::Mat current_depth;
bool saving = false;
bool rgb_ready = false;
bool depth_ready = false;

void
rgb_cb(const sensor_msgs::ImageConstPtr& msg) {
        if (saving) return;
        try {
                rgb_ready = false;
                current_rgb = cv_bridge::toCvCopy(msg, "bgr8")->image;
                rgb_ready = true;
        } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
}

void
depth_cb(const sensor_msgs::ImageConstPtr& msg) {
        if (saving) return;
        try {
                depth_ready = false;
                current_depth = cv_bridge::toCvCopy(msg, "32FC1")->image;
                depth_ready = true;
        } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
}

void
pose_cb(const lwr_controllers::PoseRPY& pose) {

        lar_tools::create_eigen_4x4(
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.roll,
                pose.orientation.pitch,
                pose.orientation.yaw,
                T_0_CAMERA
                );

        T_0_CAMERA = T_0_ROBOT * T_0_CAMERA;
        T_0_CAMERA = T_0_CAMERA * T_EE;
}
int save_counter = 0;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void* viewer_void) {
        //    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.getKeySym() == "v" && event.keyDown()) {
                if (cloud->points.size() > 1000) {
                        //(*cloud_full) += (*cloud_trans);
                        //            //
                        saving = true;
                        while (!rgb_ready) ;
                        while (!depth_ready) ;
                        std::string save_folder = "/home/daniele/temp/0.000000000";

                        for (int i = 0; i < 5; i++) {
                                std::ofstream myfile;
                                std::stringstream ss;
                                ss << save_folder << "/" << save_counter << ".txt";

                                myfile.open(ss.str().c_str());
                                myfile << T_0_CAMERA;
                                myfile.close();
                                //            //
                                //            //
                                ss.str("");
                                ss << save_folder << "/" << save_counter << ".png";
                                cv::imwrite(ss.str(), current_rgb);

                                ss.str("");
                                ss << save_folder << "/" << save_counter << "_depth.png";
                                cv::Mat ucharMat;
                                current_depth.convertTo(ucharMat, CV_16UC1, 65535,0);
                                ucharMat =  cv::Scalar::all(65535) - ucharMat;
                                cv::imwrite(ss.str(), ucharMat);

                                ss.str("");
                                ss << save_folder << "/" << save_counter << ".pcd";
                                pcl::io::savePCDFileBinary(ss.str().c_str(), *cloud);

                                //            //
                                //            //                        std::cout << "Saved snapshot: " << save_counter << std::endl;
                                save_counter++;
                        }
                        saving = false;
                        //        }
                }
        }
}

/** MAIN NODE **/
int
main(int argc, char** argv) {

        // Initialize ROS
        ros::init(argc, argv, "comau_clusterizer");
        ROS_INFO("comau_clusterizer node started...");
        nh = new ros::NodeHandle();

        nh->param<double>("slice_size", slice_size, 0.01);
        nh->param<double>("offset", offset, 0.0);
        nh->param<double>("reduction", reduction, 1.01);
        nh->param<double>("filter_leaf", filter_leaf, 0.0051);
        nh->param<double>("minz", min_z, 0.4f);
        nh->param<double>("maxz", max_z, 3.0f);
        nh->param<bool>("doslice", doslice, false);
        nh->param<bool>("dograsp", grasp, false);
        nh->param<bool>("grasp_alpha", grasp_alpha, 0.1f);
        nh->param<bool>("grasp_delta", grasp_delta, 0.005f);

        //dynamic_reconfigure::Server<node_example::node_example_paramsConfig> dr_srv;
        dynamic_reconfigure::Server<trimod_gripper::VisionConfig> srv;
        /** VIEWER */
        viewer = new pcl::visualization::PCLVisualizer("viewer");
        viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

        /** TRANSFORMS */
        lar_tools::create_eigen_4x4(0, 0, 2.0f, 0, M_PI, 0, T_0_ROBOT);
        lar_tools::create_eigen_4x4(0, 0, 0, 0, 0, -M_PI / 2, T_EE);

        //        std::stringstream ss;
        //        ss << "/home/daniele/temp/" << ros::Time::now();
        //        save_folder = ss.str();
        //
        //        T <<
        //        1, 0, 0, 0,
        //        0, 1, 0, 0,
        //        0, 0, 1, 0,
        //        0, 0, 0, 1;


        //        transformMatrixT(0, 0, 2.0f, 0, M_PI, 0, Robot_Base);
        //        rotationMatrixT(0, 0, -M_PI / 2, EE);
        //        transformMatrixT(0, 0, 1.0f, 0, 0, 0, Target);


        //        boost::filesystem::create_directory(save_folder);

        sub_cloud = nh->subscribe("/xtion/xtion/depth/points", 1, cloud_cb);
        sub_pose = nh->subscribe("/lwr/full_control_simple/current_pose", 1, pose_cb);

        image_transport::ImageTransport it(*nh);
        sub_rgb = it.subscribe("/xtion/xtion/rgb/image_raw", 1, rgb_cb);
        sub_depth = it.subscribe("/xtion/xtion/depth/image_raw", 1, depth_cb);

        // Spin

        // Spin
        while (nh->ok() && !viewer->wasStopped()) {

                viewer->spinOnce();
                ros::spinOnce();
        }

}
