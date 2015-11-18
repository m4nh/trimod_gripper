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


//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/voxel_grid.h>

//CUSTOM NODES
#include <lwr_controllers/PoseRPY.h>
#include "lar_tools.h"
#include "lar_vision/commons/lar_vision_commons.h"
#include "lar_vision/commons/Noiser.h"
#include <dynamic_reconfigure/server.h>
#include <trimod_gripper/LwrManualPhotographerConfig.h>

using namespace std;
using namespace lar_vision;

//ROS
ros::NodeHandle* nh;


//CLOUDS & VIEWER
pcl::visualization::PCLVisualizer* viewer;

pcl::PointCloud<PointType>::Ptr cloud_full(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_noise(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_filtered(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_full_filtered(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
std::string save_folder;
Noiser noiser;


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




void callback(trimod_gripper::LwrManualPhotographerConfig& config, uint32_t level) {
        ROS_INFO("Reconfigure Request:");

        /*float axial_coefficient;
           float axial_bias;
           float axial_offset;
           float lateral_coefficient;
           float lateral_default_focal;*/

        noiser.axial_offset = config.noise_axial_offset;
        noiser.axial_coefficient = config.noise_axial_coefficient;
        noiser.axial_bias = config.noise_axial_bias;
        noiser.lateral_coefficient = config.noise_lateral_coefficient;
        noiser.lateral_default_focal = config.noise_lateral_focal;
}

void
cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud);

        noiser.setCloud(cloud);
        noiser.addNoise(cloud_noise);

        pcl::transformPointCloud(*cloud_noise, *cloud_trans, T_0_CAMERA);




        // Create the filtering object
        /*double leaf =0.02;
           nh->param<double>("filter_leaf", leaf, 0.02);

           pcl::VoxelGrid<PointType> sor;
           sor.setInputCloud (cloud_trans);
           sor.setLeafSize (leaf,leaf,leaf);
           sor.filter (*cloud_trans_filtered);

           (*cloud_full) += (*cloud_trans_filtered);
           sor.setInputCloud (cloud_full);
           sor.setLeafSize (leaf,leaf,leaf);
           sor.filter (*cloud_full_filtered);
         */

        viewer->removeAllPointClouds();

        //&lar_vision::display_cloud(*viewer, cloud_full_filtered, 0, 255, 0, 3.0f, "view");
        viewer->addPointCloud(cloud_trans, "scene");
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
        ros::init(argc, argv, "lwr_manual_photographer");
        ROS_INFO("lwr_manual_photographer node started...");
        nh = new ros::NodeHandle();

        dynamic_reconfigure::Server<trimod_gripper::LwrManualPhotographerConfig> srv;
        dynamic_reconfigure::Server<trimod_gripper::LwrManualPhotographerConfig>::CallbackType f;
        f = boost::bind(&callback, _1, _2);
        srv.setCallback(f);

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
