#include <ros/ros.h>
// PCL specific includes

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <lwr_controllers/PoseRPY.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include "lar_tools.h"
#include <opencv2/opencv.hpp>
#include <kdl/frames_io.hpp>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

using namespace std;

pcl::visualization::PCLVisualizer* viewer;
typedef pcl::PointXYZRGBA PointType;
pcl::PointCloud<PointType>::Ptr cloud_full(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_full_filtered(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
std::string save_folder;
int save_counter = 0;



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

void
cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*input, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);

    pcl::transformPointCloud(*cloud, *cloud_trans, T_0_CAMERA);

    viewer->removeAllPointClouds();
    viewer->addPointCloud(cloud_trans, "scene");
    viewer->addPointCloud(cloud_full, "scene_full");
}

void
rgb_cb(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat source =cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow("rgb", source);
        cv::waitKey(30);
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

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
        void* viewer_void) {
    //    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getKeySym() == "v" && event.keyDown()) {
        if (cloud->points.size() > 1000) {
            (*cloud_full) += (*cloud_trans);
            //            //
            //            //                        std::ofstream myfile;
            //            //                        std::stringstream ss;
            //            //                        ss << save_folder << "/" << save_counter << ".txt";
            //            //
            //            //                        myfile.open(ss.str().c_str());
            //            //                        myfile << T;
            //            //                        myfile.close();
            //            //
            //            //
            //            //                        ss.str("");
            //            //                        ss << save_folder << "/" << save_counter << ".pcd";
            //            //                        pcl::io::savePCDFileASCII(ss.str().c_str(), *cloud);
            //            //
            //            //                        std::cout << "Saved snapshot: " << save_counter << std::endl;
            //            //                        save_counter++;
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
//    nh = new ros::NodeHandle();
    ros::NodeHandle nh;
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

    sub_cloud = nh.subscribe("/xtion/xtion/depth/points", 1, cloud_cb);
    sub_pose = nh.subscribe("/lwr/full_control_simple/current_pose", 1, pose_cb);
    
    image_transport::ImageTransport it(nh);
//    sub_rgb = it.subscribe("/xtion/xtion/rgb/image_raw", 1, rgb_cb);

    // Spin

    // Spin
    while (nh.ok() && !viewer->wasStopped()) {

        viewer->spinOnce();
        ros::spinOnce();
    }

}
