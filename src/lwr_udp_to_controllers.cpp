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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <kdl/frames_io.hpp>

#include "lar_tool_utils/UDPNode.h"

#define COMMAND_UPDATE_JOINTS 666

using namespace std;
using namespace lar_tools;


ros::NodeHandle* nh;
ros::Publisher joints_publisher;
lar_tools::UDPNode* node;

/** RPY Pose */
lwr_controllers::PoseRPY pose;

/** JOIN MESSAGE*/
sensor_msgs::JointState joint_msg;

void updateJoints() {

    joint_msg.name.resize(7);
    joint_msg.name[0] = "lwr_0_joint";
    joint_msg.name[1] = "lwr_1_joint";
    joint_msg.name[2] = "lwr_2_joint";
    joint_msg.name[3] = "lwr_3_joint";
    joint_msg.name[4] = "lwr_4_joint";
    joint_msg.name[5] = "lwr_5_joint";
    joint_msg.name[6] = "lwr_6_joint";
    joint_msg.position.resize(7);

    for (int i = 0; i < 7; i++) {
        joint_msg.position[i] = 0;
    }

    while (nh->ok() && node->isReady()) {
        
        joints_publisher.publish(joint_msg);
        ros::spinOnce();
    }
}

/** MAIN NODE **/
int
main(int argc, char** argv) {

    
    // Initialize ROS
    ros::init(argc, argv, "lwr_udp_to_controllers");
    ROS_INFO("lwr_udp_to_controllers node started...");
    nh = new ros::NodeHandle();
    joints_publisher = nh->advertise<sensor_msgs::JointState>("/lwr/full_control_simple/command_joints", 1);

    /* UDP NODE*/
    ROS_INFO("UDP Node creating...");
    node = new lar_tools::UDPNode(9999);
    lar_tools::UDPNodeMessage message;
    ROS_INFO("UDP Node created!");   
        
    /* PUBLISHING THREAD */
    boost::thread updateTFsThread(updateJoints);
    
    // Spin
    while (nh->ok() && node->isReady()) {
        /* RECEIVE MESSAGE*/
        ROS_INFO("Waiting message.");
        node->receiveMessage(message);
        ROS_INFO("Received/n command:%d\n  data:%f\n-----------\n", message.command, message.payload[0]);

        if (message.command == COMMAND_UPDATE_JOINTS) {
            for (int i = 0; i < 7; i++) {
                joint_msg.position[i] = message.payload[i]*M_PI/180.0f;
            }
        }

        ros::spinOnce();
    }

    ROS_INFO("Waiting for thread ends...");
    updateTFsThread.join();
}
