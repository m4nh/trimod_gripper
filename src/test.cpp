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

using namespace std;

void
pose_cb(const lwr_controllers::PoseRPY& pose) {
    KDL::Frame frame;

}





//ros::Publisher pose_publisher;
//ros::Publisher joints_publisher;
//ros::Subscriber sub;
//ros::Subscriber sub_pose;

lwr_controllers::PoseRPY pose;

void updateCommands() {


}

/** MAIN NODE **/
int
main(int argc, char** argv) {

    std::cout << "Testing node started...\n";
    // Initialize ROS
    ros::init(argc, argv, "");
    ros::NodeHandle nh;


    //        pose_publisher = nh.advertise<lwr_controllers::PoseRPY>("/lwr/full_control_simple/command", 1);
    //pose_publisher = nh.advertise<lwr_controllers::PoseRPY>("/lwr/one_task_inverse_kinematics/command", 1);

    //        joints_publisher = nh.advertise<sensor_msgs::JointState>("/lwr/full_control_simple/command_joints", 1);
    //        sub = nh.subscribe("/xtion/xtion/depth/points", 1, cloud_cb);
    //        sub_pose = nh.subscribe("/lwr/full_control_simple/current_pose", 1, pose_cb);


    // Spin

    boost::thread updateTFsThread(updateCommands);

    lar_tools::UDPNode node(9999);
    lar_tools::UDPNodeMessage message;
    
    // Spin
    while (nh.ok() && node.isReady()) {
        ROS_INFO("Waiting message.");
        node.receiveMessage(message);
        ROS_INFO("Received/n command:%d/n time:%d/n data:%f/n-----------/n",message.command,message.time,message.payload[0]);
        ros::spinOnce();
    }

    updateTFsThread.join();
}
