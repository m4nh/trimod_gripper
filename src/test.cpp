#include <ros/ros.h>
// PCL specific includes

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include "lar_tool_utils/UDPNode.h"
#include <boost/thread.hpp>

ros::Subscriber joint_states;
ros::Publisher joints_publisher;
ros::NodeHandle* nh;
lar_tools::UDPNode* node;


sensor_msgs::JointState joint_msg;

trajectory_msgs::JointTrajectory msg;

char mode = 'j';

struct TestGripper {
        float r;
        float d;
        float a;
        float b;
        float S;
        float tetha;
        float l1;
        float l2;

        TestGripper(float r,float l1,float l2){
                this->r = r;
                this->l1=l1;
                this->l2=l2;
                this->d=1.4641f*r;
                this->a=0.5774f*(this->d*0.5f+this->r);
                this->b=(this->d*0.5f+this->r)/0.8660f;
                this->S=(2.0f*r+this->a+this->b);
        }

        float ik_parallel_vertical(float Gr){
                return acos((-Gr+this->S)/(2*this->l1));
        }

};


void joint_state_cb( const control_msgs::JointTrajectoryControllerState& msg){

}

void updateJoints() {




        while (nh->ok() && node->isReady()) {

                joints_publisher.publish(msg);
                ros::spinOnce();
        }
}

/** MAIN NODE **/
int
main(int argc, char** argv) {

        std::cout << "Testing node started...\n";
        // Initialize ROS
        ros::init(argc, argv, "trimod_gripper");
        nh = new ros::NodeHandle();

        joint_states =  nh->subscribe("/trimod/joint_trajectory_controller/state", 1, joint_state_cb);
        joints_publisher = nh->advertise<trajectory_msgs::JointTrajectory>("/trimod/trimod_joint_trajectory_effort_controller/command",1);


        /* UDP NODE*/
        ROS_INFO("UDP Node creating...");
        node = new lar_tools::UDPNode(9999);
        lar_tools::UDPNodeMessage message;
        ROS_INFO("UDP Node created!");




        int i = 0;
        msg.joint_names.resize(9);
        msg.joint_names[i++] = "trimod_finger_left_joint_palm";
        msg.joint_names[i++] = "trimod_finger_left_joint_proximal";
        msg.joint_names[i++] = "trimod_finger_left_joint_distal";
        msg.joint_names[i++] = "trimod_finger_right_joint_palm";
        msg.joint_names[i++] = "trimod_finger_right_joint_proximal";
        msg.joint_names[i++] = "trimod_finger_right_joint_distal";
        msg.joint_names[i++] = "trimod_finger_center_joint_palm";
        msg.joint_names[i++] = "trimod_finger_center_joint_proximal";
        msg.joint_names[i++] = "trimod_finger_center_joint_distal";

        msg.points.resize(1);
        trajectory_msgs::JointTrajectoryPoint point;
        msg.points[0] = point;
        msg.points[0].time_from_start = ros::Duration(1);
        msg.points[0].positions.resize(9);
        for (size_t i = 0; i < 9; i++) {
                msg.points[0].positions[i] = 0;
        }



        /* PUBLISHING THREAD */
        boost::thread updateTFsThread(updateJoints);

        // Spin
        while (nh->ok() && node->isReady() ) {

                /* RECEIVE MESSAGE*/
                ROS_INFO("Waiting message.");
                  node->receiveMessage(message);
                ROS_INFO("Received/n command:");


                msg.points[0].positions[0]=message.payload[0]*M_PI/180.0f;
                msg.points[0].positions[3]=-message.payload[0]*M_PI/180.0f;

                msg.points[0].positions[1]=message.payload[1]*M_PI/180.0f;
                msg.points[0].positions[2]=-message.payload[2]*M_PI/180.0f;


                msg.points[0].positions[4]=message.payload[1]*M_PI/180.0f;
                msg.points[0].positions[5]=-message.payload[2]*M_PI/180.0f;

                msg.points[0].positions[7]=message.payload[1]*M_PI/180.0f;
                msg.points[0].positions[8]=-message.payload[2]*M_PI/180.0f;



                //std::cout << msg<<std::endl;
                ros::spinOnce();
        }
        ROS_INFO("Waiting for thread ends...");
        updateTFsThread.join();
}
