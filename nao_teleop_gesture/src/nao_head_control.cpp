//
// /*********************************************************************
// * Software License Agreement (BSD License)
// *
// * Copyright 2014, RSAIT research group, University of Basque Country
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// *
// *  * Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// *  * Redistributions in binary form must reproduce the above
// *    copyright notice, this list of conditions and the following
// *    disclaimer in the documentation and/or other materials provided
// *    with the distribution.
// *  * Neither the name of the University of Freiburg nor the names of its
// *    contributors may be used to endorse or promote products derived
// *    from this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *********************************************************************/
//

#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include <naoqi_msgs/JointAnglesWithSpeed.h>
#include <naoqi_msgs/TactileTouch.h>

#include <std_srvs/Empty.h>
#include <naoqi_msgs/BodyPoseAction.h>
#include <naoqi_msgs/BodyPoseActionGoal.h>
#include <actionlib/client/simple_action_client.h>

#include <skeleton_tracker/Skeleton.h>

double killSignal = 0;
class NaoHeadControl{
    public:
        // Publisher
        ros::Publisher jointPub;
        // Subscriber
        ros::Subscriber skelSub;
        ros::Subscriber tactileSub;
        // Node Handle
        ros::NodeHandle nh;
        // Node variables
        double head_x, head_y, head_z, head_yaw, head_pitch;
        bool exited = false;
        naoqi_msgs::JointAnglesWithSpeed jointAngleMsg;
        ros::ServiceClient m_stiffnessDisableClient;
        ros::ServiceClient m_stiffnessEnableClient;
        std_srvs::Empty e;
        // Functions
        void skeletonCallBack(const skeleton_tracker::Skeleton msg);
        void tactileCallBack(naoqi_msgs::TactileTouch msg);
        void publishHeadJointAngles();
        void enableStiffness();
        void disableStiffness();
};

// Get Users's head data from Skeleton topic
void NaoHeadControl::skeletonCallBack(const skeleton_tracker::Skeleton msg){
    // Head
    head_x = msg.head_x;
    head_y = msg.head_y;
    head_z = msg.head_z;
    head_yaw = -msg.head_yaw*3;
    head_pitch = -msg.head_pitch*2;

    //ROS_INFO("Subscribed X:%.2g, Y:%.2g, Z:%.2g, YAW:%.2g, PITCH:%.2g", head_x, head_y, head_z, head_yaw, head_pitch);
}

// Get Users's head data from Skeleton topic
void NaoHeadControl::tactileCallBack(naoqi_msgs::TactileTouch msg){
	int headTouched = msg.state;
	
	if(headTouched == 1){
		exited = true;
  	//actionlib::SimpleActionClient<naoqi_msgs::BodyPoseAction> bodyPoseClient("body_pose", true);
  	//callBodyPoseClient("init");
	}
}

// Publish NAO's head data
void NaoHeadControl::publishHeadJointAngles(){

    // Head
    jointAngleMsg.joint_names.push_back("HeadYaw");
    jointAngleMsg.joint_angles.push_back(head_yaw);

    jointAngleMsg.joint_names.push_back("HeadPitch");
    jointAngleMsg.joint_angles.push_back(head_pitch);

    jointAngleMsg.speed = 0.2;

    jointPub.publish(jointAngleMsg);

    jointAngleMsg.joint_names.clear();
    jointAngleMsg.joint_angles.clear();
}
// Enable stiffness
void NaoHeadControl::enableStiffness(){
    m_stiffnessEnableClient = nh.serviceClient<std_srvs::Empty>("body_stiffness/enable");
    m_stiffnessEnableClient.call(e);
}
// Disable stiffness
void NaoHeadControl::disableStiffness(){
    m_stiffnessDisableClient = nh.serviceClient<std_srvs::Empty>("body_stiffness/disable");
    m_stiffnessDisableClient.call(e);
}
// Shutdown signal
void shutdown(int signal){
    killSignal = -1;
}
// Main function
int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "nao_head_control" );
  NaoHeadControl naoHeadControl;
  // Skeleton topic subscriber
  naoHeadControl.skelSub = naoHeadControl.nh.subscribe("skeleton", 10, &NaoHeadControl::skeletonCallBack, &naoHeadControl);
  // Head tactile subscriber
  naoHeadControl.tactileSub = naoHeadControl.nh.subscribe("nao_robot/contact/tactile_touch", 10, &NaoHeadControl::tactileCallBack, &naoHeadControl);
  // NAO's joint publisher
  naoHeadControl.jointPub = naoHeadControl.nh.advertise<naoqi_msgs::JointAnglesWithSpeed>("joint_angles",10);
  // Rate of publishing head motion commands
	ros::Rate loop_rate(10);
  // Define kill signal
  signal(SIGINT, shutdown);

  naoHeadControl.enableStiffness();
  while( naoHeadControl.nh.ok() && killSignal == 0) // naoHeadControl.nh.ok()
    {	
      ros::spinOnce();
      naoHeadControl.publishHeadJointAngles();
      loop_rate.sleep();
      if(naoHeadControl.exited == true) {
	break;
      }
    }
	
  //naoHeadControl.disableStiffness();
  ROS_INFO("Killing nao_head_control...");
  ros::shutdown();
	return 0;
}
