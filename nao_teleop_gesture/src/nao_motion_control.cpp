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

#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <skeleton_tracker/Skeleton.h>
#include <naoqi_msgs/BodyPoseAction.h>
#include <naoqi_msgs/BodyPoseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <naoqi_msgs/TactileTouch.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

#include <iostream>

// Global definitions
// Subscriber
ros::Subscriber skelSub;
ros::Subscriber headTouchSub;
ros::Subscriber sonarLeftSub, sonarRightSub;
ros::Subscriber fallSub;
ros::Subscriber teleopStatusSub;
// Publisher
ros::Publisher velPub;
ros::Publisher walkPub;
ros::Publisher ttsPub;
// Messages and Services
geometry_msgs::Twist cmd;
ros::ServiceClient stiffnessDisableClient;
ros::ServiceClient stiffnessEnableClient;
naoqi_msgs::BodyPoseGoal goal;
ros::Duration bodyPoseTimeOut(5.0);
// Launch file parameters
double sonar_status;
// Skeleton Info
float head_x, head_y, head_z;
float neck_x, neck_y, neck_z;
float torso_x, torso_y, torso_z;
float left_shoulder_x, left_shoulder_y, left_shoulder_z;
float left_elbow_x, left_elbow_y, left_elbow_z;
float left_hand_x, left_hand_y, left_hand_z;
float left_hip_x, left_hip_y, left_hip_z;
float left_knee_x, left_knee_y, left_knee_z;
float left_foot_x, left_foot_y, left_foot_z;
float right_shoulder_x, right_shoulder_y, right_shoulder_z;
float right_elbow_x, right_elbow_y, right_elbow_z;
float right_hand_x, right_hand_y, right_hand_z;
float right_hip_x, right_hip_y, right_hip_z;
float right_knee_x, right_knee_y, right_knee_z;
float right_foot_x, right_foot_y, right_foot_z;
// Tactile info
int headTouched; // 1 pressed, 0 released
// Sonar info
double leftSonar;
double rightSonar;

// Teleoperation marks
float standUp_mark = -0.3;
float forward_mark = 2.15;
float backward_mark = 2.45;
float turnLeft_mark = -0.3;
float turnRight_mark = 0.2;
// Teleoperation variables
bool isStandUp = false;
bool isTeleopStarted = false;
int countStiffness = 0;
int countStand = 0;
int countCrouch = 0;
bool teleopStarted = false;
bool exited = false;
double obstacleMinDist = 0.35;
bool obstacleFront = false;
int countObstacle = 0;
double forwardVel = 0.3;
double backwardVel = -forwardVel;
double turnVel = 0.2;
double lateralVel = 0.3;


// Order poses to NAO
bool callBodyPoseClient(const std::string& poseName){

  actionlib::SimpleActionClient<naoqi_msgs::BodyPoseAction> bodyPoseClient("body_pose", true);
	
  std_srvs::Empty e;
  stiffnessEnableClient.call(e);
 	
  if(poseName == "init"){
    goal.pose_name="init";
  }
  else if(poseName == "crouch"){
    goal.pose_name="crouch";
  }
  bodyPoseClient.sendGoal(goal);
  //bodyPoseClient.sendGoalAndWait(goal, ros::Duration(5.0));
	
}

// NAO teleoperation control
void nao_gesture_control(){
	
  actionlib::SimpleActionClient<naoqi_msgs::BodyPoseAction> bodyPoseClient("body_pose", true);
  // Teleop system starts when the operator stands up
  if(!isStandUp && left_hip_y > standUp_mark && right_hip_y > standUp_mark){
		
    callBodyPoseClient("init"); //init
    countStand++;
    // Send standup pose command more than one, because sometimes it does not react at first time
    // Teleoperation starts after robot stands up
    if(countStand == 5){
      isStandUp = true;
      countStand = 0;

      std_msgs::String text_msg;
      text_msg.data = "Active pour contrôler les mouvements de la";
      ttsPub.publish(text_msg);

      ROS_INFO("J'ai leve");
      countStiffness=0;
      teleopStarted = true;
    }	
  }
  // Teleop system finishes when the operator takes crouch pose
  else if(isStandUp && left_hip_y < standUp_mark && right_hip_y < standUp_mark){
		
    callBodyPoseClient("crouch");
    countCrouch++;
    // Send crouch pose command more than one, because sometimes it does not react at first time
    if(countCrouch == 5){
      ROS_INFO("Est-ce");
      isStandUp = false;
      countCrouch = 0;
    }
  }
  // Eserita dagoela, esertzeko posearekin jarraituz gero stiffness desaktibatzen da
  else if(teleopStarted && !isStandUp && left_hip_y < standUp_mark && right_hip_y < standUp_mark){
    countStiffness++;
    ROS_INFO("countStiffness: %d", countStiffness);
    //if(countStiffness > 30){
    if (countStiffness == 21){				
      std_msgs::String text_msg;
      text_msg.data = "Le contrôle des mouvements est desactive";
      ttsPub.publish(text_msg);
    }
    //}	
  }
  // When NAO is stand up
  else if(isStandUp){
    if(sonar_status == 0){
      // Move forward
      if(left_foot_z < forward_mark && right_foot_z < forward_mark){
	obstacleFront = false;
	countObstacle = 0;
			
	cmd.linear.x = forwardVel;
	cmd.linear.y = 0.0;
	// Move forward and turn left
	if(left_shoulder_x < turnLeft_mark){
	  cmd.angular.z = turnVel;
	}
	// Move forward and turn right
	else if(right_shoulder_x > turnRight_mark){
	  cmd.angular.z = -turnVel;
	}
	else{
	  cmd.angular.z = 0.0;
	}
      }
      // Move backward
      else if(left_foot_z > backward_mark && right_foot_z > backward_mark){
	cmd.linear.x = backwardVel;
	cmd.linear.y = 0.0;
	// Move backward and turn left
	if(left_shoulder_x < turnLeft_mark){
	  cmd.angular.z = turnVel;
	}
	// Move backward and turn right
	else if(right_shoulder_x > turnRight_mark){
	  cmd.angular.z = -turnVel;
	}
	// Move backward without turn 
	else{
	  cmd.angular.z = 0.0;
	}
      }
      // Move left - lateral displacement
      else if(left_foot_x < -0.3){
	cmd.linear.x = 0.0;
	cmd.linear.y = lateralVel;
      }
      // Move right - lateral displacement
      else if(right_foot_x > 0.3){
	cmd.linear.x = 0.0;
	cmd.linear.y = -lateralVel;
      }
      // Stop movements in the X and Y axes
      else{
	cmd.linear.y = 0.0;
	cmd.linear.x = 0.0;
	// Turn left
	if(left_shoulder_x < turnLeft_mark){
	  cmd.angular.z = turnVel;
	}
	// Turn right
	else if(right_shoulder_x > turnRight_mark){
	  cmd.angular.z = -turnVel;
	}
	// NAO is completely stopped
	else{
	  cmd.angular.z = 0.0;
	}
      }
    }
    // if(sonar_status == 1){
    else {
      if(leftSonar >= obstacleMinDist && rightSonar >= obstacleMinDist){
	// Move forward
	if(left_foot_z < forward_mark && right_foot_z < forward_mark){
	  obstacleFront = false;
	  countObstacle = 0;
			
	  cmd.linear.x = forwardVel;
	  cmd.linear.y = 0.0;
	  // Move forward and turn left
	  if(left_shoulder_x < turnLeft_mark){
	    cmd.angular.z = turnVel;
	  }
	  // Move forward and turn right
	  else if(right_shoulder_x > turnRight_mark){
	    cmd.angular.z = -turnVel;
	  }
	  else{
	    cmd.angular.z = 0.0;
	  }
	}
	// Move backward
	else if(left_foot_z > backward_mark && right_foot_z > backward_mark){
	  cmd.linear.x = backwardVel;
	  cmd.linear.y = 0.0;
	  // Move backward and turn left
	  if(left_shoulder_x < turnLeft_mark){
	    cmd.angular.z = turnVel;
	  }
	  // Move backward and turn right
	  else if(right_shoulder_x > turnRight_mark){
	    cmd.angular.z = -turnVel;
	  }
	  // Move backward without turn 
	  else{
	    cmd.angular.z = 0.0;
	  }
	}
	// Move left - lateral displacement
	else if(left_foot_x < -0.3){
	  cmd.linear.x = 0.0;
	  cmd.linear.y = lateralVel;
	}
	// Move right - lateral displacement
	else if(right_foot_x > 0.3){
	  cmd.linear.x = 0.0;
	  cmd.linear.y = -lateralVel;
	}
	// Stop movements in the X and Y axes
	else{
	  cmd.linear.y = 0.0;
	  cmd.linear.x = 0.0;
	  // Turn left
	  if(left_shoulder_x < turnLeft_mark){
	    cmd.angular.z = turnVel;
	  }
	  // Turn right
	  else if(right_shoulder_x > turnRight_mark){
	    cmd.angular.z = -turnVel;
	  }
	  // NAO is completely stopped
	  else{
	    cmd.angular.z = 0.0;
	  }
	}
      }
      else{
	obstacleFront = true;
	countObstacle++;

	// cmd.linear.y = 0.0;
	// cmd.linear.x = 0.0;
	// velPub.publish(cmd);
	// ros::spinOnce();

	if(obstacleFront && countObstacle == 10){
	  std_msgs::String text_msg;
	  text_msg.data = "Je ne peux pas proceder";
	  ttsPub.publish(text_msg);
	}
	// Move backward
	if(left_foot_z > backward_mark && right_foot_z > backward_mark){
	  cmd.linear.x = backwardVel;
	  cmd.linear.y = 0.0;
	  // Move backward and turn left
	  if(left_shoulder_x < turnLeft_mark){
	    cmd.angular.z = turnVel;
	  }
	  // Move backward and turn right
	  else if(right_shoulder_x > turnRight_mark){
	    cmd.angular.z = -turnVel;
	  }
	  // Move backward without turn
	  else{
	    cmd.angular.z = 0.0;
	  }
	}
	// Stop movements in the X and Y axes
	else{
	  cmd.linear.y = 0.0;
	  cmd.linear.x = 0.0;
	  // Turn left
	  if(left_shoulder_x < turnLeft_mark){
	    cmd.angular.z = turnVel;
	  }
	  // Turn right
	  else if(right_shoulder_x > turnRight_mark){
	    cmd.angular.z = -turnVel;
	  }
	  // NAO is completely stopped
	  else{
	    cmd.angular.z = 0.0;
	  }
	}
      }
    }
  }
  return;
}

void skeletonCallBack(const skeleton_tracker::Skeleton msg)
{
  // Hand
  left_hand_x = msg.left_hand_x;
  left_hand_y = msg.left_hand_y;
  right_hand_x = msg.right_hand_x;
  right_hand_y = msg.right_hand_y;
  
  // Foot
  left_foot_x = msg.left_foot_x;
  left_foot_y = msg.left_foot_y;
  left_foot_z = msg.left_foot_z;
  right_foot_x = msg.right_foot_x;
  right_foot_y = msg.right_foot_y;
  right_foot_z = msg.right_foot_z;
  
  // Hip
  left_hip_x = msg.left_hip_x;
  left_hip_y = msg.left_hip_y;
  right_hip_x = msg.right_hip_x;
  right_hip_y = msg.right_hip_y;
  
  // Shoulder
  left_shoulder_x = msg.left_shoulder_x;
  left_shoulder_y = msg.left_shoulder_y;
  left_shoulder_z = msg.left_shoulder_z;
  right_shoulder_x = msg.right_shoulder_x;
  right_shoulder_y = msg.right_shoulder_y;
  right_shoulder_z = msg.right_shoulder_z;
  
  /*ROS_INFO("LeftHand_X: %f, LeftHand_Y: %f", left_hand_x, left_hand_y);*/
  //ROS_INFO("LeftFoot_X: %f, LeftFoot_Y: %f, LeftFoot_Z: %f", left_foot_x, left_foot_y, left_foot_z);
  /*ROS_INFO("LeftHip_X: %f, LeftHip_Y: %f", left_hip_x, left_hip_y);
    ROS_INFO("LeftShoulder_X: %f, LeftShoulder_Y: %f, LeftShoulder_z: %f", left_shoulder_x, left_shoulder_y, left_shoulder_z);
    ROS_INFO("RightHand_X: %f, RightHand_Y: %f", right_hand_x, right_hand_y);*/
  //ROS_INFO("RightFoot_X: %f, RightFoot_Y: %f, RightFoot_Z: %f", right_foot_x, right_foot_y,right_foot_z);
  /*ROS_INFO("RightHip_X: %f, RightHip_Y: %f", right_hip_x, right_hip_y);
    ROS_INFO("RightShoulder_X: %f, RightShoulder_Y: %f, RightShoulder_Z: %f", right_shoulder_x, right_shoulder_y, right_shoulder_z);*/
  //ROS_ERROR("Head_X: %f, Head_Y: %f, Head_z: %f", head_x, head_y, head_z);
}

// Callback from tactile touch sensors
void tactileCallBack(naoqi_msgs::TactileTouch msg){
  headTouched = msg.state;
  ROS_INFO("Head: %d", headTouched);
  if(countStiffness > 20 && headTouched == 1){
    std_srvs::Empty e;
    
    actionlib::SimpleActionClient<naoqi_msgs::BodyPoseAction> bodyPoseClient("body_pose", true);
    callBodyPoseClient("crouch");

    sleep(5);
    stiffnessDisableClient.call(e);
    
    exited = true;
  }
}

// Callback from sonar sensors
void sonarLeftCallBack(sensor_msgs::Range msg){
  leftSonar = msg.range;
  //ROS_INFO("Left: %g",leftSonar);
}
void sonarRightCallBack(sensor_msgs::Range msg){
  rightSonar = msg.range; 
  //ROS_INFO("Right: %g",rightSonar);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "nao_motion_control");
  ros::NodeHandle n;
 	
  // Get parameters from launch file
  double walk_vel, lat_vel, rot_vel;
  n.getParam("sonar_status", sonar_status);
  ROS_INFO("Sonar status: %.1f", sonar_status);
  n.getParam("walk_vel", walk_vel); 
  n.getParam("lat_vel", lat_vel);
  n.getParam("rot_vel", rot_vel); 
  forwardVel = walk_vel;
  backwardVel = -forwardVel;
  turnVel = rot_vel;
  lateralVel = lat_vel;
	
  // Subscribers
  skelSub = n.subscribe("skeleton", 10, skeletonCallBack);
  headTouchSub = n.subscribe("nao_robot/contact/tactile_touch", 1, tactileCallBack);
  sonarLeftSub = n.subscribe("nao_robot/sonar/left/sonar", 1, sonarLeftCallBack);
  sonarRightSub = n.subscribe("nao_robot/sonar/right/sonar", 1, sonarRightCallBack);
  // Publishers
  velPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  walkPub = n.advertise<std_msgs::Int8>("walking", 10);
  ttsPub = n.advertise<std_msgs::String>("text_speech", 10);

  ros::Rate pubRate(10.0);
  // NAO stiffness service
  stiffnessDisableClient = n.serviceClient<std_srvs::Empty>("body_stiffness/disable");
  stiffnessEnableClient = n.serviceClient<std_srvs::Empty>("body_stiffness/enable");
  
  //ros::Duration time(10);
  //time.sleep();

  while (ros::ok)
    {
      if(left_hand_x != 0 && left_hand_y != 0 && right_hand_x != 0 && right_hand_y != 0){
	// Erabiltzailea ohartarazteko mezuak
	if(isTeleopStarted == false){
	  ROS_INFO("NAO teleoperation Kinect started");
	  isTeleopStarted = true;
	}
	nao_gesture_control();
	velPub.publish(cmd);
      }
      ros::spinOnce();
      pubRate.sleep();
      if(exited == true){
	break;
      }		
    }
  ROS_INFO("Killing nao_motion_control...");
  ros::shutdown();
  return 0;
}
