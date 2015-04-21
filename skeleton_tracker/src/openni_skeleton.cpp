/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright 2014, RSAIT research group, University of Basque Country
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of the University of Freiburg nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <ctime>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv/cxcore.h"  
#include "opencv/cvwimage.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

#include <skeleton_tracker/Skeleton.h>

#define SCAN_RES 512
#define PI 3.141592654

using namespace cv;
using namespace std;

//aldagai orokorrak
geometry_msgs::Twist cmd;
ros::Publisher vel_pub;
ros::Publisher skelImgPub;

cv_bridge::CvImagePtr cv_ptr;

float head_x, head_y, head_z;
float neck_x, neck_y, neck_z;
float torso_x, torso_y, torso_z;
float left_hand_x, left_hand_y, left_hand_z;
float right_hand_x, right_hand_y, right_hand_z;
float left_elbow_x, left_elbow_y, left_elbow_z;
float right_elbow_x, right_elbow_y, right_elbow_z;
float left_shoulder_x, left_shoulder_y, left_shoulder_z;
float right_shoulder_x, right_shoulder_y, right_shoulder_z;
float left_hip_x, left_hip_y, left_hip_z;
float left_knee_x, left_knee_y, left_knee_z;
float left_foot_x, left_foot_y, left_foot_z;
float right_hip_x, right_hip_y, right_hip_z;
float right_knee_x, right_knee_y, right_knee_z;
float right_foot_x, right_foot_y, right_foot_z;

int calibrationState = 0; // 0: Waiting for calibration pose, 1: Calibration completed, -1: User lost.


void imageCallBack(const sensor_msgs::ImageConstPtr &msg){
	
  Mat skeletonImg;
  int img_height, img_width;
  float px_head_Y, px_head_X;
  float px_neck_Y, px_neck_X; 
  float px_torso_Y, px_torso_X;  
  float px_left_hand_Y, px_left_hand_X, px_righ_hand_Y, px_righ_hand_X;
  float px_left_elbow_Y, px_left_elbow_X, px_right_elbow_Y, px_right_elbow_X;
  float px_left_shoulder_Y, px_left_shoulder_X, px_right_shoulder_Y, px_right_shoulder_X;
  float px_left_hip_Y, px_left_hip_X, px_right_hip_Y, px_right_hip_X;
  float px_left_knee_Y, px_left_knee_X, px_right_knee_Y, px_right_knee_X;
  float px_left_foot_Y, px_left_foot_X, px_right_foot_Y, px_right_foot_X;
	
  // Convert Kinect image into opencv image
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding.c_str());
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	
  // Create an empty image to draw skeleton
  img_height = cv_ptr->image.size().height;
  img_width = cv_ptr->image.size().width;
  skeletonImg = Mat(img_height,img_width,CV_8UC3, Scalar(0,0,255));
	
	
  if (head_z == 0.0 && neck_z == 0.0 && torso_z == 0.0 && left_shoulder_z == 0.0 && left_elbow_z == 0.0 && left_hand_z == 0.0 
      && right_shoulder_z == 0.0 && right_elbow_z == 0.0 && right_hand_z == 0.0 && left_hip_z == 0.0 && left_knee_z == 0.0 && left_foot_z == 0.0 
      && right_hip_z == 0.0 && right_knee_z == 0.0 && right_foot_z == 0.0)
    {
      if (calibrationState == 0){
	putText(cv_ptr->image, "Waiting for Calibration Pose...", Point(100,180), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255), 2, 8, false);
      }
      else if(calibrationState == 1){
	putText(cv_ptr->image, "User Lost. Do Calibration Pose again...", Point(100,180), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255), 2, 8, false);
      }
    }
  else{
    if (calibrationState == 0){
      putText(cv_ptr->image, "Calibration Completed!", Point(140,180), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0), 2, 8, false);
      calibrationState = 1;
      //imshow("Kinect Image", cv_ptr->image);
      //waitKey(2000);
    }
	
    /*if (calibrationState == 0){
      putText(cv_ptr->image, "Calibration Completed!", Point(140,240), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0), 2, 8, false);
      waitKey(1);
      calibrationState = 1;
      }
      else{
      putText(cv_ptr->image, "", Point(180,240), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0), 2, 8, false);
      }*/
    float meanZ = (left_hand_z + right_hand_z + left_elbow_z + right_elbow_z + left_shoulder_z + right_shoulder_z) / 6;
    float kX = 2.34;
    float kY = 2.34;

		
    //ROS_INFO("img_width: %d x img_height: %d", img_width, img_height);
	
    /*imgScribble = Mat(img_height,img_width,CV_8UC3, Scalar(0,0,255));
      scribbleImage(cv_ptr->image, imgScribble);*/

    //ROS_INFO("Mean Z value: %f", meanZ);
    //meanZ = 1;

    // Head
    px_head_Y = 260 - head_y * 320/1.5 * kY/meanZ;
    px_head_X = 320 - head_x * 320/1.5 * kX/meanZ;
    // Neck
    px_neck_Y = 260 - neck_y * 320/1.5 * kY/meanZ;
    px_neck_X = 320 - neck_x * 320/1.5 * kX/meanZ;
    // Torso
    px_torso_Y = 260 - torso_y * 320/1.5 * kY/meanZ;
    px_torso_X = 320 - torso_x * 320/1.5 * kX/meanZ;
    // Hand	
    px_left_hand_Y = 260 - left_hand_y * 320/1.5 * kY/meanZ;
    px_left_hand_X = 320 - left_hand_x * 320/1.5 * kX/meanZ;
    px_righ_hand_Y = 260 - right_hand_y * 320/1.5 * kY/meanZ;
    px_righ_hand_X = 320 - right_hand_x * 320/1.5 * kX/meanZ;
    // Elbow
    px_left_elbow_Y = 260 - left_elbow_y * 320/1.5 * kY/meanZ;
    px_left_elbow_X = 320 - left_elbow_x * 320/1.5 * kX/meanZ;
    px_right_elbow_Y = 260 - right_elbow_y * 320/1.5 * kY/meanZ;
    px_right_elbow_X = 320 - right_elbow_x * 320/1.5 * kX/meanZ;
    // Shoulder
    px_left_shoulder_Y = 260 - left_shoulder_y * 320/1.5 * kY/meanZ;
    px_left_shoulder_X = 320 - left_shoulder_x * 320/1.5 * kX/meanZ;
    px_right_shoulder_Y = 260 - right_shoulder_y * 320/1.5 * kY/meanZ;
    px_right_shoulder_X = 320 - right_shoulder_x * 320/1.5 * kX/meanZ;
    // Hip
    px_left_hip_Y = 260 - left_hip_y * 320/1.5 * kY/meanZ;
    px_left_hip_X = 320 - left_hip_x * 320/1.5 * kX/meanZ;
    px_right_hip_Y = 260 - right_hip_y * 320/1.5 * kY/meanZ;
    px_right_hip_X = 320 - right_hip_x * 320/1.5 * kX/meanZ;
    // Knee
    px_left_knee_Y = 260 - left_knee_y * 320/1.5 * kY/meanZ;
    px_left_knee_X = 320 - left_knee_x * 320/1.5 * kX/meanZ;
    px_right_knee_Y = 260 - right_knee_y * 320/1.5 * kY/meanZ;
    px_right_knee_X = 320 - right_knee_x * 320/1.5 * kX/meanZ;
    // Foot
    px_left_foot_Y = 260 - left_foot_y * 320/1.5 * kY/meanZ;
    px_left_foot_X = 320 - left_foot_x * 320/1.5 * kX/meanZ;
    px_right_foot_Y = 260 - right_foot_y * 320/1.5 * kY/meanZ;
    px_right_foot_X = 320 - right_foot_x * 320/1.5 * kX/meanZ;

    //ROS_INFO("left_hand_y: %f x left_y: %f", left_hand_y, px_left_hand_Y);

    // Head
    circle(cv_ptr->image, Point(px_head_X, px_head_Y), 20, Scalar( 0, 0, 255 ),2,8);
    // Neck
    circle(cv_ptr->image, Point(px_neck_X, px_neck_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    //Torso
    circle(cv_ptr->image, Point(px_torso_X, px_torso_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    // Hand
    circle(cv_ptr->image, Point(px_left_hand_X, px_left_hand_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    circle(cv_ptr->image, Point(px_righ_hand_X, px_righ_hand_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    // Elbow
    circle(cv_ptr->image, Point(px_left_elbow_X, px_left_elbow_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    circle(cv_ptr->image, Point(px_right_elbow_X, px_right_elbow_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    // Shoulder
    circle(cv_ptr->image, Point(px_left_shoulder_X, px_left_shoulder_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    circle(cv_ptr->image, Point(px_right_shoulder_X, px_right_shoulder_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    // Hip
    circle(cv_ptr->image, Point(px_left_hip_X, px_left_hip_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    circle(cv_ptr->image, Point(px_right_hip_X, px_right_hip_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    // Knee
    circle(cv_ptr->image, Point(px_left_knee_X, px_left_knee_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    circle(cv_ptr->image, Point(px_right_knee_X, px_right_knee_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    // Foot
    circle(cv_ptr->image, Point(px_left_foot_X, px_left_foot_Y), 4, Scalar( 0, 0, 255 ),-1,8);
    circle(cv_ptr->image, Point(px_right_foot_X, px_right_foot_Y), 4, Scalar( 0, 0, 255 ),-1,8);


    // Draw Skeleton lines
    line(cv_ptr->image, Point(px_torso_X, px_torso_Y), Point(px_neck_X, px_neck_Y), Scalar(0,255,0), 2, 8);
    line(cv_ptr->image, Point(px_left_hand_X, px_left_hand_Y), Point(px_left_elbow_X, px_left_elbow_Y), Scalar(0,255,0), 2, 8);
    line(cv_ptr->image, Point(px_righ_hand_X, px_righ_hand_Y), Point(px_right_elbow_X, px_right_elbow_Y), Scalar(0,255,0), 2, 8);
    line(cv_ptr->image, Point(px_left_elbow_X, px_left_elbow_Y), Point(px_left_shoulder_X, px_left_shoulder_Y), Scalar(0,255,0), 2, 8);
    line(cv_ptr->image, Point(px_right_elbow_X, px_right_elbow_Y), Point(px_right_shoulder_X, px_right_shoulder_Y), Scalar(0,255,0), 2, 8);
    line(cv_ptr->image, Point(px_left_shoulder_X, px_left_shoulder_Y), Point(px_right_shoulder_X, px_right_shoulder_Y), Scalar(0,255,0), 2, 8);
    line(cv_ptr->image, Point(px_left_hip_X, px_left_hip_Y), Point(px_torso_X, px_torso_Y), Scalar(0,255,0), 2, 8);
    line(cv_ptr->image, Point(px_right_hip_X, px_right_hip_Y), Point(px_torso_X, px_torso_Y), Scalar(0,255,0), 2, 8);
    line(cv_ptr->image, Point(px_left_hip_X, px_left_hip_Y), Point(px_right_hip_X, px_right_hip_Y), Scalar(0,255,0), 2, 8);
    line(cv_ptr->image, Point(px_left_hip_X, px_left_hip_Y), Point(px_left_knee_X, px_left_knee_Y), Scalar(0,255,0), 2, 8);
    line(cv_ptr->image, Point(px_right_hip_X, px_right_hip_Y), Point(px_right_knee_X, px_right_knee_Y), Scalar(0,255,0), 2, 8);
    line(cv_ptr->image, Point(px_left_knee_X, px_left_knee_Y), Point(px_left_foot_X, px_left_foot_Y), Scalar(0,255,0), 2, 8);
    line(cv_ptr->image, Point(px_right_knee_X, px_right_knee_Y), Point(px_right_foot_X, px_right_foot_Y), Scalar(0,255,0), 2, 8);
	
  }
  skelImgPub.publish(cv_ptr);
  //imshow("Kinect Image", cv_ptr->image);
  //waitKey(1);
}

void skeletonCallBack(const skeleton_tracker::Skeleton msg)
{
  // Head
  head_x = msg.head_x;
  head_y = msg.head_y;
  head_z = msg.head_z;
  // Neck
  neck_x = msg.neck_x;
  neck_y = msg.neck_y;
  neck_z = msg.neck_z;
  // Torso
  torso_x = msg.torso_x;
  torso_y = msg.torso_y;
  torso_z = msg.torso_z;
  // Hands
  left_hand_x = msg.left_hand_x;
  left_hand_y = msg.left_hand_y;
  left_hand_z = msg.left_hand_z;
  right_hand_x = msg.right_hand_x;
  right_hand_y = msg.right_hand_y;
  right_hand_z = msg.right_hand_z;  
  // Elbows
  left_elbow_x = msg.left_elbow_x;
  left_elbow_y = msg.left_elbow_y;
  left_elbow_z = msg.left_elbow_z;
  right_elbow_x = msg.right_elbow_x;
  right_elbow_y = msg.right_elbow_y;
  right_elbow_z = msg.right_elbow_z; 
  // Shoulder
  left_shoulder_x = msg.left_shoulder_x;
  left_shoulder_y = msg.left_shoulder_y;
  left_shoulder_z = msg.left_shoulder_z;
  right_shoulder_x = msg.right_shoulder_x;
  right_shoulder_y = msg.right_shoulder_y;
  right_shoulder_z = msg.right_shoulder_z; 
  // Foot
  left_foot_x = msg.left_foot_x;
  left_foot_y = msg.left_foot_y;
  left_foot_z = msg.left_foot_z;
  right_foot_x = msg.right_foot_x;
  right_foot_y = msg.right_foot_y;
  right_foot_z = msg.right_foot_z;
  // Knee
  left_knee_x = msg.left_knee_x;
  left_knee_y = msg.left_knee_y;
  left_knee_z = msg.left_knee_z;
  right_knee_x = msg.right_knee_x;
  right_knee_y = msg.right_knee_y;
  right_knee_z = msg.right_knee_z;
  // Hip
  left_hip_x = msg.left_hip_x;
  left_hip_y = msg.left_hip_y;
  right_hip_x = msg.right_hip_x;
  right_hip_y = msg.right_hip_y;
  
  //ROS_INFO("LeftHand_X: %f, RightHand_x: %f", left_hand_x, right_hand_x);
  //ROS_INFO("LeftHand_Y: %f, RightHand_Y: %f", left_hand_y, right_hand_y);
  //ROS_INFO("RightHand_X: %f, RightHand_Y: %f", right_hand_x, right_hand_y);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "openni_skeleton");

  ros::NodeHandle n;
  ros::Subscriber imageSub;
  ros::Subscriber skelSub;
  
  imageSub = n.subscribe("/camera/rgb/image_color", 10, imageCallBack);
  skelSub = n.subscribe("skeleton", 10, skeletonCallBack);
  skelImgPub = n.advertise<sensor_msgs::Image>("skeleton_image", 1000);
  
  
  //namedWindow("Kinect Image", CV_WINDOW_AUTOSIZE );
  
  ros::spin();
  //destroyWindow("Kinect Image");

  return 0;
}
