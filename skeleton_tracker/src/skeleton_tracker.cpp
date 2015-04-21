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


#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include "skeleton_tracker/Skeleton.h"

using std::string;

ros::Publisher skeleton_pub;
skeleton_tracker::Skeleton skelMsg;


xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
		ROS_INFO("***************************************************************************************");
	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

// Yaw pitch RAD
void addPointPositionToSkeleton(string const&child_frame_id, double x, double y, double z, double yaw, double pitch){
    if(child_frame_id == "head"){
    	skelMsg.head_x = x;
    	skelMsg.head_y = y;
    	skelMsg.head_z = z;
    	skelMsg.head_yaw = yaw;
    	skelMsg.head_pitch = pitch;	
		}
		else if(child_frame_id == "neck"){
    	skelMsg.neck_x = x;
    	skelMsg.neck_y = y;
    	skelMsg.neck_z = z;
		}
		else if(child_frame_id == "torso"){
    	skelMsg.torso_x = x;
    	skelMsg.torso_y = y;
    	skelMsg.torso_z = z;
		}
		else if(child_frame_id == "right_shoulder"){
    	skelMsg.left_shoulder_x = x;
    	skelMsg.left_shoulder_y = y;
    	skelMsg.left_shoulder_z = z;
		}
		else if(child_frame_id == "right_elbow"){
    	skelMsg.left_elbow_x = x;
    	skelMsg.left_elbow_y = y;
    	skelMsg.left_elbow_z = z;
		}
		else if(child_frame_id == "right_hand"){
    	skelMsg.left_hand_x = x;
    	skelMsg.left_hand_y = y;
    	skelMsg.left_hand_z = z;
		}
		else if(child_frame_id == "right_hip"){
    	skelMsg.left_hip_x = x;
    	skelMsg.left_hip_y = y;
    	skelMsg.left_hip_z = z;
		}
		else if(child_frame_id == "right_knee"){
    	skelMsg.left_knee_x = x;
    	skelMsg.left_knee_y = y;
    	skelMsg.left_knee_z = z;
		}
		else if(child_frame_id == "right_foot"){
    	skelMsg.left_foot_x = x;
    	skelMsg.left_foot_y = y;
    	skelMsg.left_foot_z = z;
		}
		else if(child_frame_id == "left_shoulder"){
    	skelMsg.right_shoulder_x = x;
    	skelMsg.right_shoulder_y = y;
    	skelMsg.right_shoulder_z = z;
		}
		else if(child_frame_id == "left_elbow"){
    	skelMsg.right_elbow_x = x;
    	skelMsg.right_elbow_y = y;
    	skelMsg.right_elbow_z = z;
		}
		else if(child_frame_id == "left_hand"){
    	skelMsg.right_hand_x = x;
    	skelMsg.right_hand_y = y;
    	skelMsg.right_hand_z = z;
		}
		else if(child_frame_id == "left_hip"){
    	skelMsg.right_hip_x = x;
    	skelMsg.right_hip_y = y;
    	skelMsg.right_hip_z = z;
		}
		else if(child_frame_id == "left_knee"){
    	skelMsg.right_knee_x = x;
    	skelMsg.right_knee_y = y;
    	skelMsg.right_knee_z = z;
		}
		else if(child_frame_id == "left_foot"){
    	skelMsg.right_foot_x = x;
    	skelMsg.right_foot_y = y;
    	skelMsg.right_foot_z = z;
		}
} 

void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
    static tf::TransformBroadcaster br;

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;
    
    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
    					   m[3], m[4], m[5],
    					   m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);
    
    // Added by IGOR

    double roll, pitch,yaw;
    pitch = atan2(2*(qy*qz+qw*qx),qw*qw-qx*qx-qy*qy+qz*qz);
    yaw = asin(-2*(qx*qz-qw*qy));
    roll = atan2(2*(qx*qy+qw*qz),qw*qw+qx*qx-qy*qy-qz*qz);
    /*if(child_frame_id == "left_hand"){
    	ROS_INFO("Quaternions: X:%g, Y:%g, Z:%g, W:%g, Roll:%g, Pitch:%g, Yaw:%g", qx, qy, qz, qw, roll, pitch, yaw);
    	ros::Duration time(0.5);
    	//time.sleep();
    }*/

    /*if(child_frame_id == "left_elbow"){
    	ROS_INFO("Quaternions: X:%g, Y:%g, Z:%g, W:%g, Roll:%g, Pitch:%g, Yaw:%g", qx, qy, qz, qw, roll, pitch, yaw);
    	ros::Duration time(0.5);
    	//time.sleep();
    }*/
        
    addPointPositionToSkeleton(child_frame_id, x, y, z, yaw, pitch);
    
    

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

void publishTransforms(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;


        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
     
        publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

        publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
        publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
        publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

        publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
        publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");

        publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
        publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
        publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

        publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
        publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
        publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
    }
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

int main(int argc, char **argv) {
    ros::init(argc, argv, "skeleton_tracker");
    ros::NodeHandle nh;
    // Added by Igor
    ros::Duration time(2.0);
    time.sleep();
    ROS_INFO("******************************* KINECT CALIBRATION ************************************");
    ROS_INFO("- Do initial calibration pose");

    string configFilename = ros::package::getPath("skeleton_tracker") + "/skeleton_tracker.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		ROS_INFO("Supplied user generator doesn't support skeleton");
		return 1;
	}

    XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(30);

        
        //ros::NodeHandle pnh("~");
	ros::NodeHandle pnh("~");
        string frame_id("openni_depth_frame");
        pnh.getParam("camera_frame_id", frame_id);
        
   //Added by IGOR
   skeleton_pub = nh.advertise<skeleton_tracker::Skeleton>("skeleton", 1000);     
                
	while (ros::ok()) {
	
		g_Context.WaitAndUpdateAll();
		publishTransforms(frame_id);
		
		//ROS_INFO("Lx: %f, Ly: %f, Lz: %f", msg.left_hand_x, msg.left_hand_y, msg.left_hand_z);
		//ROS_INFO("Rx: %f, Ry: %f, Rz: %f", msg.right_hand_x, msg.right_hand_y, msg.right_hand_z);
		skeleton_pub.publish(skelMsg);
    ros::spinOnce();
		
		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}
