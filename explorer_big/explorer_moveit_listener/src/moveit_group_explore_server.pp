/**
 * \file move_group_explore_server.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief receive grasp pose request, calculate pre-grasp pose, then send requests to move_group_server in the sequence of
 *        pre-grasp pose, grasp pose
 */

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cstdlib>

#include <move_to_target_pose.h>
#include <explore_reach.h>
#include <std_msgs/String.h>

ros::ServiceClient client;

bool explore_reach(zhen_baxter_moveit::explore_reach::Request  &req,
				   zhen_baxter_moveit::explore_reach::Response &res)
{
	clock_t begin = std::clock();
	
	geometry_msgs::Pose reach_pose = req.target_pose;
	ROS_INFO("[explore server] explore reach pose: %f, %f, %f, %f, %f, %f, %f", reach_pose.position.x, reach_pose.position.y, reach_pose.position.z,
			 reach_pose.orientation.x, reach_pose.orientation.y, reach_pose.orientation.z, reach_pose.orientation.w);
	
	// GRASP BLOCK FROM ABOVE 
	// 	- Translation: [0.000, 0.060, 0.116]
	// 	- Rotation: in Quaternion [0.590, -0.399, 0.607, -0.354]
	// 	- Translation: [-0.004, 0.103, 0.104]
	// 	- Rotation: in Quaternion [0.667, -0.268, 0.632, -0.291]
	
	//simple rule to define pre-grasp pose (offset the grasp pose along the approching direction)
	tf::Transform offset;
	offset.setIdentity();
	offset.setOrigin(tf::Vector3(0,0,-0.1));
	
	tf::Transform grasp_pose;
	grasp_pose.setOrigin(tf::Vector3(reach_pose.position.x, reach_pose.position.y, reach_pose.position.z));
	grasp_pose.setRotation(tf::Quaternion(reach_pose.orientation.x,reach_pose.orientation.y,reach_pose.orientation.z,reach_pose.orientation.w));
	
	tf::Transform pre_grasp_pose;
	pre_grasp_pose = grasp_pose*offset;
	
	// 	printf("pre_grasp_pose: %f %f %f %f %f %f %f\n", pre_grasp_pose.getOrigin().x(), pre_grasp_pose.getOrigin().y(), pre_grasp_pose.getOrigin().z(),
	// 			pre_grasp_pose.getRotation().getX(), pre_grasp_pose.getRotation().getY(), pre_grasp_pose.getRotation().getZ(), pre_grasp_pose.getRotation().getW()	);	
	
	geometry_msgs::Pose target_grasp_pose = reach_pose;
	
	geometry_msgs::Quaternion odom_quat;
	tf::quaternionTFToMsg(pre_grasp_pose.getRotation(), odom_quat);
	
	geometry_msgs::Pose target_pregrasp_pose;
	target_pregrasp_pose.orientation = odom_quat;
	target_pregrasp_pose.position.x = pre_grasp_pose.getOrigin().x();  
	target_pregrasp_pose.position.y = pre_grasp_pose.getOrigin().y();
	target_pregrasp_pose.position.z = pre_grasp_pose.getOrigin().z();
	
	ROS_INFO("[explore server] target_pregrasp_pose: %f %f %f %f %f %f %f\n", target_pregrasp_pose.position.x, target_pregrasp_pose.position.y, target_pregrasp_pose.position.z,
			 target_pregrasp_pose.orientation.x, target_pregrasp_pose.orientation.y, target_pregrasp_pose.orientation.z, target_pregrasp_pose.orientation.w	);
	
	ROS_INFO("[explore server] target_grasp_pose: %f %f %f %f %f %f %f\n", target_grasp_pose.position.x, target_grasp_pose.position.y, target_grasp_pose.position.z,
			 target_grasp_pose.orientation.x, target_grasp_pose.orientation.y, target_grasp_pose.orientation.z, target_grasp_pose.orientation.w	);
	
	zhen_baxter_moveit::move_to_target_pose srv;
	std::vector<geometry_msgs::Pose> target_poses;
	std::vector<std::string> reference_frames;
	
	std::string req_reference_frame = req.reference_frame;
	reference_frames.push_back(req_reference_frame);
	reference_frames.push_back(req_reference_frame);
	reference_frames.push_back(req_reference_frame);
	
	target_poses.push_back(target_pregrasp_pose);
	target_poses.push_back(target_grasp_pose);
	target_poses.push_back(target_pregrasp_pose);
	
	srv.request.target_poses = target_poses;
	srv.request.reference_frames = reference_frames; 
	client.call(srv);
	ROS_INFO("explore pose attempt succeed: %d", srv.response.succeed);
	
	if(srv.response.succeed)
		res.succeed = true;
	else
		res.succeed = false;
	
	// 	srv.request.target_pose=target_pregrasp_pose; 
	// 	std_msgs::String reference_frame;
	// 	reference_frame.data = "/block_link0";
	// 	srv.request.reference_frame=reference_frames;
	// 	client.call(srv);
	// 	ROS_INFO("pregrasp pose attempt succeed: %d", srv.response.succeed);
	// 	res.succeed = false;
	// 	if(srv.response.succeed)
	// 	{
	// 		srv.request.target_pose=target_grasp_pose; 
	// 		client.call(srv);
	// 		ROS_INFO("grasp pose attempt succeed: %d", srv.response.succeed);
	// 		if(srv.response.succeed)
	// 		{
	// 			srv.request.target_pose=target_pregrasp_pose; 
	// 			client.call(srv);
	// 			ROS_INFO("retrieve attempt succeed: %d", srv.response.succeed);
	// 			if(srv.response.succeed)
	// 				res.succeed = true;
	// 		}
	// 	}
	
	clock_t end = std::clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	ROS_INFO("[explore server] move group motion planning + motion execution takes time: %f seconds", elapsed_secs);
	
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_explore_server");
	
	ros::NodeHandle n;
	client = n.serviceClient<zhen_baxter_moveit::move_to_target_pose>("move_to_target_pose");
	ros::ServiceServer service = n.advertiseService("explore_reach", explore_reach);
	ros::spin();
	
	return 0;
}
