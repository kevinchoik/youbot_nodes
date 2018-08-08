#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

std::vector<double> getInputDouble() {
	std::string inputStr;
	std::getline(std::cin, inputStr);
	std::istringstream inputSs(inputStr);
	std::vector<double> inputVec;
	for (std::string s; inputSs >> s; ) {
		inputVec.push_back(atof(s.c_str()));
	}
	return inputVec;
}

std::vector<std::string> getInputString() {
	std::string inputStr;
	std::getline(std::cin, inputStr);
	std::istringstream inputSs(inputStr);
	std::vector<std::string> inputVec;
	for (std::string s; inputSs >> s; ) {
		inputVec.push_back(s);
	}
	return inputVec;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_pick_place");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	// Planning group: arm
	static const std::string PLANNING_GROUP = "arm_1";
	// Define planning group
	moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
	// Define planning scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// Define raw pointer for planning group
	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	while (ros::ok()) {
		std::cout << "> ";
		std::vector<std::string> cmdInput = getInputString();
		// Add object
		if (cmdInput[0] == "add") {
			// Define collision object
			moveit_msgs::CollisionObject collision_object;
			collision_object.header.frame_id = move_group.getPlanningFrame();
			// Box ID
			collision_object.id = cmdInput[1];
			// Reading box dimension from console
			std::cout << "Dimension\n> ";
			std::vector<double> inputDim = getInputDouble();
			if (inputDim.size() == 3) {
				shape_msgs::SolidPrimitive primitive;
				primitive.type = primitive.BOX;
				primitive.dimensions.resize(3);
				primitive.dimensions[0] = inputDim[0];
				primitive.dimensions[1] = inputDim[1];
				primitive.dimensions[2] = inputDim[2];
				// Update colliison object
				collision_object.primitives.push_back(primitive);
				// Reading box pose from console
				std::cout << "Pose\n> ";
				std::vector<double> inputPose = getInputDouble();
				if (inputPose.size() == 6 || inputPose.size() == 7) {
					geometry_msgs::Pose box_pose;
					box_pose.position.x = inputPose[0];
					box_pose.position.y = inputPose[1];
					box_pose.position.z = inputPose[2];
					if (inputPose.size() == 6) {
						// RPY to quaternion conversion
						tf::Quaternion quat(1, 0, 0, 0);
						quat.setRPY(inputPose[3] * M_PI / 180, inputPose[4] * M_PI / 180, inputPose[5] * M_PI / 180);
						box_pose.orientation.x = quat.x();
						box_pose.orientation.y = quat.y();
						box_pose.orientation.z = quat.z();
						box_pose.orientation.w = quat.w();
					} else {
						// Inputted quaternion orientation
						box_pose.orientation.x = inputPose[3];
						box_pose.orientation.y = inputPose[4];
						box_pose.orientation.z = inputPose[5];
						box_pose.orientation.w = inputPose[6];
					}
					// Update colliison object
					collision_object.primitive_poses.push_back(box_pose);
					// Add to planning scene
					collision_object.operation = collision_object.ADD;
					std::vector<moveit_msgs::CollisionObject> collision_objects;
					collision_objects.push_back(collision_object);
					ROS_INFO_STREAM_NAMED("my_pick_place_node", "Adding ID: " << collision_object.id);
					planning_scene_interface.addCollisionObjects(collision_objects);
				}
			}
		// Pick up object
		} else if (cmdInput[0] == "pick") {
			// Grasp method
			std::vector<moveit_msgs::Grasp> grasps;
			// RPY to quaternion conversion
			tf::Quaternion quat(1, 0, 0, 0);
			quat.setRPY(-M_PI, 0, 0);
			// Pose of grasping destination
			geometry_msgs::PoseStamped p;
			p.pose.position.x = 0;
			p.pose.position.y = 0.2;
			p.pose.position.z = 0.08;
			p.pose.orientation.x = quat.x();
			p.pose.orientation.y = quat.y();
			p.pose.orientation.z = quat.z();
			p.pose.orientation.w = quat.w();
			moveit_msgs::Grasp g;
			g.grasp_pose = p;
			// Grasping approach
			g.pre_grasp_approach.direction.vector.z = -1.0;
			g.pre_grasp_approach.direction.header.frame_id = "base_link";
			g.pre_grasp_approach.min_distance = 0.04;
			g.pre_grasp_approach.desired_distance = 0.08;
			// Grasping retreat
			g.post_grasp_retreat.direction.vector.z = 1.0;
			g.post_grasp_retreat.direction.header.frame_id = "base_link";
			g.post_grasp_retreat.min_distance = 0.04;
			g.post_grasp_retreat.desired_distance = 0.08;
			// Open gripper before grasping
			g.pre_grasp_posture.joint_names.push_back("gripper_finger_joint_l");
			g.pre_grasp_posture.joint_names.push_back("gripper_finger_joint_r");
			g.pre_grasp_posture.points.resize(1);
			g.pre_grasp_posture.points[0].positions.resize(2);
			g.pre_grasp_posture.points[0].positions[0] = 0.01109;
			g.pre_grasp_posture.points[0].positions[1] = 0.01109;
			// Close gripper after grasping
			g.grasp_posture.joint_names.push_back("gripper_finger_joint_l");
			g.grasp_posture.joint_names.push_back("gripper_finger_joint_r");
			g.grasp_posture.points.resize(1);
			g.grasp_posture.points[0].positions.resize(2);
			g.grasp_posture.points[0].positions[0] = 0.0009;
			g.grasp_posture.points[0].positions[1] = 0.0009;
			// Brute force time
			for (int i = 0; i < 1000; i++){
				p.pose.position.z = -0.5 + i * 0.001;
				g.grasp_pose = p;
				grasps.push_back(g);
			}
			// Push back possible grasps
			//grasps.push_back(g);
			move_group.pick(cmdInput[1], grasps);
		}
	}
}