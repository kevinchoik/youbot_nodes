#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_youbot_ik");
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
    // Planning to an inputted pose
    geometry_msgs::Pose target_pose;
    // Reading pose input from console
    std::cout << "> ";
    std::string inputStr;
    std::getline(std::cin, inputStr);
    std::istringstream inputSs(inputStr);
    std::vector<double> inputVec;
    for (std::string s; inputSs >> s; ) {
      inputVec.push_back(atof(s.c_str()));
    }
    if (inputVec.size() == 6 || inputVec.size() == 7) {
    	// Inputted position
    	target_pose.position.x = inputVec[0];
    	target_pose.position.y = inputVec[1];
    	target_pose.position.z = inputVec[2];
    	if (inputVec.size() == 6) {
    		// Inputted RPY orientation
	    	double oriR = inputVec[3];
	    	double oriP = inputVec[4];
	    	double oriY = inputVec[5];
	    	// RPY to quaternion conversion
		    tf::Quaternion quat(1, 0, 0, 0);
		    quat.setRPY(oriR * M_PI / 180, oriP * M_PI / 180, oriY * M_PI / 180);
		    target_pose.orientation.x = quat.x();
		    target_pose.orientation.y = quat.y();
		    target_pose.orientation.z = quat.z();
		    target_pose.orientation.w = quat.w();
    	} else {
    		// Inputted quaternion orientation
    		target_pose.orientation.x = inputVec[3];
    		target_pose.orientation.y = inputVec[4];
    		target_pose.orientation.z = inputVec[5];
    		target_pose.orientation.w = inputVec[6];
    	}
	    move_group.setJointValueTarget(target_pose);
	    // Calling the planner
	    moveit::planning_interface::MoveGroup::Plan my_plan;
	    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	    ROS_INFO_NAMED("my_youbot_ik_node", "Visualizing pose goal %s", success ? "" : "FAILED");
	    move_group.move();
    }
  }
}
