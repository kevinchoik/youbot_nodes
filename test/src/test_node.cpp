#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // Planning group: arm
  // static const std::string PLANNING_GROUP = "arm_1";
  // Planning group: gripper
  static const std::string PLANNING_GROUP = "arm_1_gripper";
  // Define planning group
  moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
  // Define planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Define raw pointer for planning group
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  while (ros::ok()) {
    /*
    // Planning to an inputted joint-space
    std::vector<double> joint_group_positions;
    // Reading joint-space input from console
    std::cout << "> ";
    double jointPos;
    for (int i = 0; i < 5; i++) {
      std::cin >> jointPos;
      joint_group_positions.push_back(jointPos);
    }
    move_group.setJointValueTarget(joint_group_positions);
    // Calling the planner
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("test_node", "Visualizing pose goal %s", success ? "" : "FAILED");
    move_group.move();
    */
    
    /*
    // Returning current pose (position & orientation)
    std::cout << "> ";
    std::string givePos;
    std::cin >> givePos;
    if (givePos == "y") {
      geometry_msgs::Pose currPose = move_group.getCurrentPose().pose;
      ROS_INFO_STREAM_NAMED("current_pose", "\nPosition (x, y, z): " << currPose.position.x << ", " << currPose.position.y << ", " << currPose.position.z << "\nOrientation (x, y, z, w): " << currPose.orientation.x << ", " << currPose.orientation.y << ", " << currPose.orientation.z << ", " << currPose.orientation.w);
    }
    */

    /*
    // Returning joint-space of target
    std::cout << "> ";
    std::string giveJnt;
    std::cin >> giveJnt;
    if (giveJnt == "y") {
    	std::vector<double> jointState;
    	move_group.getJointValueTarget().copyJointGroupPositions("arm_1", jointState);
      ROS_INFO_STREAM_NAMED("target_joint", "\nJoint 1: " << jointState[0] << "\nJoint 2: " << jointState[1] << "\nJoint 3: " << jointState[2] << "\nJoint 4: " << jointState[3] << "\nJoint 5: " << jointState[4]);
    }
    */


    // Open and close gripper
    std::vector<double> gripper_position;
    // Reading joint-space input from console (Min: 0.0009, Max: 0.0111 - epsilon)
    std::cout << "> ";
    double gripPos;
    std::cin >> gripPos;
    move_group.setJointValueTarget("gripper_finger_joint_l", gripPos);
    move_group.setJointValueTarget("gripper_finger_joint_r", gripPos);
    // Calling the planner
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("test_node", "Visualizing gripper goal %s", success ? "" : "FAILED");
    move_group.move(); 


    /*
    // Return all active joints in the group
    std::cout << "> ";
    std::string button;
    std::cin >> button;
    if (button == "y") {
      std::vector<std::string> test = move_group.getActiveJoints();
      std::stringstream ss;
      for (int i = 0; i < test.size(); i++) {
        ss << test[i] << " ";
      }
      ROS_INFO_STREAM_NAMED("test_node", ss.str());
    }
    */

    /*
    // Convert input into vector of inputted values
    std::cout << "> ";
    std::string test;
    std::getline(std::cin, test);
    std::istringstream test1(test);
    std::vector<double> test2;
    for (std::string s; test1 >> s; ) {
      test2.push_back(atof(s.c_str()));
    }
    std::stringstream test3;
    for (int i = 0; i < test2.size(); i++) {
      test3 << test2[i] << " ";
    }
    ROS_INFO_STREAM_NAMED("test_node", test3.str());
    */
    
    /*
    // Misc
    std::cout << "> ";
    double test;
    std::cin >> test;
    std::cin >> test;
    ROS_INFO_STREAM_NAMED("test_node", "val: " << test);
    */
  }
}
