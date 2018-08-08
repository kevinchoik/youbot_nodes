#include "ros/ros.h"
#include "moveit_msgs/MoveGroupActionGoal.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hello_world");
  ros::NodeHandle n;
  ros::Publisher goalPublisher = n.advertise<moveit_msgs::MoveGroupActionGoal>("move_group/goal", 1);
  moveit_msgs::MoveGroupActionGoal actGl;
  // header of MoveGroupActionGoal
  actGl.header.stamp = ros::Time::now();
  // goal_id of MoveGroupActionGoal
  actGl.goal_id.stamp = ros::Time::now();
  // goal of MoveGroupActionGoal
  moveit_msgs::MoveGroupGoal gl;
  // request of MoveGroupGoal
  moveit_msgs::MotionPlanRequest rqst;
  // workspace_parameters of MotionPlanRequest
  moveit_msgs::WorkspaceParameters param;
  param.header.stamp = ros::Time::now();
  param.header.frame_id = "/base_link";
  param.min_corner.x = -1.0;
  param.min_corner.y = -1.0;
  param.min_corner.z = -1.0;
  param.max_corner.x = 1.0;
  param.max_corner.y = 1.0;
  param.max_corner.z = 1.0;
  rqst.workspace_parameters = param;
  // goal_constraints of MotionPlanRequest
  moveit_msgs::Constraints glCnst;
  // joint_constraints of Constraints
  moveit_msgs::JointConstraint jntCnst;
  jntCnst.tolerance_above = 0.0001;
  jntCnst.tolerance_below = 0.0001;
  jntCnst.weight = 1.0;
  for (int i = 0; i < 5; i++) {
    std::stringstream ss;
    ss << "arm_joint_" << (i + 1);
    jntCnst.joint_name = ss.str();
    glCnst.joint_constraints.push_back(jntCnst);
  }
  rqst.goal_constraints.push_back(glCnst);
  rqst.group_name = "arm_1";
  rqst.num_planning_attempts = 10;
  rqst.allowed_planning_time = 5.0;
  rqst.max_velocity_scaling_factor = 1.0;
  rqst.max_acceleration_scaling_factor = 1.0;
  gl.request = rqst;
  // planning_options of MoveGroupGoal
  gl.planning_options.planning_scene_diff.is_diff = true;
  gl.planning_options.replan_delay = 2.0;
  actGl.goal = gl;
  // Values for candle position
  float candlePos[5] = {2.95, 1.13, -2.55, 1.79, 2.92};
  for (int i = 0; i < 5; i++) {
    actGl.goal.request.goal_constraints[0].joint_constraints[i].position = candlePos[i];
  }
  // Rest for 1 second??
  ros::Duration(1).sleep();
  // Publish candle position
  goalPublisher.publish(actGl);
  // Rest for 5 seconds
  ros::Duration(5).sleep();
  // update timestamp
  actGl.header.stamp = ros::Time::now();
  actGl.goal_id.stamp = ros::Time::now();
  actGl.goal.request.workspace_parameters.header.stamp = ros::Time::now();
  // Values for folded position
  float foldedPos[5] = {0.02, 0.02, -0.02, 0.02, 0.02};
  for (int i = 0; i < 5; i++) {
    actGl.goal.request.goal_constraints[0].joint_constraints[i].position = foldedPos[i];
  }
  // Publish folded position
  goalPublisher.publish(actGl);
  // Rest for 5 seconds
  ros::Duration(5).sleep();
  ros::shutdown();
  return 0;
}


  
