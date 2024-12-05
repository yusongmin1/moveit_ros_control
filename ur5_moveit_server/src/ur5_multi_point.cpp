//控制机器人移动到多个笛卡尔坐标点，轨迹融合成一个轨迹，实际机器人，仿真机器人都可以


#include <atomic>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include <sensor_msgs/Joy.h>

#include "std_msgs/String.h"
#include <string.h>

#include <iostream>
#include <vector>

#include <std_msgs/Int32.h>
#include<sensor_msgs/Joy.h>


using namespace std;



int main(int argc, char** argv) {
    /*********************************Humanoid Body ROS Initialization*******************************/
    ros::init(argc, argv, "ur5_robot_State_Learning");
    ros::NodeHandle nh;
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    
    std::cout<<kinematic_model->getName()<<std::endl;
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    // kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    for(auto i: joint_names){
        ROS_INFO_STREAM("Joint name: "<<i);
    }
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    /* Set one joint in the Panda arm outside its joint limit */
    joint_values[0] = 1.57;
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    /* Check whether any joint is outside its joint limits */
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    /* Enforce the joint limits for this state and check again*/
    kinematic_state->enforceBounds();
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    // kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("wrist_3_link");

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    double timeout = 0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    if (found_ik)
    {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("wrist_3_link");

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    }
    else
    {
    ROS_INFO("Did not find IK solution");
    }
    return 0;
}