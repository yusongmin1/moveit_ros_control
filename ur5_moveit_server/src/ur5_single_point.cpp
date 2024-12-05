#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "ros/ros.h"
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include "std_msgs/String.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
// #include <Eigen>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace rvt = rviz_visual_tools;


int main(int argc, char** argv) {


    ros::init(argc, argv, "ur5_planing_scene");
    ros::NodeHandle nh;


    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit::planning_interface::MoveGroupInterface arm("arm");

    moveit_visual_tools::MoveItVisualTools visual_tools("bask_link");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 2.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = arm.getPlanningFrame();
    ROS_INFO_STREAM(collision_object.header.frame_id);
    collision_object.id = "box1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 0.5;


    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
    geometry_msgs::PoseStamped msg_left;
    // arm.setStartStateToCurrentState();
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = 0.0;
    joint_group_positions[1] = -1.57;
    joint_group_positions[2] = 0.0;
    joint_group_positions[3] = -1.57;
    joint_group_positions[4] = 0.0;
    joint_group_positions[5] = 0.0;
    arm.setJointValueTarget(joint_group_positions);
    arm.move();

    // msg_left=arm.getCurrentPose("wrist_3_link");
    // std::cout<<msg_left.pose.orientation.x<<std::endl;
    // msg_left.pose.orientation.x += 0.1;

    // arm.setPoseTarget(msg_left,"wrist_3_link");

    // std::cout<<"Planning"<<std::endl;
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // std::cout<<"Planning2"<<std::endl;

    // bool success = (arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // std::cout<<"Planning3"<<std::endl;

    // const moveit::core::JointModelGroup* joint_model_group =
    // arm.getCurrentState()->getJointModelGroup("arm");
    // std::cout<<"Planning4"<<std::endl;

    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // std::cout<<"Planning5"<<std::endl;

    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    return 0;

}