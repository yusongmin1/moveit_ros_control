//控制机器人移动到多个笛卡尔坐标点，这个轨迹点就是要抓取的姿态的正确点，但是x,y,z需要调整
//轨迹融合成一个轨迹，实际机器人，仿真机器人都可以
//接下来订阅/left_arm_control /right_contol 话题，每次接受到话题，
//改变末端笛卡尔坐标 大小取决于发过来的数值大小
//订阅/show 话题，显示当前的笛卡尔坐标
//订阅/object_positions 左右手xyz坐标 ，机械臂移动到该位置
// rostopic pub /object_positions std_msgs/Float64MultiArray "layout:
//   dim:
//   - label: ''
//     size: 2
//     stride: 2
//   data_offset: 0
// data: [0.45197512 ,0.13693847 ,0.25788548,0.44789194 ,-0.07314274 ,0.2208666]"

#include <ymbot_c2_moveit_server/ymbot_cartesian_keyboard.h>
#include<std_msgs/Float64MultiArray.h> 

using namespace std;
float offset_left_x=0,offset_right_x=0,offset_left_y=0,offset_right_y=0,offset_left_z=0,offset_right_z=0;

void left_arm_callback(const geometry_msgs::Pose& msg)
{
    moveit::planning_interface::MoveGroupInterface move_group_arms("arms");
    move_group_arms.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    geometry_msgs::PoseStamped msg_left;
    msg_left=move_group_arms.getCurrentPose("left_tool_link");
    msg_left.pose.position.x+=msg.position.x;
    msg_left.pose.position.y+=msg.position.y;
    msg_left.pose.position.z+=msg.position.z;
    offset_left_x+=msg.position.x;
    offset_left_y+=msg.position.y;
    offset_left_z+=msg.position.z;
    std::cout<<"控制指令为左臂 "<<"控制数据为 "
    <<msg.position.x<<"\n"
    <<msg.position.y<<"\n"
    <<msg.position.z<<"\n"
    <<"当前已累计为 "
    <<offset_left_x<<" "
    <<offset_left_y<<" "
    <<offset_left_z<<"\n";
    move_group_arms.setPoseTarget(msg_left, "left_tool_link");
    // move_group_arms.setPoseTarget(msg_right, "Right_Arm_Link7");
    success = (move_group_arms.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arms.execute(plan);
}
void right_arm_callback(const geometry_msgs::Pose& msg)
{
    moveit::planning_interface::MoveGroupInterface move_group_arms("arms");
    move_group_arms.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    geometry_msgs::PoseStamped msg_right;
    msg_right=move_group_arms.getCurrentPose("right_tool_link");
    msg_right.pose.position.x+=msg.position.x;
    msg_right.pose.position.y+=msg.position.y;
    msg_right.pose.position.z+=msg.position.z;
    offset_right_x+=msg.position.x;
    offset_right_y+=msg.position.y;
    offset_right_z+=msg.position.z;
    std::cout<<"控制指令为左臂 "<<"控制数据为 "
    <<msg.position.x<<" "
    <<msg.position.y<<" "
    <<msg.position.z<<"\n"
    <<"当前已累计为 "
    <<offset_right_x<<" "
    <<offset_right_y<<" "
    <<offset_right_z<<"\n";
    // move_group_arms.setPoseTarget(msg_left, "Left_Arm_Link7");
    move_group_arms.setPoseTarget(msg_right, "right_tool_link");
    success = (move_group_arms.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arms.execute(plan);
    

}
void multarray_callback(const std_msgs::Float64MultiArray& msg){
    moveit::planning_interface::MoveGroupInterface move_group_arms("arms");
    move_group_arms.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    geometry_msgs::PoseStamped msg_right,msg_left;
    vector<double> msgs=msg.data;
    if(msgs.size()!=6){
        std::cerr<<"输出维度不对，重新输入\n";
        return ;
    }
    vector<geometry_msgs::Pose> target_poses_left_arm, target_poses_right_arm,target_poses_arms;
    vector<moveit::planning_interface::MoveGroupInterface::Plan> plan_arms;

    msg_left=move_group_arms.getCurrentPose("left_tool_link");
    msg_right=move_group_arms.getCurrentPose("right_tool_link");

    msg_left.pose.position.x=msgs[0];
    msg_left.pose.position.y=msgs[1]+0.05;
    msg_left.pose.position.z=msgs[2];
    msg_right.pose.position.x=msgs[3];
    msg_right.pose.position.y=msgs[4]-0.05;
    msg_right.pose.position.z=msgs[5];
    msg_left.pose.orientation.w=0.5;
    msg_left.pose.orientation.x=-0.5;
    msg_left.pose.orientation.y=-0.5;
    msg_left.pose.orientation.z=0.5;
    msg_right.pose.orientation.w=0.5;
    msg_right.pose.orientation.x=-0.5;
    msg_right.pose.orientation.y=-0.5;
    msg_right.pose.orientation.z=0.5;
    target_poses_left_arm.push_back(msg_left.pose);
    target_poses_right_arm.push_back(msg_right.pose);

    msg_left.pose.position.x=msgs[0];
    msg_left.pose.position.y=msgs[1];
    msg_left.pose.position.z=msgs[2];
    msg_right.pose.position.x=msgs[3];
    msg_right.pose.position.y=msgs[4];
    msg_right.pose.position.z=msgs[5];
    target_poses_left_arm.push_back(msg_left.pose);
    target_poses_right_arm.push_back(msg_right.pose);

    msg_left.pose.position.x=msgs[0];
    msg_left.pose.position.y=msgs[1];
    msg_left.pose.position.z=msgs[2]+0.04;
    msg_right.pose.position.x=msgs[3];
    msg_right.pose.position.y=msgs[4];
    msg_right.pose.position.z=msgs[5]+0.04;
    target_poses_left_arm.push_back(msg_left.pose);
    target_poses_right_arm.push_back(msg_right.pose);

    // 开始规划
    for (int i = 0; i < target_poses_right_arm.size(); i++) {
        move_group_arms.setPoseTarget(target_poses_left_arm[i], "left_tool_link");
        move_group_arms.setPoseTarget(target_poses_right_arm[i], "right_tool_link");
        success = (move_group_arms.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        plan_arms.push_back(plan);
        robot_trajectory::RobotTrajectory robot_traj(move_group_arms.getCurrentState()->getRobotModel(),
                                                     "arms");
        robot_traj.setRobotTrajectoryMsg(*move_group_arms.getCurrentState(), plan.trajectory_);

        // 获取第一段轨迹的终点状态
        robot_state::RobotState end_state = robot_traj.getLastWayPoint();

        // 第三步：将第一段轨迹的终点状态设置为下一段轨迹的起点
        move_group_arms.setStartState(end_state);
    }



    // // 合并所有轨迹
    robot_trajectory::RobotTrajectory combined_trajectory = mergeTrajectories(
        move_group_arms.getCurrentState()->getRobotModel(), "arms", plan_arms, move_group_arms);

    // 执行合并后的轨迹
    moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
    combined_trajectory.getRobotTrajectoryMsg(combined_plan.trajectory_);
    move_group_arms.execute(combined_plan);
    sleep(1);
    // move_group_arms.setPoseTarget(msg_left, "left_tool_link");
    // move_group_arms.setPoseTarget(msg_right, "right_tool_link");
    success = (move_group_arms.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arms.execute(plan);
    target_poses_left_arm.clear();
    target_poses_right_arm.clear();

}
void put_callback(const std_msgs::Int32& msg)
{
    moveit::planning_interface::MoveGroupInterface move_group_arms("arms");
    move_group_arms.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    geometry_msgs::PoseStamped msg_right,msg_left;
    vector<geometry_msgs::Pose> target_poses_left_arm, target_poses_right_arm,target_poses_arms;
    vector<moveit::planning_interface::MoveGroupInterface::Plan> plan_arms;

    msg_left=move_group_arms.getCurrentPose("left_tool_link");
    msg_right=move_group_arms.getCurrentPose("right_tool_link");

    msg_left.pose.position.z-=0.02;
    msg_right.pose.position.z-=0.02;
    target_poses_left_arm.push_back(msg_left.pose);
    target_poses_right_arm.push_back(msg_right.pose);

    msg_left.pose.position.y+=0.05;
    msg_right.pose.position.y-=0.05;
    target_poses_left_arm.push_back(msg_left.pose);
    target_poses_right_arm.push_back(msg_right.pose);

    // msg_left.pose.position.x=msgs[0];
    // msg_left.pose.position.y=msgs[1];
    // msg_left.pose.position.z=msgs[2]+0.04;
    // msg_right.pose.position.x=msgs[3];
    // msg_right.pose.position.y=msgs[4];
    // msg_right.pose.position.z=msgs[5]+0.04;
    // target_poses_left_arm.push_back(msg_left.pose);
    // target_poses_right_arm.push_back(msg_right.pose);

    // 开始规划
    for (int i = 0; i < target_poses_right_arm.size(); i++) {
        move_group_arms.setPoseTarget(target_poses_left_arm[i], "left_tool_link");
        move_group_arms.setPoseTarget(target_poses_right_arm[i], "right_tool_link");
        success = (move_group_arms.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        plan_arms.push_back(plan);
        robot_trajectory::RobotTrajectory robot_traj(move_group_arms.getCurrentState()->getRobotModel(),
                                                     "arms");
        robot_traj.setRobotTrajectoryMsg(*move_group_arms.getCurrentState(), plan.trajectory_);

        // 获取第一段轨迹的终点状态
        robot_state::RobotState end_state = robot_traj.getLastWayPoint();

        // 第三步：将第一段轨迹的终点状态设置为下一段轨迹的起点
        move_group_arms.setStartState(end_state);
    }



    // // 合并所有轨迹
    robot_trajectory::RobotTrajectory combined_trajectory = mergeTrajectories(
        move_group_arms.getCurrentState()->getRobotModel(), "arms", plan_arms, move_group_arms);

    // 执行合并后的轨迹
    moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
    combined_trajectory.getRobotTrajectoryMsg(combined_plan.trajectory_);
    move_group_arms.execute(combined_plan);
    sleep(1);
    // move_group_arms.setPoseTarget(msg_left, "left_tool_link");
    // move_group_arms.setPoseTarget(msg_right, "right_tool_link");
    success = (move_group_arms.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group_arms.execute(plan);
    target_poses_left_arm.clear();
    target_poses_right_arm.clear();
}
//显示当前笛卡尔坐标 1 left_arm 2 right_arm else arms
void show_callback(const std_msgs::Int32& msg)
{
    moveit::planning_interface::MoveGroupInterface move_group_arms("arms");
    move_group_arms.setStartStateToCurrentState();
    std_msgs::String left_,right_;
    left_.data="left_arm";
    right_.data="right_arm";
    if(msg.data==1)//左臂
    {
        geometry_msgs::PoseStamped msg_left;
        msg_left=move_group_arms.getCurrentPose("left_tool_link");
        ROS_INFO_STREAM("left_tool_link"<<msg_left.header.frame_id<<"\n");
        show(msg_left.pose,left_);

    }
    else if (msg.data==2)//右臂
    {
        geometry_msgs::PoseStamped msg_right;
        msg_right=move_group_arms.getCurrentPose("left_tool_link");
        show(msg_right.pose,right_);
    }
    else//双臂
    {
        geometry_msgs::PoseStamped msg_left,msg_right;
        msg_left=move_group_arms.getCurrentPose("left_tool_link");
        msg_right=move_group_arms.getCurrentPose("right_tool_link");
        show(msg_left.pose,left_);
        show(msg_right.pose,right_);
    }
    
}

int main(int argc, char** argv) {
    /*********************************Humanoid Body ROS Initialization*******************************/
    ros::init(argc, argv, "ymbot_hand_eye");
    ros::NodeHandle nh;


    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit::planning_interface::MoveGroupInterface move_group_left_arm("left_arm");
    moveit::planning_interface::MoveGroupInterface move_group_right_arm("right_arm");
    moveit::planning_interface::MoveGroupInterface move_group_arms("arms");
    geometry_msgs::Pose target_pose_left_arm, target_pose_right_arm,target_pose_arms;


    move_group_left_arm.setMaxVelocityScalingFactor(0.9);
    move_group_left_arm.setMaxAccelerationScalingFactor(0.9);
    move_group_right_arm.setMaxVelocityScalingFactor(0.9);

    move_group_right_arm.setMaxAccelerationScalingFactor(0.9);
    move_group_arms.setMaxVelocityScalingFactor(0.9);
    move_group_arms.setMaxAccelerationScalingFactor(0.9);

    //打印这个组的参考系的名称
    ROS_INFO("Reference frame::move_group_arms: %s", move_group_left_arm.getPlanningFrame().c_str());
    //打印这个组的末端执行器的名称
    ROS_INFO("Reference frame::move_group_arms: %s", move_group_left_arm.getEndEffectorLink().c_str());
    ros::Subscriber sub_left  = nh.subscribe("/left_arm_control",   1, left_arm_callback);
    ros::Subscriber sub_right = nh.subscribe("/right_arm_control",  1, right_arm_callback);
    ros::Subscriber sub_show  = nh.subscribe("/show",               1, show_callback);
    ros::Subscriber multarray = nh.subscribe("/object_positions",   1, multarray_callback);
    ros::Subscriber put = nh.subscribe("/put",   1, put_callback);
    ros::AsyncSpinner spinner(6);  // 使用2个线程
    spinner.start(); 
    //伸直到前面
    move_group_arms.setStartStateToCurrentState();
    move_group_arms.setNamedTarget("front");
    move_group_arms.move();
    sleep(1);

    cout<<"rostopic pub /left_arm_contol    控制左臂偏移"<<"\n"
        <<"rostopic pub /right_arm_contol   控制右臂偏移"<<"\n"
        <<"rostopic pub /arms_contol        控制双臂偏移"<<"\n"
        <<"rostopic pub /object_positions        控制双臂位置"<<"\n"
        
        <<"注意!!!!!!!!输出的位置偏移不应过大"
        <<"rostopic pub /show 输出笛卡尔坐标 1:左臂 2:右臂 其他:双臂"<<"\n";

    while (ros::ok)
    {
        ros::Duration(1).sleep();
    }
    
    //spinner.stop();  // 停止spinner

    return 0;
}


