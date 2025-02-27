#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include<thread>

#define JOINT_INDICES_DEBUG

using namespace std;
struct motors{
    float position;
    float target_position;
    float velocity;
    float target_velocity;
    float kp;
    float kd;
}motor[7];//7个关节电机

int control_frequency = 50;
vector<string> joints_name={
"elbow_joint", "robotiq_85_left_knuckle_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint","wrist_3_joint"
};


class ur5_actuator {
  public:
    ur5_actuator(ros::NodeHandle& nh,const string& action_name);
    ~ur5_actuator();
    void execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

  private:
    ros::NodeHandle& nh_; // 使用传递进来的 NodeHandle 引用
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_server_;
    control_msgs::FollowJointTrajectoryFeedback feedback_;
    control_msgs::FollowJointTrajectoryResult result_;
    string action_name_;
};

ur5_actuator::ur5_actuator(ros::NodeHandle& nh,const string& action_name)
:   nh_(nh), // 使用传递的 NodeHandle
    action_server_(nh_, action_name, std::bind(&ur5_actuator::execute_callback, this, std::placeholders::_1), false),
    action_name_(action_name)
{
    action_server_.start(); // 启动 Action Server
    ROS_INFO_STREAM(action_name_.c_str() << ": Action server is starting...");

}

ur5_actuator::~ur5_actuator() {
    ROS_INFO_STREAM(action_name_.c_str() << ": ur5_actuator destructor.");
}

// 从 moveit 接收轨迹点，在分发给对应电机
void ur5_actuator::execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) {
    bool success = true;
    ros::Rate rate(control_frequency);
    ROS_INFO("%s: Executing trajectory", action_name_.c_str());
    for (int i = 0; i < goal->trajectory.points.size(); i++) 
    {   
        //每一次执行轨迹点中的一个
        // 确保 moveit 规划出来的轨迹点，其中的关节数量和底层设置的关节数量一致
        if (goal->trajectory.joint_names.size() != joints_name.size()) 
        {
            ROS_ERROR_STREAM(action_name_.c_str()
                             << ": The number of motors and the number of joints do not match. joint: "
                             << goal->trajectory.joint_names.size() << ", motor: " << joints_name.size());
            
            result_.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
            action_server_.setAborted(result_);
            success = false;
            return; // 退出当前的动作执行
        }

        cout<<"共规划了"<<goal->trajectory.points.size()<<"个点,目前正在规划第"<<i<<"个点发送到电机层\n";

        const trajectory_msgs::JointTrajectoryPoint& point = goal->trajectory.points[i];
        for (int j = 0; j < goal->trajectory.joint_names.size(); j++) 
        {
            motor[j].target_position =point.positions[j] ;
            motor[j].target_velocity = point.velocities[j];
            cout<<"电机 "<<j<<" 目标角度 "<<motor[j].target_position<<" 当前角度 "<<motor[j].position<<" "<<goal->trajectory.joint_names[j]<<
            " 目标速度 "<<point.velocities[j]<<"\n";
        }
    
        // 发送反馈
        feedback_.header.stamp = ros::Time::now();
        feedback_.joint_names = goal->trajectory.joint_names;
        feedback_.desired = point;
        action_server_.publishFeedback(feedback_);

        rate.sleep();
    }

    result_.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    action_server_.setSucceeded(result_);
}

void thread_motor_communication_canable1() {
    ros::Rate rate(control_frequency);
    while (ros::ok()) {
        for (int i = 0; i<7; i++) {
            motor[i].position = motor[i].target_position;
            // cout<<" 电机 "<<i<<" 当前位置 "<<motor[i].position<<""<<"\n";
        }
        rate.sleep();
    }
    ROS_INFO_STREAM("canable1 thread is exiting");
}

bool motor_communication_initialization() {
    for(int i = 0; i<7; i++) {
        motor[i].kp = 0.5;
        motor[i].kd = 0.1;
        motor[i].target_position = 0;
        motor[i].target_velocity = 0;
    }
    ROS_INFO_STREAM("motor communication initialization");
    return true;
}

// 该线程函数负责周期性的给 ros 发布机器人各个关机的状态
void thread_joints_state_publisher(ros::NodeHandle& nh) 
{
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(7);
    joint_state.position.resize(7);
    ros::Rate rate(control_frequency);

    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    while (ros::ok()) 
    {
        joint_state.header.stamp = ros::Time::now();
        for (int i = 0; i < 7; i++) 
        {
            joint_state.name[i] = joints_name[i];
            joint_state.position[i] = motor[i].position;
        }
        joint_state_pub.publish(joint_state);
        rate.sleep();
    }
    ROS_INFO_STREAM("ros topic: '/joint_states' exits normally");
}


void set_thread_priority(thread& thread, int priority, int cpu_core) 
{
    pthread_t nativeHandle = thread.native_handle();
    //设置线程优先级
    int policy = SCHED_FIFO;
    struct sched_param param;
    param.sched_priority = priority;
    // 设置 CPU 亲和性
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);
    int result = pthread_setaffinity_np(nativeHandle, sizeof(cpu_set_t), &cpuset);
    if (result != 0) 
    {
        ROS_ERROR_STREAM("Failed to set CPU affinity for thread ID " << pthread_self() << ". Error code: " << result);
    }
    else 
    {
        ROS_INFO_STREAM("Successfully set CPU affinity for thread ID " << pthread_self());
    }
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "ur5_actuator");
    ros::NodeHandle nh;

    // 首先连接电机
    if (!motor_communication_initialization()) {
        return 0;
    };

    // // 使用多线程的方式，维持和电机的通讯
    thread t1(thread_motor_communication_canable1);
    set_thread_priority(t1, 99, 5);
    t1.detach();

    // 启动线程，周期性的向 ros 发布机器人关节的状态
    thread t_ros_pub(bind(thread_joints_state_publisher, ref(nh)));
    set_thread_priority(t_ros_pub, 99, 1);
    t_ros_pub.detach();

    string action_name = "arm_controller/follow_joint_trajectory";
    ur5_actuator execute_trajectory_left_arm(nh, action_name);


    ros::spin();
    return 0;

}