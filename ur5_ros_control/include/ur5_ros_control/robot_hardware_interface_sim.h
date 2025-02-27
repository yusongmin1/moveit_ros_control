
#include <memory>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring> // for memcpy

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <angles/angles.h>

using namespace std;

#define joint_num 6
class UR5HW : public hardware_interface::RobotHW 
{
	public:
        UR5HW(ros::NodeHandle& nh);
        UR5HW();
        ~UR5HW();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
   
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        hardware_interface::EffortJointInterface     effort_joint_interface_;

        
        joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
        
        std::vector<string> joint_name_;  
        vector<double> joint_position_;
        vector<double> joint_velocity_;
        vector<double> joint_effort_;
        vector<double> joint_position_command_;
        vector<double> joint_effort_command_;
        vector<double> joint_velocity_command_;
        
        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

};

