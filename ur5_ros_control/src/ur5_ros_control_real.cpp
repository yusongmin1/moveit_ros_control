// #include <ur5_ros_control/robot_hardware_interface.h>


// UR5HW::UR5HW(ros::NodeHandle& nh) : nh_(nh) {
//     init();
//     controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
//     loop_hz_=5;
//     ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
//     non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
// }

// UR5HW::~UR5HW() {
// }

// void UR5HW::init() 
// {
//     joint_name_={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
//     joint_position_.resize(joint_num,0.0);
//     joint_velocity_.resize(joint_num,0.0);
//     joint_effort_.resize(joint_num,0.0);
//     joint_position_command_.resize(joint_num,0.0);
//     for(int i=0;i<6;i++)
//     {
//         //std::string JointStateHandle_name="jointStateHandle"+std::to_string(i);
        
//         // Create joint state interface
//         hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
//         joint_state_interface_.registerHandle(jointStateHandle);

//         // Create position joint interface
//         hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
//         position_joint_interface_.registerHandle(jointPositionHandle);
//     }
//     // Register all joints interfaces    
//     registerInterface(&joint_state_interface_);
//     registerInterface(&position_joint_interface_);

// }

// void UR5HW::update(const ros::TimerEvent& e) {
//     elapsed_time_ = ros::Duration(e.current_real - e.last_real);
//     read();
//     controller_manager_->update(ros::Time::now(), elapsed_time_);
//     write(elapsed_time_);
// }

// void UR5HW::read() {

// }

// void UR5HW::write(ros::Duration elapsed_time) {
	
// 	ROS_INFO("PWM Cmd: %.2f",joint_position_command_);
//     joint_position_= joint_position_command_;
		
// }



// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "UR5_HARDWARE_INTERFACE");
//     ros::NodeHandle nh;
//     //ros::AsyncSpinner spinner(4);  
//     ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
//     UR5HW ROBOT(nh);

//     spinner.spin();
//     //ros::spin();
//     return 0;
// }
