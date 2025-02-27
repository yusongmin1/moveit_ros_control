#include <ur5_ros_control/robot_hardware_interface_sim.h>


// 

int fd=0;
void configure_serial_port(int fd) {
    struct termios tty;
    
    // 获取当前串口设置
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr" << std::endl;
        return;
    }
    
    // 设置输入输出波特率为9600
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
 
    // 禁用校验位
    tty.c_cflag &= ~PARENB;
    // 设置一个停止位
    tty.c_cflag &= ~CSTOPB;
    // 设置每个字节8位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    // 禁用硬件流控制
    tty.c_cflag &= ~CRTSCTS;
    // 开启接收使能和本地模式（忽略调制解调器）
    tty.c_cflag |= CREAD | CLOCAL;
 
    // 将终端设置为原始模式
    cfmakeraw(&tty);
    // 刷新输入缓冲区
    tcflush(fd, TCIFLUSH);
    // 设置新的串口参数
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr" << std::endl;
    }
}

UR5HW::UR5HW(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=50;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &UR5HW::update, this);
}

UR5HW::~UR5HW() {
}

void UR5HW::init() {
    
    
    joint_name_={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    joint_position_.resize(joint_num,0.0);
    joint_velocity_.resize(joint_num,0.0);
    joint_effort_.resize(joint_num,0.0);
    joint_position_command_.resize(joint_num,0.0);
        // Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    for(int i=0;i<6;i++)
    {
        // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        position_joint_interface_.registerHandle(jointPositionHandle);
    }

}

void UR5HW::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void UR5HW::read() {
    char buf[100];
    int n_read = cnm::read__(fd, buf, sizeof(buf));
    if (n_read >=0) {
        std::cout << "Read: " << std::string(buf, n_read) << std::endl; // 打印读取到的消息
    }
    auto str=std::string(buf, n_read);
    joint_position_[0]=(float)std::stof(str);

}

void UR5HW::write(ros::Duration elapsed_time) {
    for(int i=1;i<5;i++)
    joint_position_[i]=joint_position_command_[i];
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "single_joint_hardware_interface");
    ros::NodeHandle nh;
    const char *portname = "/dev/ttyUSB0"; // 串口设备文件的名称
    fd = open(portname, O_RDWR);
    if (fd < 0) {
        std::cerr << "Error opening " << portname << std::endl;
        // return 1;
    }
 
    configure_serial_port(fd); // 配置串口参数
    ros::MultiThreadedSpinner spinner(2); 
    UR5HW robot(nh);
    // char buf[100];
    // auto cnmb = cnm::read__(fd, buf, sizeof(buf));
    // if (cnmb >=0) {
    //     std::cout << "Read: " << std::string(buf, cnmb) << std::endl; // 打印读取到的消息
    // }
    // auto str=std::string(buf, cnmb);
    // // joint_position_[0]=(float)std::stof(str);
    spinner.spin();
    return 0;
}
