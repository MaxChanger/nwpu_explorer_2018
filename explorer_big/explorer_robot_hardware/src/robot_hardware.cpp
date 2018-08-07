
//author : rescuer liao
//date : 2015-4-20

#include "robot_hardware.h"
#include <boost/make_shared.hpp>
#include <std_msgs/Float64.h>
#include <algorithm>
#include <sstream>
ExplorerHardware::ExplorerHardware(ros::NodeHandle node)
    : nh_(node),
      nh_private("~"),
      frep_(30.0),
      last_back_vice_wheel_angular_(0.0),
      last_front_vice_wheel_angular_(0.0),
      last_left_wheel_vel_(0.0),
      last_right_wheel_vel_(0.0) {
    robot_drive_pub_ = nh_.advertise<explorer_msgs::explorer_message>("explorer_driver", 10);

    nh_.setCallbackQueue(&callback_quequ_);
    ::std::stringstream sub_name_back;
    sub_name_back << "explorer_serial_data/" << down_vice_id;
    down_vice_wheel_pos_sub = nh_.subscribe(sub_name_back.str(), 10, &ExplorerHardware::getBackViceWheelPos, this);

    ::std::stringstream sub_name_front;
    sub_name_front << "explorer_serial_data/" << up_vice_id;
    up_vice_wheel_pos_sub = nh_.subscribe(sub_name_front.str(), 10, &ExplorerHardware::getFrontViceWheelPos, this);
    // nh_.setCallbackQueue(&callback_quequ_);
    
    //base wheel joint name
    if (!this->getNameList(this->nh_, "/explorer_drive_controller/left_wheel", base_joint_name_) ||
        !this->getNameList(this->nh_, "/explorer_drive_controller/right_wheel",  base_joint_name_)) {
        ROS_ERROR("get vice wheel name fail");
    } else {
        for (auto name : base_joint_name_) {
            ROS_ERROR_STREAM(name);
        }
    }

    nh_private.param("vice_wheel_param/left_up_vice_wheel_init", left_up_vice_wheel_init, 150.0);
    nh_private.param("vice_wheel_param/left_down_vice_wheel_init", left_down_vice_wheel_init, 150.0);
    nh_private.param("vice_wheel_param/right_up_vice_wheel_init", right_up_vice_wheel_init, 150.0);
    nh_private.param("vice_wheel_param/right_down_vice_wheel_init", right_down_vice_wheel_init, 150.0);
    ROS_ERROR_STREAM("lu" << left_up_vice_wheel_init << "\tld" << left_down_vice_wheel_init<<"\tru"<<right_up_vice_wheel_init<<"\t"<<right_down_vice_wheel_init);

    std::vector<std::string> front_vice_whell_name_, back_vice_whell_name_;

    //vice wheel joint name
    if (!this->getNameList(this->nh_, "/explorer_vice_wheel_controller/front_vice_wheel", vice_wheel_joint_name_) ||
        !this->getNameList(this->nh_, "/explorer_vice_wheel_controller/back_vice_wheel",  vice_wheel_joint_name_)) {
        ROS_ERROR("get vice wheel name fail");
    } else {
        // get front/back whell name
        this->getNameList(this->nh_, "/explorer_vice_wheel_controller/front_vice_wheel", front_vice_whell_name_);
        this->getNameList(this->nh_, "/explorer_vice_wheel_controller/back_vice_wheel",  back_vice_whell_name_);
    }

    // arm joint name
    if (!this->getNameList(this->nh_, "/explorer_arm_controller/joints", arm_joint_name_)) {
        ROS_ERROR("get arm joints name fail");
    }

    // the initialize position of the robot arm
    for (auto it : arm_joint_name_) {
        if (!this->nh_.getParam("/explorer_arm_controller/reset/" + it, arm_joint_off_pos_[it])) {
            ROS_ERROR_STREAM("get reset position fail: " + ("/explorer_arm_controller/reset/" + it));
            break;
        }
    }

    //the initialize position of the robot vice wheel
    for (auto it : front_vice_whell_name_) {
        if (!this->nh_.getParam("/explorer_vice_wheel_controller/front_vice_whell_position", vice_wheel_off_pos_[it])) {
            ROS_ERROR_STREAM("get vice_whell_position fail: " + ("/explorer_vice_wheel_controller/front_vice_whell_position as " + it + "`s value"));
            break;
        }
    }

    for (auto it : back_vice_whell_name_) {
        if (!this->nh_.getParam("/explorer_vice_wheel_controller/back_vice_whell_position", vice_wheel_off_pos_[it])) {
            ROS_ERROR_STREAM("get vice_whell_position fail: " + ("/explorer_vice_wheel_controller/back_vice_whell_position as " + it + "`s value"));
            break;
        }
    }

    for (auto name : arm_joint_name_) {
        arm_joint_pos_[name] = 0.0;
        this->nh_.getParam("/explorer_arm_controller/reset/" + name, arm_joint_pos_[name]);
        arm_joint_vel_[name] = 0.0;
        arm_joint_eff_[name] = 0.0;
        arm_joint_cmd_[name] = 0.0;
        joint_pose_[name] = 0.0;

        joint_state_sub_[name] = nh_.subscribe("/" + name + "/state", 10, &ExplorerHardware::jointStateCallBack, this);//将从joint_state接受到的消息放到joint_pose

        // nh_.setCallbackQueue(&callback_quequ_);

        explorer_interface::PosionReaderHandle arm_pos_reader(name, &arm_joint_pos_[name]);
        posion_reader_interface.registerHandle(arm_pos_reader);

        hardware_interface::JointStateHandle arm_joint_state(
            name,
            &arm_joint_pos_[name],
            &arm_joint_vel_[name],
            &arm_joint_eff_[name]);
        joint_state_interface_.registerHandle(arm_joint_state);

        hardware_interface::JointHandle arm_joint_pose(
            joint_state_interface_.getHandle(name),
            &arm_joint_cmd_[name]);
        joint_position_interface_.registerHandle(arm_joint_pose);
    }

    for (auto name : base_joint_name_) {
        base_wheel_eff_[name] = 0.0;
        base_wheel_pos_[name] = 0.0;
        base_wheel_vel_[name] = 0.0;
        base_wheel_cmd_[name] = 0.0;
        joint_pose_[name] = 0.0;

        explorer_interface::PosionReaderHandle baseJointPos(name, &base_wheel_pos_[name]);
        posion_reader_interface.registerHandle(baseJointPos);

        hardware_interface::JointStateHandle base_wheel_interface_(
            name,
            &base_wheel_pos_[name],
            &base_wheel_vel_[name],
            &base_wheel_eff_[name]);
        joint_state_interface_.registerHandle(base_wheel_interface_);

        hardware_interface::JointHandle base_wheel_pose_(
            joint_state_interface_.getHandle(name),
            &base_wheel_cmd_[name]);
        joint_vel_interface_.registerHandle(base_wheel_pose_);
    }

    for (auto name : vice_wheel_joint_name_) {
        vice_wheel_eff_[name] = 0.0;
        vice_wheel_pos_[name] = 0.0;
        vice_wheel_vel_[name] = 0.0;
        vice_wheel_cmd_[name] = 0.0;
        joint_pose_[name] = 0.0;

        joint_state_sub_[name] = nh_.subscribe("/" + name + "/state", 10, &ExplorerHardware::jointStateCallBack, this);
        // nh_.setCallbackQueue(&callback_quequ_);

        explorer_interface::PosionReaderHandle viceJointPos(name, &vice_wheel_pos_[name]);
        posion_reader_interface.registerHandle(viceJointPos);
        hardware_interface::JointStateHandle vice_wheel_interface_(
            name,
            &vice_wheel_pos_[name],
            &vice_wheel_vel_[name],
            &vice_wheel_eff_[name]);
        joint_state_interface_.registerHandle(vice_wheel_interface_);

        hardware_interface::JointHandle vice_wheel_pose_(
            joint_state_interface_.getHandle(name),
            &vice_wheel_cmd_[name]);
        joint_position_interface_.registerHandle(vice_wheel_pose_);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&joint_position_interface_);
    registerInterface(&joint_vel_interface_);
    registerInterface(&posion_reader_interface);

    spinner_.reset(new ros::AsyncSpinner(1, &callback_quequ_));
    spinner_->start();
    ROS_ERROR("2333");
    //for the sync callback quequ
}

ExplorerHardware::~ExplorerHardware() {
}

void ExplorerHardware::getFrontViceWheelPos(const ::explorer_msgs::explorer_low_level_data::ConstPtr &msg) {
    vice_wheel_pos_["right_up_fin_base_joint"] = -(msg->can_serial_data_1 - right_up_vice_wheel_init) / 360.0 * (acos(0) * 2) * 2;
    vice_wheel_pos_["left_up_fin_base_joint"] = (msg->can_serial_data_2 - left_up_vice_wheel_init) / 360.0 * (acos(0) * 2) * 2;
    // ROS_ERROR_STREAM("right_up_fin_base_joint is\t" << vice_wheel_pos_["right_up_fin_base_joint"] << "\tand left_up_fin_base_joint is \t" << vice_wheel_pos_["left_up_fin_base_joint"]);
}

void ExplorerHardware::getBackViceWheelPos(const ::explorer_msgs::explorer_low_level_data::ConstPtr &msg) {
    vice_wheel_pos_["right_down_fin_base_joint"] = -(msg->can_serial_data_1 - right_down_vice_wheel_init) / 360.0 * (acos(0) * 2) * 2;
    vice_wheel_pos_["left_down_fin_base_joint"] = (msg->can_serial_data_2 - left_down_vice_wheel_init) / 360.0 * (acos(0) * 2) * 2;
    // ROS_ERROR_STREAM("right_down_fin_base_joint is\t" << vice_wheel_pos_["right_down_fin_base_joint"] <<"\t and  "<<msg->can_serial_data_2 - left_down_vice_wheel_init <<"\tand left_down_fin_base_joint is \t" << vice_wheel_pos_["left_down_fin_base_joint"]);
}

bool ExplorerHardware::start() {
    for (auto name : vice_wheel_joint_name_) {
        vice_wheel_pos_[name] = vice_wheel_off_pos_[name];
        vice_wheel_vel_[name] = 0.0;
        vice_wheel_eff_[name] = 0.0;
    }

    return true;
}

void ExplorerHardware::stop() {
    spinner_->stop();
}

void ExplorerHardware::write(ros::Time current_time, ros::Duration period) {
    callback_quequ_.callAvailable();
    bool base_move = false;
    float left_wheel_vel  = std::max(fabs(base_wheel_cmd_["left_up_wheel_base_joint"]),
                                     fabs(base_wheel_cmd_["left_down_wheel_base_joint"]));
    float right_wheel_vel = std::max(fabs(base_wheel_cmd_["right_up_wheel_base_joint"]),
                                     fabs(base_wheel_cmd_["right_down_wheel_base_joint"]));

    float front_vice_wheel_angular = std::max(fabs(vice_wheel_cmd_["left_up_fin_base_joint"]),
                                              fabs(vice_wheel_cmd_["right_up_fin_base_joint"]));
    float back_vice_wheel_angular  = std::max(fabs(vice_wheel_cmd_["left_down_fin_base_joint"]),
                                              fabs(vice_wheel_cmd_["right_down_fin_base_joint"]));

    ROS_INFO_STREAM(" VEL = " << left_wheel_vel << " vel2 =  " << right_wheel_vel << " angular3 = " << front_vice_wheel_angular << " angular 4 = " << back_vice_wheel_angular);

    //for our robot , you can not control the arm , wheel when you are controling
    //the wheels and the vice wheels
    //需要改;;
    if (
        (left_wheel_vel || right_wheel_vel) || (last_left_wheel_vel_ && !left_wheel_vel) || (last_right_wheel_vel_ && !right_wheel_vel)) {
        pub_vel_cmd();
        base_move = true;
    }

    if ((front_vice_wheel_angular || (last_front_vice_wheel_angular_ && !front_vice_wheel_angular)) || (back_vice_wheel_angular || (last_back_vice_wheel_angular_ && !back_vice_wheel_angular))) {
        pub_vice_wheel_cmd();
        base_move = true;
    }

    if (!base_move) {
        //no command to control the robot base ,
        //so arm command can send to the robot
        pub_arm_joint_cmd();
    }

    last_left_wheel_vel_ = left_wheel_vel;
    last_right_wheel_vel_ = right_wheel_vel;
    last_front_vice_wheel_angular_ = front_vice_wheel_angular;
    last_back_vice_wheel_angular_ = back_vice_wheel_angular;
}

void ExplorerHardware::read(ros::Time current_time, ros::Duration period) {
    //get current robot joint state
    callback_quequ_.callAvailable();

    /*
    // 以下这段代码是用于将controller传回的数据进行处理
    // 已经改到发送的同时处理了
    for (auto name : base_joint_name_) {
        base_wheel_vel_[name] = base_wheel_cmd_[name];
    }

    for (auto name : arm_joint_name_) {
        //ROS_INFO_STREAM("arm command :" << name << " " << arm_joint_cmd_[name]);
        //arm_joint_pos_[name] = joint_pose_[name];//这句话不知道有什么用
        arm_joint_pos_[name] = arm_joint_cmd_[name] + arm_joint_eff_[name];
        arm_joint_vel_[name] = 0.0;
        arm_joint_eff_[name] = 0.0;
    }

    for (auto name : vice_wheel_joint_name_) {
        vice_wheel_pos_[name] += vice_wheel_cmd_[name] / 20;
        vice_wheel_vel_[name] = 0.0;
        vice_wheel_eff_[name] = 0.0;
    }
    */
}

ros::Duration ExplorerHardware::getPeriod() {
    return ros::Duration(1 / frep_);
}

ros::Time ExplorerHardware::getCurrentTime() {
    return ros::Time::now();
}

ros::CallbackQueue *ExplorerHardware::getCallBackQuequ() {
    return &callback_quequ_;
}
/*
 * 以下为主履带速度数据
 * 各个履带速度分别发送,以满足小explorer和大explorer的要求
 */
void ExplorerHardware::pub_vel_cmd() {
    for (auto name : base_joint_name_) {
        base_wheel_vel_[name] = base_wheel_cmd_[name];
    }

    explorer_msgs::explorer_message move_message;
    move_message.low_level_id = 1;
    move_message.high_level_id = 1;
    move_message.data.push_back(base_wheel_cmd_["left_up_wheel_base_joint"]);
    move_message.data.push_back(base_wheel_cmd_["left_down_wheel_base_joint"]);
    //        ROS_INFO("pub move order") ;
    robot_drive_pub_.publish(move_message);
    move_message.low_level_id = 2;
    move_message.high_level_id = 2;
    move_message.data.clear();
    move_message.data.push_back(base_wheel_cmd_["right_up_wheel_base_joint"]);
    move_message.data.push_back(base_wheel_cmd_["right_down_wheel_base_joint"]);
    robot_drive_pub_.publish(move_message);
}
/*
 * 以下为副履带的底层数据传输函数
 * 为使得两者相对应 强制除以参宿进行设置
 */
void ExplorerHardware::pub_vice_wheel_cmd() {
    for (auto name : vice_wheel_joint_name_) {
        vice_wheel_pos_[name] += vice_wheel_cmd_[name] / 20;
        vice_wheel_vel_[name] = 0.0;
        vice_wheel_eff_[name] = 0.0;
    }

    explorer_msgs::explorer_message vice_wheel_message;
    vice_wheel_message.low_level_id = 3;
    vice_wheel_message.high_level_id = 3;
    vice_wheel_message.data.push_back(vice_wheel_cmd_["left_up_fin_base_joint"]/6);
    vice_wheel_message.data.push_back(-vice_wheel_cmd_["left_down_fin_base_joint"]/7);

    //    ROS_INFO("pub front vice order") ;
    robot_drive_pub_.publish(vice_wheel_message);

    vice_wheel_message.low_level_id = 4;
    vice_wheel_message.high_level_id = 4;
    vice_wheel_message.data.at(0) = vice_wheel_cmd_["right_up_fin_base_joint"]/6 ;
    vice_wheel_message.data.at(1) = -vice_wheel_cmd_["right_down_fin_base_joint"]/7;

    robot_drive_pub_.publish(vice_wheel_message);

}

float ExplorerHardware::getFrep() {
    return frep_;
}
/*
 * 以下为机械臂的底层数据传输函数
 * 注意! 将机械臂在ros中的移动与显示中的数据整合在一起,以保证计算机"认为"的机械臂位置数据与实际位置尽可能的一致
 */
void ExplorerHardware::pub_arm_joint_cmd() {

    for (auto name : arm_joint_name_) {
        //ROS_INFO_STREAM("arm command :" << name << " " << arm_joint_cmd_[name]);
        //arm_joint_pos_[name] = joint_pose_[name];//这句话不知道有什么用？？？？？------------------------------直接从moveit的消息中读取joint的信息
        //ROS_INFO_STREAM("arm " << name << " :\t" << arm_joint_pos_[name]);
        arm_joint_pos_[name] = arm_joint_cmd_[name] + arm_joint_pos_[name];//-------------------------------可能需要修改 此处是pos+cmd 也就是相对角度
        arm_joint_vel_[name] = 0.0;
        arm_joint_eff_[name] = 0.0;
        //arm_joint_cmd_[name] = 0.0;
    }

    static double last_arm1_bearing_joint = 0.0;

    if (arm_joint_cmd_["arm1_bearing_joint"] != 0.0) {
        explorer_msgs::explorer_message arm_rotate_joint_message;
        arm_rotate_joint_message.low_level_id = 5;
        arm_rotate_joint_message.high_level_id = 5;
        arm_rotate_joint_message.data.push_back(arm_joint_pos_["arm1_bearing_joint"]);
        last_arm1_bearing_joint = 1.0;
        arm_joint_cmd_["arm1_bearing_joint"] = 0.0;
        robot_drive_pub_.publish(arm_rotate_joint_message);
    } /*else if (last_arm1_bearing_joint != 0.0) {

        explorer_msgs::explorer_message arm_move_joint_message;
        arm_move_joint_message.low_level_id = 4;
        arm_move_joint_message.high_level_id = 4;
        arm_move_joint_message.data.push_back(0.0);
        last_arm1_bearing_joint = 0.0;
        robot_drive_pub_.publish(arm_move_joint_message);
    }*/

    //last_arm1_bearing_joint = arm_joint_cmd_["arm1_bearing_joint"];

    static double last_arm2_arm1_joint = 0.0, last_arm3_arm2_joint = 0.0;

    if (arm_joint_cmd_["arm2_arm1_joint"] != 0.0 || arm_joint_cmd_["arm3_arm2_joint"] != 0.0) {
        explorer_msgs::explorer_message arm_move_joint_message;
        arm_move_joint_message.low_level_id = 6;
        arm_move_joint_message.high_level_id = 6;
        arm_move_joint_message.data.push_back(arm_joint_pos_["arm2_arm1_joint"]);
        arm_move_joint_message.data.push_back(arm_joint_pos_["arm3_arm2_joint"]/2);
        last_arm2_arm1_joint = 1.0;
        last_arm3_arm2_joint = 1.0;
        arm_joint_cmd_["arm2_arm1_joint"] = 0.0;
        arm_joint_cmd_["arm3_arm2_joint"] = 0.0;
        robot_drive_pub_.publish(arm_move_joint_message);
    }/* else if (last_arm2_arm1_joint != 0.0 || last_arm3_arm2_joint != 0.0) {

        explorer_msgs::explorer_message arm_move_joint_message;
        arm_move_joint_message.low_level_id = 5;
        arm_move_joint_message.high_level_id = 5;
        arm_move_joint_message.data.push_back(0.0);
        arm_move_joint_message.data.push_back(0.0);
        last_arm2_arm1_joint = 0.0;
        last_arm3_arm2_joint = 0.0;
        robot_drive_pub_.publish(arm_move_joint_message);
    }*/

    //last_arm2_arm1_joint = arm_joint_cmd_["arm2_arm1_joint"];
    //last_arm3_arm2_joint = arm_joint_cmd_["arm3_arm2_joint"];

    if (arm_joint_cmd_["pt1_arm_joint"] != 0.0 || arm_joint_cmd_["pt2_pt1_joint"] != 0.0) {
        explorer_msgs::explorer_message camera_move_joint_message;
        camera_move_joint_message.high_level_id = 7;
        camera_move_joint_message.low_level_id  = 7;
        camera_move_joint_message.data.push_back(arm_joint_pos_["pt1_arm_joint"]);
        camera_move_joint_message.data.push_back(-arm_joint_pos_["pt2_pt1_joint"]);
        arm_joint_cmd_["pt1_arm_joint"] = 0.0;
        arm_joint_cmd_["pt2_pt1_joint"] = 0.0;
        robot_drive_pub_.publish(camera_move_joint_message);
    }

    if (arm_joint_cmd_["paw"] != 0.0 || arm_joint_cmd_["rotate_joint"] != 0.0) {
        explorer_msgs::explorer_message paw_move_joint_message;
        paw_move_joint_message.high_level_id = 8;
        paw_move_joint_message.low_level_id = 8;
        paw_move_joint_message.data.push_back(arm_joint_pos_["paw"]);
        paw_move_joint_message.data.push_back(arm_joint_pos_["rotate_joint"]);
        arm_joint_cmd_["paw"] = 0.0;
        arm_joint_cmd_["rotate_joint"] = 0.0;
        robot_drive_pub_.publish(paw_move_joint_message);
    }
}

void ExplorerHardware::jointStateCallBack(const explorer_msgs::explorer_jointConstPtr &ptr) {
    //    ROS_ERROR_STREAM("name = "<<ptr->name<<"\n") ;
    //    ROS_ERROR_STREAM("pose = "<<ptr->position<<"\n") ;
    joint_pose_[ptr->name] = ptr->position;
}

/*
 * 从参数服务器(ros.param)中获取名称列表(其实是从diff_wheel_controller中抄过来的)
 * 参数 controller_nh ros::NodeHandle     读取参数服务器数据的节点(注意其命名空间)
 * 参数 name_param    std::string         读取参数服务器数据的名称
 * 参数 names         std::vector<string> 读取参数的保存数组
 */
bool ExplorerHardware::getNameList(ros::NodeHandle controller_nh, const std::string name_param, std::vector<std::string> &names) {
    XmlRpc::XmlRpcValue name_list;

    if (!controller_nh.getParam(name_param, name_list)) {
        ROS_ERROR("can not get param list!!");
        return false;
    }

    if (name_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        if (name_list.size() == 0) {
            ROS_ERROR("did not get param name");
            return false;
        }

        for (int i = 0; i < name_list.size(); i++) {
            if (name_list[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("get the error param list");
                return false;
            }
        }

        //names.resize(name_list.size());

        for (int i = 0; i < name_list.size(); i++) {
            names.push_back(static_cast<std::string>(name_list[i]));
        }
    } else if (name_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
        names.push_back(name_list);
    } else {
        ROS_ERROR("the param get error");
        return false;
    }

    return true;
}
