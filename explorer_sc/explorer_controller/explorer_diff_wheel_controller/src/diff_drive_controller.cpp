
//author : rescuer liao
//date : 2015-4-20

#include "diff_drive_controller.h"

using namespace std ;

namespace explorer_diff_drive_controller {
DiffDriveController::DiffDriveController()
    : current_command_(),
      wheel_radius_(0.0),
      wheel_radius_multiplier_(1.0),
      wheel_separation_(0.0),
      cmd_vel_timeout_(1.0) {

}


DiffDriveController::~DiffDriveController() {

}

bool DiffDriveController::init(hardware_interface::VelocityJointInterface *hw,
                               ros::NodeHandle &root_nh,
                               ros::NodeHandle &controller_nh) {
    if (!getWheelNameLists(controller_nh , "left_wheel", left_wheel_name_)
        || !getWheelNameLists(controller_nh, "right_wheel", right_wheel_name_)) {
        ROS_ERROR("could not get the wheel list ,make sure you have get the wheel list") ;
        return false ;
    }
    //读取车轮半径
    controller_nh.param("wheel_radius", wheel_radius_, 0.1) ;
    //change this param from your robot
    //车轮乘数  一般取1.0
    controller_nh.param("wheel_radius_multiplier", wheel_radius_multiplier_, 1.0) ;
    //车轮间隙
    controller_nh.param("wheel_separation", wheel_separation_, 0.4) ;
    controller_nh.param("time_out" , cmd_vel_timeout_ , 2.0) ;
    // 读取车长度
    controller_nh.param("length" , car_length, 0.4/*这个数字是随便填的,请根据小车实际填写*/);
    // 读取车宽度
    controller_nh.param("width" , car_width, 0.4/*这个数字是随便填的,请根据小车实际填写*/);
    // 生成小车原地自转的参数
    raw_param = 1.0;

    // for (int i = 0 ; i < left_wheel_name_.size() ; i++) {
    //     left_wheel_joint_[left_wheel_name_[i]] = hw->getHandle(left_wheel_name_[i]) ;
    // }

    // for (int i  = 0 ; i < right_wheel_name_.size() ; i++) {
    //     right_wheel_joint_[right_wheel_name_[i]] = hw->getHandle(right_wheel_name_[i]) ;
    // }
    for (auto name : left_wheel_name_){
        left_wheel_joint_.insert(std::make_pair(name,hw->getHandle(name)));
    }
    for (auto name : right_wheel_name_){
        right_wheel_joint_.insert(std::make_pair(name,hw->getHandle(name)));
    }

    cmd_vel_sub_ = controller_nh.subscribe("cmd_vel", 1, &DiffDriveController::velocityCallBack, this) ;
    return true ;
}

void DiffDriveController::starting(const ros::Time &time) {
    brake();
    ROS_INFO("the diff wheel controller start") ;
}

void DiffDriveController::stopping(const ros::Time &time) {
    brake();
}

void DiffDriveController::update(const ros::Time &time, const ros::Duration &period) {
    current_command_ = *(realtime_buffer_.readFromRT()) ;
    double dt = (time - current_command_.current_time).toSec() ;

    double current_left_vel = left_wheel_joint_[left_wheel_name_[0]].getVelocity() ;
    double current_right_vel = right_wheel_joint_[right_wheel_name_[0]].getVelocity() ;

    //ROS_INFO_STREAM("current left_wheel_vel = " << current_left_vel << "      " << "current_right_vel = " << current_right_vel) ;


    if (dt > cmd_vel_timeout_) {
        current_command_.linear_x_vel = 0.0;
        current_command_.linear_y_vel = 0.0;
        current_command_.angular_z_vel = 0.0;
        current_command_.linear_z_vel = 0.0;
    }

    //to get the vel of left wheel and right wheel
    // double angular_z =current_command_.angular_z_vel + atan(current_command_.linear_y_vel/current_command_.linear_x_vel) ;
    // double x_vel = pow(current_command_.linear_x_vel,2)+ pow(current_command_.linear_y_vel,2);
    // x_vel = sqrt(x_vel);
    // double left_up_wheel_vel,left_down_wheel_vel,right_up_wheel_vel,right_down_wheel_vel;
    // left_up_wheel_vel = left_down_wheel_vel = (x_vel - angular_z *wheel_separation_ * raw_param / 2.0) / wheel_radius_ * wheel_radius_multiplier_;
    // right_up_wheel_vel = right_down_wheel_vel = (x_vel + angular_z *wheel_separation_ * raw_param / 2.0) / wheel_radius_ * wheel_radius_multiplier_;
    //小车结算 直接忽略y轴速度
    double left_up_wheel_vel = (current_command_.linear_x_vel - current_command_.linear_y_vel - current_command_.angular_z_vel * wheel_separation_ * raw_param / 2.0) / wheel_radius_ * wheel_radius_multiplier_;
    double left_down_wheel_vel = (current_command_.linear_x_vel + current_command_.linear_y_vel - current_command_.angular_z_vel * wheel_separation_ * raw_param / 2.0) / wheel_radius_ * wheel_radius_multiplier_;
    //double left_down_wheel_vel = current_command_.linear_z_vel;//用来重启底盘
    //double right_up_wheel_vel = (current_command_.linear_x_vel ) / wheel_radius_ * wheel_radius_multiplier_;
    double right_up_wheel_vel = (current_command_.linear_x_vel + current_command_.linear_y_vel + current_command_.angular_z_vel * wheel_separation_ * raw_param / 2.0) / wheel_radius_ * wheel_radius_multiplier_;
    double right_down_wheel_vel = (current_command_.linear_x_vel - current_command_.linear_y_vel + current_command_.angular_z_vel * wheel_separation_ * raw_param / 2.0) / wheel_radius_ * wheel_radius_multiplier_;
    
    ROS_INFO_STREAM("left_up_wheel_vel   = " << left_up_wheel_vel <<   "   " << "right_up_wheel_vel   = " << right_up_wheel_vel) ;
    ROS_INFO_STREAM("left_down_wheel_vel = " << left_down_wheel_vel << "   " << "right_down_wheel_vel = " << right_down_wheel_vel) ;

    //get the vel of each wheel and set the vel command
    left_wheel_joint_["left_up_wheel_base_joint"].setCommand(left_up_wheel_vel);
    left_wheel_joint_["left_down_wheel_base_joint"].setCommand(left_down_wheel_vel);
    right_wheel_joint_["right_up_wheel_base_joint"].setCommand(right_up_wheel_vel);
    right_wheel_joint_["right_down_wheel_base_joint"].setCommand(right_down_wheel_vel);
}

void DiffDriveController::brake() {
    double left_wheel_vel = 0.0 ;
    double right_wheel_vel = 0.0 ;

    for (auto joint : left_wheel_joint_) {
        joint.second.setCommand(left_wheel_vel) ;
    }

    for (auto joint : right_wheel_joint_) {
        joint.second.setCommand(right_wheel_vel) ;
    }
}

bool DiffDriveController::getWheelNameLists(ros::NodeHandle controller_nh,  const string wheel_param ,  vector<string> &wheel_names) {
    XmlRpc::XmlRpcValue wheel_list ;

    if (!controller_nh.getParam(wheel_param, wheel_list)) {
        ROS_ERROR("can not get the wheel list!!") ;
        return false ;
    }

    if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        if (wheel_list.size() == 0) {
            ROS_ERROR("did not get the wheel name") ;
            return false ;
        }

        for (int i = 0 ; i < wheel_list.size() ; i++) {
            if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("get the error name list") ;
                return false ;
            }
        }

        wheel_names.resize(wheel_list.size());

        for (int i = 0 ; i < wheel_list.size() ; i++) {
            wheel_names[i] = static_cast<string>(wheel_list[i]) ;
        }
    } else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
        wheel_names.push_back(wheel_list);
    } else {
        ROS_ERROR("the wheel param get error") ;
        return false ;
    }

    return true;
}

void DiffDriveController::velocityCallBack(const geometry_msgs::TwistConstPtr &ptr) {
    //  ROS_INFO("get cmd vel") ;
    last_command_.linear_x_vel = ptr->linear.x;
    // y向右为正
    last_command_.linear_y_vel = ptr->linear.y;
    last_command_.linear_z_vel = ptr->linear.z;

    last_command_.angular_z_vel = ptr->angular.z;
    last_command_.current_time = ros::Time::now() ;
    realtime_buffer_.writeFromNonRT(last_command_);
}

}


