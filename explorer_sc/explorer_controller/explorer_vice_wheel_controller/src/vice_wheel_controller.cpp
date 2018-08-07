
//author : rescuer liao
//date : 2015-4-20

#include "vice_wheel_controller.h"

using namespace explorer_vice_wheel_controller ;

ViceWheelController::ViceWheelController():
    current_command_(),
    timeout_(0.0),
    angular_multiplier_(1.0) {}

ViceWheelController::~ViceWheelController() {}

bool ViceWheelController::init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {

    if (!getViceWheelNameList(controller_nh, "front_vice_wheel", front_vice_wheel_names_) ||
        !getViceWheelNameList(controller_nh, "back_vice_wheel", back_vice_wheel_names_)) {
        ROS_ERROR("can not get the vice wheel names!!!") ;
        return false ;
    }

    controller_nh.param("angular_multiplier", angular_multiplier_,50.0) ;
    controller_nh.param("max_speed", max_speed, 1.0);
    controller_nh.param("time_out", timeout_ , 0.5) ;

    for (auto name : front_vice_wheel_names_) {
        front_vice_wheel_handles_[name] = hw->getHandle(name);
    }

    for (auto name : back_vice_wheel_names_) {
        back_vice_wheel_handles_[name] = hw->getHandle(name) ;
    }

    vice_wheel_sub_ = controller_nh.subscribe("explorer_vice_wheel", 10, &ViceWheelController::viceWheelCallBack, this) ;
    vice_reset_sub = controller_nh.subscribe("explorer_vice_wheel_reset", 10, &ViceWheelController::resetCommandCallBack, this);
    //imu_pub = controller_nh.subscribe("explorer_vice_wheel_reset", 10, &ViceWheelController::resetCommandCallBack, this);
    //来自explorer_joy_control
    return true ;
}

void ViceWheelController::update(const ros::Time &time, const ros::Duration &period) {
    current_command_ = *(realtime_buffer_.readFromRT()) ;
    double dt = (time - current_command_.time).toSec() ;

    /* 信息实时性判定 */
    if (dt > timeout_) {
        /* 如果超时,将数据清零 */
        Commands command;
        current_command_ = command;
        reset_msg.front_vice_wheel_reset = reset_msg.back_vice_wheel_reset = false;
    }

    if (!reset_msg.front_vice_wheel_reset && !reset_msg.back_vice_wheel_reset) {
        command_map["left_up_fin_base_joint"] = current_command_.front_left_vice_angular * angular_multiplier_;
        command_map["right_up_fin_base_joint"] = current_command_.front_right_vice_angular * angular_multiplier_;
        command_map["left_down_fin_base_joint"] = -current_command_.back_left_vice_angular * angular_multiplier_;
        command_map["right_down_fin_base_joint"] = -current_command_.back_right_vice_angular * angular_multiplier_;
    } else {
        if (reset_msg.front_vice_wheel_reset) {
            if (std::fabs(front_vice_wheel_handles_["left_up_fin_base_joint"].getPosition() - front_vice_wheel_handles_["right_up_fin_base_joint"].getPosition()) > 10e-3) {
                double diff = front_vice_wheel_handles_["left_up_fin_base_joint"].getPosition() - front_vice_wheel_handles_["right_up_fin_base_joint"].getPosition();

                if (std::fabs(diff) > max_speed * angular_multiplier_) {
                    diff = max_speed * angular_multiplier_ * (diff / (std::fabs(diff)));
                }

                command_map["left_up_fin_base_joint"] = diff;
                command_map["right_up_fin_base_joint"] = -diff;
            } else {
                if (std::fabs(front_vice_wheel_handles_["left_up_fin_base_joint"].getPosition() - 0.7) > 10e-3) {
                    double diff = front_vice_wheel_handles_["left_up_fin_base_joint"].getPosition() - 0.7;

                    if (std::fabs(diff) > max_speed * angular_multiplier_) {
                        diff = max_speed * angular_multiplier_ * (diff / (std::fabs(diff)));
                    }

                    command_map["left_up_fin_base_joint"] = -diff;
                    command_map["right_up_fin_base_joint"] = -diff;

                }
            }

            if (reset_msg.back_vice_wheel_reset) {
                if (std::fabs(back_vice_wheel_handles_["left_down_fin_base_joint"].getPosition() - back_vice_wheel_handles_["right_down_fin_base_joint"].getPosition()) > 10e-3) {
                    double diff = back_vice_wheel_handles_["left_down_fin_base_joint"].getPosition() - back_vice_wheel_handles_["right_down_fin_base_joint"].getPosition();

                    if (std::fabs(diff) > max_speed * angular_multiplier_) {
                        diff = max_speed * angular_multiplier_ * (diff / (std::fabs(diff)));
                    }

                    command_map["left_down_fin_base_joint"] = -diff;
                    command_map["right_down_fin_base_joint"] = diff;
                }  else {
                    if (std::fabs(back_vice_wheel_handles_["left_down_fin_base_joint"].getPosition() + 0.7) > 10e-3) {
                        double diff = back_vice_wheel_handles_["left_down_fin_base_joint"].getPosition() + 0.7;

                        if (std::fabs(diff) > max_speed * angular_multiplier_) {
                            diff = max_speed * angular_multiplier_ * (diff / (std::fabs(diff)));
                        }

                        command_map["left_down_fin_base_joint"] = -diff;
                        command_map["right_down_fin_base_joint"] = -diff;

                    }
                }
            }
        }
    }


    ROS_INFO_STREAM("current position of the vice wheel is :");

    /* 这里使用了c++11的特性auto关键字
     * (其实就是懒得写长长的类型名 + 装X)
     * 需要在CMakeLists里面添加add_compile_options(-std=c++11)
     */
    for (auto handle : back_vice_wheel_handles_) {
        ROS_INFO_STREAM(handle.first << " : " << handle.second.getPosition() << "\t" << command_map[handle.first]);
        handle.second.setCommand(command_map[handle.first]);
        command_map[handle.first] = 0.0;
    }

    for (auto handle : front_vice_wheel_handles_) {
        ROS_INFO_STREAM(handle.first << " : " << handle.second.getPosition() << "\t" << command_map[handle.first]);
        handle.second.setCommand(command_map[handle.first]);
        command_map[handle.first] = 0.0;
    }
}

void ViceWheelController::stopping(const ros::Time &time) {
    brake();
}

void ViceWheelController::starting(const ros::Time &time) {
    brake();
}

void ViceWheelController::brake() {
    for (auto handle : front_vice_wheel_handles_) {
        handle.second.setCommand(0.0) ;
    }

    for (auto handle : back_vice_wheel_handles_) {
        handle.second.setCommand(0.0) ;
    }
}

void ViceWheelController::viceWheelCallBack(const explorer_msgs::explorer_vice_wheelConstPtr &ptr) {
    reset_msg.front_vice_wheel_reset = reset_msg.back_vice_wheel_reset = false;
    last_command_.time = ros::Time::now() ;
    last_command_.front_left_vice_angular = ptr->front_left_wheel_angular;
    last_command_.front_right_vice_angular = ptr->front_right_wheel_angular;
    last_command_.back_left_vice_angular = ptr->back_left_wheel_angular;
    last_command_.back_right_vice_angular = ptr->back_right_wheel_angular;
    realtime_buffer_.writeFromNonRT(last_command_);
}

void ViceWheelController::resetCommandCallBack(const explorer_msgs::explorer_vice_reset &msg) {
    reset_msg = msg;
    last_command_.time = ros::Time::now() ;
    last_command_.front_left_vice_angular =
        last_command_.front_right_vice_angular =
            last_command_.back_left_vice_angular =
                last_command_.back_right_vice_angular = 0.0;
    realtime_buffer_.writeFromNonRT(last_command_);
}

bool ViceWheelController::getViceWheelNameList(ros::NodeHandle controller_nh, const string vice_wheel_param, vector<string> &vice_wheel_names) {
    XmlRpc::XmlRpcValue vice_wheel_list ;

    if (!controller_nh.getParam(vice_wheel_param, vice_wheel_list)) {
        ROS_ERROR("can not get the wheel list!!") ;
        return false ;
    }

    if (vice_wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        if (vice_wheel_list.size() == 0) {
            ROS_ERROR("did not get the wheel name") ;
            return false ;
        }

        for (int i = 0 ; i < vice_wheel_list.size() ; i++) {
            if (vice_wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("get the error name list") ;
                return false ;
            }
        }

        vice_wheel_names.resize(vice_wheel_list.size());

        for (int i = 0 ; i < vice_wheel_list.size() ; i++) {
            vice_wheel_names[i] = static_cast<string>(vice_wheel_list[i]) ;
        }
    } else if (vice_wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
        vice_wheel_names.push_back(vice_wheel_list);
    } else {
        ROS_ERROR("the wheel param get error") ;
        return false ;
    }

    return true;
}

