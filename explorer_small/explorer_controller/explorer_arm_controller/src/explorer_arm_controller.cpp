#include "explorer_arm_controller.h"
#include <cmath>

using namespace explorer_arm_controller;
ArmController::ArmController() {}
ArmController::~ArmController() {}
bool ArmController::init(hardware_interface::PositionJointInterface *joint_handle_,
                         ros::NodeHandle &root_nh,
                         ros::NodeHandle &controller_nh) {

    // 获取机械臂名称列表
    if (!getNameList(controller_nh, "joints", this->arm_name)) {
        ROS_ERROR("can not get name list");
        return false;
    }

    // 建立机械臂描述对象
    for (auto it:arm_name) {
        joint_map.insert(std::make_pair(it, new joint_with_limit(it, controller_nh, *joint_handle_)));
    }

    this->state_sub_  = controller_nh.subscribe("/explorer_arm_driect", 10, &ArmController::jointStateSub, this);
    this->reset_sub_  = controller_nh.subscribe("/explorer_reset", 10, &ArmController::resetStateSub, this);
    this->paw_sub_    = controller_nh.subscribe("/explorer_paw", 10, &ArmController::pawStateSub   , this);
    this->moveit_sub_ = controller_nh.subscribe("/explorer_moveit_listener/explorer_moveit_joint", 50, &ArmController::moveitSub, this);
    this->yuntai_sub_ = controller_nh.subscribe("/explorer_serial_data/19",10,&ArmController::yuntaisub,this);
    //this->imu_pub_ = controller_nh.advertise<sensor_msgs::Imu>("imu", 1);
    ROS_ERROR("arm init successed");
    return true;
}
/*
 *机械臂数据的数据刷新
 */
void ArmController::update(const ros::Time &time, const ros::Duration &period) {
    ros::Time now_time = time;
    // 计算与上一次的刷新的时间差
    double timep = fabs((front_time.toSec() - now_time.toSec()));
    front_time = now_time;

    // 显示所有机械臂的位置,如果机械臂的位置达到限制,标红
    // 为了显示顺序,map遍历没有采用迭代器
    for (auto name : arm_name) {
        if (joint_map[name]->moveWillDanger()) {
            ROS_ERROR_STREAM("arm " << name << " :\t" << joint_map[name]->getNowPose());
        } else {
            ROS_INFO_STREAM("arm " << name << " :\t" << joint_map[name]->getNowPose());
        }
    }

    // 检测是否需要复位
    if (!reset_queue.empty()) {
        std::vector<std::pair<std::string, double> > line;
        // 取出当前的目标
        line = reset_queue.front();
        int get_goal = 0;

        for (auto name : line) {
            joint_map[name.first]->setAim(name.second);

            if (joint_map[name.first]->moveToAim(timep)) {
                // 记录达到目标的数量
                ++get_goal;
            }
        }

        // 全部达到目标,转入下一个位置
        if (get_goal == line.size()) {
            reset_queue.pop();
        }

        return;
    }


    // 建议对map的遍历使用iterator保证效率
    for (auto it : joint_map) {
        it.second->moveToAim(timep);
    }

}
void ArmController::jointStateSub(const geometry_msgs::TwistStamped &msg) {
    if (msg.twist.linear.x != 0 || msg.twist.linear.y != 0 || msg.twist.linear.z != 0 ||
        msg.twist.angular.x != 0 || msg.twist.angular.y != 0 || msg.twist.angular.z != 0) {
        while (reset_queue.size()) {
            reset_queue.pop();
        }
    }
    joint_map["arm1_bearing_joint"  ]->setAim(joint_map["arm1_bearing_joint" ]->getNowPose()//整体旋转
                                              + msg.twist.linear.y);
    joint_map["arm2_arm1_joint"     ]->setAim(joint_map["arm2_arm1_joint"    ]->getNowPose()//整体上下
                                               + msg.twist.linear.x);
    joint_map["arm3_arm2_joint"     ]->setAim(joint_map["arm3_arm2_joint"    ]->getNowPose()//小臂上下
                                                - msg.twist.linear.z);

    joint_map["pt1_arm_joint"       ]->setAim(joint_map["pt1_arm_joint"      ]->getNowPose()//第一轴向
                                              - msg.twist.angular.y);
    joint_map["pt2_pt1_joint"       ]->setAim(joint_map["pt2_pt1_joint"      ]->getNowPose()//摆动
                                              - msg.twist.angular.x);
    joint_map["rotate_joint"        ]->setAim(joint_map["rotate_joint"       ]->getNowPose()//转动，转爪子
                                              + msg.twist.angular.z);
}

void ArmController::pawStateSub(const std_msgs::Float32 &ptr) {
    double static const cnt = 0.07;
    joint_map["paw"]->setAim(joint_map["paw"]->getNowPose() + ptr.data);
}
void ArmController::resetStateSub(const explorer_msgs::explorer_reset &ptr) {
    ROS_INFO("reset pub");
    now_moveit_pose = -1;

    while (!reset_queue.empty()) {
        reset_queue.pop();
    }

    if (ptr.reset_arm) {
        if (fabs(joint_map["arm1_bearing_joint"]->getResetPose() - joint_map["arm1_bearing_joint"]->getNowPose()) < pi / 4) {
            // 当机械臂偏移角度不大时(小于45度),直接复位
            for (auto it = joint_map.begin(); it != joint_map.end(); ++it) {
                it->second->readyForResetPose();
            }
        } else {
            // 当机械臂偏移角度过大时,分步骤复位
            std::vector<std::pair<std::string, double> >  queue;

            /*第一步,将大臂立起来*/
            queue.push_back(std::make_pair(std::string("arm2_arm1_joint"), 0.0/*这个角度是目测出来的*/));
            queue.push_back(std::make_pair(std::string("arm3_arm2_joint"), 0.0/*这个角度是目测出来的*/));
            reset_queue.push(queue);

            queue.clear();

            /*第二步,复位小臂以及将大臂转正*/
            queue.push_back(std::make_pair(std::string("arm1_bearing_joint"), joint_map["arm1_bearing_joint"]->getNowPose()));
            queue.push_back(std::make_pair(std::string("pt1_arm_joint"), joint_map["pt1_arm_joint"]->getNowPose()));
            queue.push_back(std::make_pair(std::string("pt2_pt1_joint"), joint_map["pt2_pt1_joint"]->getNowPose()));
            queue.push_back(std::make_pair(std::string("rotate_joint"), joint_map["rotate_joint"]->getNowPose()));
            queue.push_back(std::make_pair(std::string("paw"), joint_map["paw"]->getNowPose()));
            reset_queue.push(queue);

            queue.clear();

            /*第三步,将大臂复位*/
            queue.push_back(std::make_pair(std::string("arm2_arm1_joint"), joint_map["arm2_arm1_joint"]->getNowPose()));
            queue.push_back(std::make_pair(std::string("arm3_arm2_joint"), joint_map["arm3_arm2_joint"]->getNowPose()));
            reset_queue.push(queue);
        }
    } else if (ptr.reset_camera) {
        for (int i = 3; i < arm_name.size(); ++i) {
            joint_map[arm_name[i]]->readyForResetPose();
        }
    } else if (ptr.reset_paws) {
        joint_map["rotate_joint"]->readyForResetPose();
        joint_map["paw"]->setAim(0.0);
    }
}

void ArmController::moveitSub(const explorer_msgs::explorer_moveit_values &ptr) {
   /* if (ptr.values[0] != 0 || ptr.values[1] != 0 || ptr.values[2] != 0 ||
        ptr.values[3] != 0 || ptr.values[4] != 0 || ptr.values[5] != 0) {
        while (reset_queue.size()) {
            reset_queue.pop();
        }
    }*/
    joint_map["arm1_bearing_joint"  ]->setAim(joint_map["arm1_bearing_joint" ]->getNowPose()
                                              + ptr.values[0] );
    joint_map["arm2_arm1_joint"     ]->setAim(joint_map["arm2_arm1_joint"    ]->getNowPose()
                                              + ptr.values[1] );
    joint_map["arm3_arm2_joint"     ]->setAim(joint_map["arm3_arm2_joint"    ]->getNowPose()
                                              + ptr.values[2] );

    joint_map["pt1_arm_joint"       ]->setAim(joint_map["pt1_arm_joint"      ]->getNowPose()
                                              + ptr.values[3] );
    joint_map["pt2_pt1_joint"       ]->setAim(joint_map["pt2_pt1_joint"      ]->getNowPose()
                                              + ptr.values[4] );
    joint_map["rotate_joint"        ]->setAim(joint_map["rotate_joint"       ]->getNowPose()
                                              + ptr.values[5] );
}

void ArmController::yuntaisub(explorer_msgs::explorer_low_level_data msg){
    joint_map["joint_front_back"] ->setAim(-msg.can_serial_data_2 * 3.1415926 /180);
    joint_map["robot_left_right"] ->setAim(-msg.can_serial_data_1 * 3.1415926 /180);
    //向上传入的四元数
    
}

bool ArmController::getNameList(ros::NodeHandle controller_nh, const std::string name_param, std::vector<std::string> &names) {
    XmlRpc::XmlRpcValue name_list;

    if (!controller_nh.getParam(name_param, name_list)) {
        ROS_ERROR("can not get the wheel list!!");
        return false;
    }

    if (name_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        if (name_list.size() == 0) {
            ROS_ERROR("did not get the wheel name");
            return false;
        }

        for (int i = 0; i < name_list.size(); i++) {
            if (name_list[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("get the error name list");
                return false;
            }
        }

        names.resize(name_list.size());

        for (int i = 0; i < name_list.size(); i++) {
            names[i] = static_cast<std::string>(name_list[i]);
        }
    } else if (name_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
        names.push_back(name_list);
    } else {
        ROS_ERROR("the wheel param get error");
        return false;
    }

    return true;
}


