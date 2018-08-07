#include <cmath>
#include <string>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
//定义pi~3.14159265...
const double pi = acos(0.0) * 2;
/*无限制关节*/
class joint {
public:
    joint() {}

    joint(std::string name_, ros::NodeHandle &nh, hardware_interface::JointCommandInterface &handle_)
        : name(name_) {
        if (!nh.getParam("reset/" + name, reset_pose)) {
            ROS_ERROR("can not get arm reset_pose for %s", name.c_str());
        }

        //ROS_ERROR_STREAM("get reset_pose for " << name << " == " << reset_pose);
        if (!nh.getParam("speed_limit/" + name, speed_limit)) {
            ROS_ERROR("can not get arm speed_limit for %s", name.c_str());
        }

        speed_limit = fabs(speed_limit);
        now_speed = speed_limit;
        
        aim_pose = now_pose = reset_pose;

        this->handle = handle_.getHandle(name);
    }

    joint(const joint &copy) {
        this->now_pose = copy.now_pose;
        this->aim_pose = copy.aim_pose;
        this->reset_pose = copy.reset_pose;
        this->name = copy.name;
        this->speed_limit = copy.speed_limit;
        this->now_speed = copy.now_speed;
        this->handle = copy.handle;
    }

    virtual void stop() {
        this->aim_pose = this->getNowPose();
    }

    // 向目标移动
    virtual bool moveToAim(double s) = 0;

    // 重置关节
    virtual void readyForResetPose() {
        this->aim_pose = this->reset_pose;
    }

    // 设定关节目标
    virtual bool setAim(double aim) {
        this->aim_pose = aim;
        return true;
    }

    virtual void setNowPose(double now_pose_) {
        this->now_pose = now_pose_;
    }

    // 获取现在的位置
    double getNowPose() {
        now_pose = handle.getPosition();
        return this->now_pose;
    }

    double getResetPose() {
        return this->reset_pose;
    }

    hardware_interface::JointHandle getHandle() {
        return this->handle;
    }

    // 告知是否可能出现不可移动
    virtual bool moveWillDanger() {
        return false;
    }

    virtual double getNowSpeed(){
        return now_speed;
    }

    virtual double getSpeedLimit() {
        return speed_limit;
    }
protected:
    // 工具函数,检查数据是否合法,返回合法的数值
    double check(double rad) {
        while (rad > pi) {
            rad -= pi * 2;
        }

        while (rad < -pi) {
            rad += pi * 2;
        }

        return rad;
    }
    // 比较两个角度,若a在b左侧,返回true
    bool compare_in_left(double a, double b) {
        if (a <= b) {
            return (b - a) < (a + 2.0 * pi - b);
        } else {
            return (a - b) > (b + 2.0 * pi - a);
        }
    }
    // 比较两个角度,若a在b右侧,返回true
    bool compare_in_right(double a, double b) {
        if (a <= b) {
            return (b - a) > (a + 2 * pi - b);
        } else {
            return (a - b) < (b + 2.0 * pi - a);
        }
    }
    // 角度的减法,获取角度的最小差值
    double sutraction(double a, double b) {
        double result = fabs(a - b);

        if (result <= pi) {
            return result;
        } else {
            return 2.0 * pi - result;
        }
    }
    std::string name;
    // 以下单位均为rad角度
    double now_pose;
    double reset_pose, aim_pose;
    // 单位为rad/s
    double speed_limit, now_speed;
    hardware_interface::JointHandle handle;
};


class joint_with_limit : public joint {
public:
    joint_with_limit() {};
    joint_with_limit(std::string name_, ros::NodeHandle &nh , hardware_interface::JointCommandInterface &handle_, double limit_left, double limit_right)
        : joint(name_, nh, handle_), left_limit(limit_left), right_limit(limit_right) {};
    joint_with_limit(std::string name_, ros::NodeHandle &nh, hardware_interface::JointCommandInterface &handle_): joint(name_, nh, handle_) {
        if (!nh.getParam("right_limit/" + name , right_limit)) {
            ROS_ERROR("can not get arm right_limit for %s", name.c_str());
        }

        //ROS_ERROR_STREAM("get the right_limit for "<< name << " == " << right_limit);
        if (!nh.getParam("left_limit/" + name  , left_limit)) {
            ROS_ERROR("can not get arm left_limit for %s", name.c_str());
        }
    }
    joint_with_limit(const joint_with_limit &copy): joint(copy) {
        this->left_limit = copy.left_limit;
        this->right_limit = copy.right_limit;
    }

    bool setNowSpeed(double speed) {
        if (speed <= 0.0 || speed > speed_limit) {
            return false;
        } else {
            now_speed = speed;
            return true;
        }
    }

    void resetSpeed() {
        now_speed = speed_limit;
    }

    double getRemainTime() {
        return fabs(now_pose - aim_pose) / now_speed;
    }

    bool moveToAim(double s) {
        //now_speed = speed_limit;
        this->getNowPose();

        if (this->now_pose == this->aim_pose) {

            handle.setCommand(0.0);
            return true;
        }

        // 计算当前一次移动的角度大小
        double move_rad = fabs(this->now_speed * s);

        if (move_rad > pi / 2.0) {
            move_rad = pi / 2.0;
        }

        // 没有限制的情况
        if (fabs(aim_pose - now_pose) < move_rad) {
            // 检查是否足够达到目标
            handle.setCommand(aim_pose - now_pose);
            return true;
        } else {
            if (aim_pose < now_pose) {

                handle.setCommand(-move_rad);
            } else {

                handle.setCommand(move_rad);
            }

            return false;
        }

        return false;
    }

    bool moveWillDanger() {
        return (now_pose == left_limit) || (now_pose == right_limit);
    }

    bool setAim(double aim) {
        if (aim >= left_limit && aim <= right_limit) {
            this->aim_pose = aim;
            return true;
        } else if (aim < left_limit) {
            this->aim_pose = left_limit;
        } else {
            this->aim_pose = right_limit;
        }

        return false;
    }
protected:
    double left_limit, right_limit;
};









