/*****************************************
 * @概述:这是explorerJoyExplainer的检查,通过
 * 显示得到的相关数据信息检查编写是否正确
 *****************************************
 * @作者:潘学谦-西工大舞蹈机器人基地-救援组
 * @创建时间:2017-07-20
 *****************************************/
#include <ros/ros.h>
#include <iostream>
#include <map>
#include "explorer_joy_explainer.hpp"
std::vector< std::string >button_dic;
autoJoyExplainer joy_msg;

void joyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
    std::cout << "get" << std::endl;
    std::cout << joy->axes.size() << std::endl;
    std::cout << joy->buttons.size() << std::endl;
    joy_msg.getMessage(joy);

    for (int i = 0; i <  joy_msg.button_size; ++i) {
        if (joy_msg.askForButton(i)) {
            std::cout << "push the " << button_dic[i] << std::endl;
        } else {
            std::cout << "nopush the " << button_dic[i] << std::endl;
        }
    }

    for (int i = 0; i <  joy_msg.axes_size; ++i) {
        std::cout << i << "\t" << joy_msg.askForAxes(i) << std::endl;
    }
}

int main(int argc , char **argv) {
    ros::init(argc , argv, "explorer_joy_control");
    ros::NodeHandle node;
    button_dic.push_back(std::string("L1"));
    button_dic.push_back(std::string("L2"));
    button_dic.push_back(std::string("R1"));
    button_dic.push_back(std::string("R2"));
    button_dic.push_back(std::string("button1"));
    button_dic.push_back(std::string("button2"));
    button_dic.push_back(std::string("button3"));
    button_dic.push_back(std::string("button4"));
    button_dic.push_back(std::string("up"));
    button_dic.push_back(std::string("down"));
    button_dic.push_back(std::string("left"));
    button_dic.push_back(std::string("right"));
    button_dic.push_back(std::string("left_axes_up"));
    button_dic.push_back(std::string("left_axes_down"));
    button_dic.push_back(std::string("left_axes_right"));
    button_dic.push_back(std::string("left_axes_left"));
    button_dic.push_back(std::string("right_axes_up"));
    button_dic.push_back(std::string("right_axes_down"));
    button_dic.push_back(std::string("right_axes_left"));
    button_dic.push_back(std::string("right_axes_right"));
    button_dic.push_back(std::string("left_axes_button"));
    button_dic.push_back(std::string("right_axes_button"));

    std::cout << "2333" << std::endl;
    ros::Subscriber joy_sub_ =
        node.subscribe<sensor_msgs::Joy>("joy", 2, &joyCallback);
    ros::spin();
}


