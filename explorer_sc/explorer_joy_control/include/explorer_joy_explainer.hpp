/****************************************
 * @概述:这是joy消息的解释器,用于简化joy消息
 * 的读取主要是为了将所有动作简化为统一参数便于
 * 适应主消息控制逻辑的调用
 * 请务必在使用不同的手柄之后修改本代码以适应
 * 新的手柄反馈
 * 接受来自package joy joy_node 的消息
 * 由于`joy joy_node`节点接收手柄数据是热插
 * 拔的，这里通过button和axes的长度判断手柄类型
 * 可以考虑改成/diagnostics内部的信息
 *****************************************
 * @作者:潘学谦-西工大舞蹈机器人基地-救援组
 * @创建时间:2017-3-17
 ****************************************/
#ifndef explorerJoyExplainer_HPP
#define explorerJoyExplainer_HPP
#pragma once
#include <sensor_msgs/Joy.h>
class explorerJoyExplainer {
public:
    explorerJoyExplainer() {

    };
    virtual ~explorerJoyExplainer() {};
    virtual void getMessage(sensor_msgs::Joy::ConstPtr msg) = 0;
    virtual double askForAxes(int name) = 0;
    virtual bool askForButton(int name) = 0;
    static int getButtonListSize() {
        return 0;
    };
    static int getAxesListSize() {
        return 0;
    };

    const static int L1 = 0;
    const static int L2 = 1;
    const static int R1 = 2;
    const static int R2 = 3;
    const static int button1 = 4;
    const static int button2 = 5;
    const static int button3 = 6;
    const static int button4 = 7;
    const static int up = 8;
    const static int down = 9;
    const static int left = 10;
    const static int right = 11;
    const static int left_axes_up = 12;
    const static int left_axes_down = 13;
    const static int left_axes_right = 14;
    const static int left_axes_left = 15;
    const static int right_axes_up = 16;
    const static int right_axes_down = 17;
    const static int right_axes_left = 18;
    const static int right_axes_right = 19;
    const static int left_axes_button = 20;
    const static int right_axes_button = 21;
    const static int button_size = 22;

    const static int up_down = 1;
    const static int left_right = 2;
    const static int left_axes_up_down = 3;
    const static int left_axes_left_right = 4;
    const static int right_axes_up_down = 5;
    const static int right_axes_left_right = 6;
    const static int right_button_up_down = 7;
    const static int right_button_left_right = 8;
    const static int axes_size = 9;
};

class saiTaiKe : public explorerJoyExplainer {
private:
    const double threshold = 0.1;
public:
    saiTaiKe() :  button(button_size, false), axes(axes_size, 0.0) , explorerJoyExplainer() {
        _L1 = 6;
        _L2 = 8;
        _R1 = 7;
        _R2 = 9;
        _button1 = 4;
        _button2 = 1;
        _button3 = 0;
        _button4 = 3;
        _up_down = 7;                  //1.0 上 -1.0 下
        _left_right = 6;               //1.0 左 -1.0 右
        _left_axes_up_down = 1;        //1.0 上 -1.0 下
        _left_axes_right_left = 0;     //1.0 左 -1.0 右
        _left_axes_button = 13;
        _right_axes_up_down = 3;       //1.0 上 -1.0 下
        _right_axes_left_right = 2;    //1.0 左 -1.0 右
        _right_axes_button = 14;
    }
    virtual ~saiTaiKe() {};

    static int getButtonListSize() {
        return 15;
    }
    static int getAxesListSize() {
        return 8;
    }

    void getMessage(sensor_msgs::Joy::ConstPtr msg) {
        // 获取joy消息后进行处理
        button.at(L1) = msg->buttons[_L1];
        button.at(R1) = msg->buttons[_R1];
        button.at(L2) = msg->buttons[_L2];
        // 存在另一种判定方法,但是建议使用上边这种
        // axes.at(L2) = msg->axes[5]<0.0;
        button.at(R2) = msg->buttons[_R2];
        // 存在另一种判定方法,但是建议使用上边这种
        // axes.at(R2) = msg->axes[6]<0.0;
        button.at(button1) = msg->buttons[_button1];
        button.at(button2) = msg->buttons[_button2];
        button.at(button3) = msg->buttons[_button3];
        button.at(button4) = msg->buttons[_button4];
        button.at(left)     = msg->axes[_left_right]    > threshold;
        button.at(right)    = msg->axes[_left_right]    < -threshold;
        button.at(up)       = msg->axes[_up_down]       > threshold;
        button.at(down)     = msg->axes[_up_down]       < -threshold;
        button.at(left_axes_left)   = msg->axes[_left_axes_right_left]  > threshold;
        button.at(left_axes_right)  = msg->axes[_left_axes_right_left]  < -threshold;
        button.at(left_axes_up)     = msg->axes[_left_axes_up_down]     > threshold;
        button.at(left_axes_down)   = msg->axes[_left_axes_up_down]     < -threshold;
        button.at(right_axes_left)  = msg->axes[_right_axes_left_right] > threshold;
        button.at(right_axes_right) = msg->axes[_right_axes_left_right] < -threshold;
        button.at(right_axes_up)    = msg->axes[_right_axes_up_down]    > threshold;
        button.at(right_axes_down)  = msg->axes[_right_axes_up_down]    < -threshold;
        button.at(left_axes_button) = msg->buttons[_left_axes_button];
        button.at(right_axes_button) = msg->buttons[_right_axes_button];

        axes.at(up_down) = msg->axes[_up_down];
        axes.at(left_right) = msg->axes[_left_right];
        axes.at(left_axes_up_down) = msg->axes[_left_axes_up_down];
        axes.at(left_axes_left_right) = msg->axes[_left_axes_right_left];
        axes.at(right_axes_up_down) = msg->axes[_right_axes_up_down];
        axes.at(right_axes_left_right) = msg->axes[_right_axes_left_right];
        axes.at(right_button_up_down) = msg->buttons[_button1] - msg->buttons[_button3];
        axes.at(right_button_left_right) = msg->buttons[_button4] - msg->buttons[_button2];
    }
    bool askForButton(int name) {
        // 读取数据
        if (name < 0 || name >= button_size) {
            return false;
        }

        return button.at(name);
    }

    double askForAxes(int name) {
        if (name == 0 || name >= axes_size || name <= -axes_size) {
            return 0.0;
        }

        if (name > 0) {
            return axes.at(name);
        } else {
            return - axes.at(-name);
        }
    }
protected:
    // 以下为按键在joy的消息中的位置
    // 请在更换手柄后更改以下数据
    int _L1;
    int _L2;
    int _R1;
    int _R2;
    int _button1;
    int _button2;
    int _button3;
    int _button4;
    int _up_down;                  //1.0 上 -1.0 下
    int _left_right;               //1.0 左 -1.0 右
    int _left_axes_up_down;        //1.0 上 -1.0 下
    int _left_axes_right_left;     //1.0 左 -1.0 右
    int _left_axes_button;
    int _right_axes_up_down;       //1.0 上 -1.0 下
    int _right_axes_left_right;    //1.0 左 -1.0 右
    int _right_axes_button;
    ::std::vector<bool> button;
    ::std::vector<double> axes;
};

class shoubing: public explorerJoyExplainer {
private:
    const double threshold = 0.1;
public:
    shoubing() : button(button_size, false), axes(axes_size, 0.0), explorerJoyExplainer() {
        _L1 = 4;
        _L2 = 6;
        _R1 = 5;
        _R2 = 7;
        _button1 = 0;
        _button2 = 1;
        _button3 = 2;
        _button4 = 3;
        _up_down = 5;                  //1.0 上 -1.0 下
        _left_right = 4;               //1.0 左 -1.0 右
        _left_axes_up_down = 1;        //1.0 上 -1.0 下
        _left_axes_right_left = 0;     //1.0 左 -1.0 右
        _left_axes_button = 10;
        _right_axes_up_down = 3;       //1.0 上 -1.0 下
        _right_axes_left_right = 2;    //1.0 左 -1.0 右
        _right_axes_button = 11;
    }
    virtual ~shoubing() {};
    static int getButtonListSize() {
        return 12;
    }
    static int getAxesListSize() {
        return 6;
    }


    void getMessage(sensor_msgs::Joy::ConstPtr msg) {
        // 获取joy消息后进行处理
        button.at(L1) = msg->buttons[_L1];
        button.at(R1) = msg->buttons[_R1];
        button.at(L2) = msg->buttons[_L2];
        button.at(R2) = msg->buttons[_R2];
        button.at(button1) = msg->buttons[_button1];
        button.at(button2) = msg->buttons[_button2];
        button.at(button3) = msg->buttons[_button3];
        button.at(button4) = msg->buttons[_button4];
        button.at(left)     = msg->axes[_left_right]    > threshold;
        button.at(right)    = msg->axes[_left_right]    < -threshold;
        button.at(up)       = msg->axes[_up_down]       > threshold;
        button.at(down)     = msg->axes[_up_down]       < -threshold;
        button.at(left_axes_left)   = msg->axes[_left_axes_right_left]  > threshold;
        button.at(left_axes_right)  = msg->axes[_left_axes_right_left]  < -threshold;
        button.at(left_axes_up)     = msg->axes[_left_axes_up_down]     > threshold;
        button.at(left_axes_down)   = msg->axes[_left_axes_up_down]     < -threshold;
        button.at(right_axes_left)  = msg->axes[_right_axes_left_right] > threshold;
        button.at(right_axes_right) = msg->axes[_right_axes_left_right] < -threshold;
        button.at(right_axes_up)    = msg->axes[_right_axes_up_down]    > threshold;
        button.at(right_axes_down)  = msg->axes[_right_axes_up_down]    < -threshold;
        button.at(left_axes_button) = msg->buttons[_left_axes_button];
        button.at(right_axes_button) = msg->buttons[_right_axes_button];

        axes.at(up_down) = msg->axes[_up_down];
        axes.at(left_right) = msg->axes[_left_right];
        axes.at(left_axes_up_down) = msg->axes[_left_axes_up_down];
        axes.at(left_axes_left_right) = msg->axes[_left_axes_right_left];
        axes.at(right_axes_up_down) = msg->axes[_right_axes_up_down];
        axes.at(right_axes_left_right) = msg->axes[_right_axes_left_right];
        axes.at(right_button_up_down) = msg->buttons[_button1] - msg->buttons[_button3];
        axes.at(right_button_left_right) = msg->buttons[_button4] - msg->buttons[_button2];
    }
    bool askForButton(int name) {
        // 读取数据
        if (name < 0 || name >= button_size) {
            return false;
        }

        return button.at(name);
    }

    double askForAxes(int name) {
        if (name == 0 || name >= axes_size || name <= -axes_size) {
            return 0.0;
        }

        if (name > 0) {
            return axes.at(name);
        } else {
            return - axes.at(-name);
        }
    }
protected:
    // 以下为按键在joy的消息中的位置
    // 请在更换手柄后更改以下数据
    int _L1;
    int _L2;
    int _R1;
    int _R2;
    int _button1;
    int _button2;
    int _button3;
    int _button4;
    int _up_down;                  //1.0 上 -1.0 下
    int _left_right;               //1.0 左 -1.0 右
    int _left_axes_up_down;        //1.0 上 -1.0 下
    int _left_axes_right_left;     //1.0 左 -1.0 右
    int _left_axes_button;
    int _right_axes_up_down;       //1.0 上 -1.0 下
    int _right_axes_left_right;    //1.0 左 -1.0 右
    int _right_axes_button;
    ::std::vector<bool> button;
    ::std::vector<double> axes;
};

class beitong: public explorerJoyExplainer {
private:
    const double threshold = 0.1;
public:
    beitong() : button(button_size, false), axes(axes_size, 0.0), explorerJoyExplainer() {
        _L1 = 4;
        _L2 = 2;
        _R1 = 5;
        _R2 = 5;
        _button1 = 3;
        _button2 = 1;
        _button3 = 0;
        _button4 = 2;
        _up_down = 7;                  //1.0 上 -1.0 下
        _left_right = 6;               //1.0 左 -1.0 右
        _left_axes_up_down = 1;        //1.0 上 -1.0 下
        _left_axes_right_left = 0;     //1.0 左 -1.0 右
        _left_axes_button = 9;
        _right_axes_up_down = 4;       //1.0 上 -1.0 下
        _right_axes_left_right = 3;    //1.0 左 -1.0 右
        _right_axes_button = 10;
    }
    virtual ~beitong() {};
    static int getButtonListSize() {
        return 11;
    }
    static int getAxesListSize() {
        return 8;
    }


    void getMessage(sensor_msgs::Joy::ConstPtr msg) {
        // 获取joy消息后进行处理
        button.at(L1) = msg->buttons[_L1];
        button.at(R1) = msg->buttons[_R1];
        button.at(L2) = msg->axes[_L2]  < -threshold;
        button.at(R2) = msg->axes[_R2]  < -threshold;
        button.at(button1) = msg->buttons[_button1];
        button.at(button2) = msg->buttons[_button2];
        button.at(button3) = msg->buttons[_button3];
        button.at(button4) = msg->buttons[_button4];
        button.at(left)     = msg->axes[_left_right]    > threshold;
        button.at(right)    = msg->axes[_left_right]    < -threshold;
        button.at(up)       = msg->axes[_up_down]       > threshold;
        button.at(down)     = msg->axes[_up_down]       < -threshold;
        button.at(left_axes_left)   = msg->axes[_left_axes_right_left]  > threshold;
        button.at(left_axes_right)  = msg->axes[_left_axes_right_left]  < -threshold;
        button.at(left_axes_up)     = msg->axes[_left_axes_up_down]     > threshold;
        button.at(left_axes_down)   = msg->axes[_left_axes_up_down]     < -threshold;
        button.at(right_axes_left)  = msg->axes[_right_axes_left_right] > threshold;
        button.at(right_axes_right) = msg->axes[_right_axes_left_right] < -threshold;
        button.at(right_axes_up)    = msg->axes[_right_axes_up_down]    > threshold;
        button.at(right_axes_down)  = msg->axes[_right_axes_up_down]    < -threshold;
        button.at(left_axes_button) = msg->buttons[_left_axes_button];
        button.at(right_axes_button) = msg->buttons[_right_axes_button];

        axes.at(up_down) = msg->axes[_up_down];
        axes.at(left_right) = msg->axes[_left_right];
        axes.at(left_axes_up_down) = msg->axes[_left_axes_up_down];
        axes.at(left_axes_left_right) = msg->axes[_left_axes_right_left];
        axes.at(right_axes_up_down) = msg->axes[_right_axes_up_down];
        axes.at(right_axes_left_right) = msg->axes[_right_axes_left_right];
        axes.at(right_button_up_down) = msg->buttons[_button1] - msg->buttons[_button3];
        axes.at(right_button_left_right) = msg->buttons[_button4] - msg->buttons[_button2];
    }
    bool askForButton(int name) {
        // 读取数据
        if (name < 0 || name >= button_size) {
            return false;
        }

        return button.at(name);
    }

    double askForAxes(int name) {
        if (name == 0 || name >= axes_size || name <= -axes_size) {
            return 0.0;
        }

        if (name > 0) {
            return axes.at(name);
        } else {
            return - axes.at(-name);
        }
    }
protected:
    // 以下为按键在joy的消息中的位置
    // 请在更换手柄后更改以下数据
    int _L1;
    int _L2;
    int _R1;
    int _R2;
    int _button1;
    int _button2;
    int _button3;
    int _button4;
    int _up_down;                  //1.0 上 -1.0 下
    int _left_right;               //1.0 左 -1.0 右
    int _left_axes_up_down;        //1.0 上 -1.0 下
    int _left_axes_right_left;     //1.0 左 -1.0 右
    int _left_axes_button;
    int _right_axes_up_down;       //1.0 上 -1.0 下
    int _right_axes_left_right;    //1.0 左 -1.0 右
    int _right_axes_button;
    ::std::vector<bool> button;
    ::std::vector<double> axes;
};

class autoJoyExplainer : public explorerJoyExplainer {
private:
    explorerJoyExplainer *explainer;
public:
    autoJoyExplainer() : explorerJoyExplainer() {
        explainer = new beitong();
    }
    virtual ~autoJoyExplainer() {
        if (explainer != NULL) {
            delete explainer;
        }
    };
    int getButtonListSize() {
        return explainer->getButtonListSize();
    }
    int getAxesListSize() {
        return explainer->getAxesListSize();
    }


    void getMessage(sensor_msgs::Joy::ConstPtr joy) {
        if (joy->buttons.size() != explainer->getButtonListSize() || joy->axes.size() != explainer->getAxesListSize()) {
            if (joy->buttons.size() == saiTaiKe::getButtonListSize() && joy->axes.size() == saiTaiKe::getAxesListSize()) {
                delete explainer;
                explainer = NULL;
                explainer = new saiTaiKe();
                // ROS_INFO("chose saitaike");
            } else if (joy->buttons.size() == beitong::getButtonListSize() && joy->axes.size() == beitong::getAxesListSize()) {
                delete explainer;
                explainer = NULL;
                explainer = new beitong();
                // ROS_INFO("chose beitong");
            }else if (//joy->buttons.size() == shoubing::getButtonListSize() && 
            joy->axes.size() == shoubing::getAxesListSize()){
                delete explainer;
                explainer = NULL;
                explainer = new shoubing();
            }else{
                delete explainer;
                explainer = NULL;
                return;
            }
        }

        explainer->getMessage(joy);
    }
    bool askForButton(int name) {
        return explainer->askForButton(name);
    }

    double askForAxes(int name) {
        return explainer->askForAxes(name);
    }
};
#endif //explorerJoyExplainer_HPP




