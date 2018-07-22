#include "explorer_joy_control.h"

int main(int argc , char **argv) {
    ros::init(argc , argv, "joy_control") ;
    ros::NodeHandle node;
    ExplorerTeleop explorer_teleop;
    ros::spin();
}

