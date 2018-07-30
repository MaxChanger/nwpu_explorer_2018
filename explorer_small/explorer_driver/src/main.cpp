#include "explorer_driver.h"

int main(int argc , char **argv) {
    ros::init(argc , argv , "explorer_driver");
    ros::NodeHandle current_node;
    RobotDriver explorer_dirver(current_node);
    ros::spin();
}

