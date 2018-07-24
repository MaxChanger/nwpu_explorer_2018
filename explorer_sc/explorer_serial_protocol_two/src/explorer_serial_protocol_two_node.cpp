#include "explorer_serial_protocol_two/SerialNode.h"

int main(int argc , char **argv)
{
    ros::init(argc , argv , "explorer_serial_protocol_two_node");
    ros::NodeHandle node;
    ::explorer_serial_protocol_two::ExplorerSerialProtocol a(node);
    ros::spin();
    return 0;
}
