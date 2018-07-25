#include "explorer_serial_protocol/SerialNode.h"

int main(int argc , char **argv)
{
    ros::init(argc , argv , "explorer_serial_protocol_node");
    ros::NodeHandle node;
    ::explorer_serial_protocol::ExplorerSerialProtocol a(node);
    ros::spin();
    return 0;
}
