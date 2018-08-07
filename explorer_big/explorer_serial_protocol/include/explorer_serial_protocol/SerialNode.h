#include "explorer_serial_protocol/SerialPort.h"
#include <explorer_msgs/explorer_agreement.h>
#include <explorer_msgs/explorer_low_level_data.h>

namespace explorer_serial_protocol {
class ExplorerSerialProtocol {
public:
    ExplorerSerialProtocol(ros::NodeHandle);
    virtual ~ExplorerSerialProtocol();
    const static int max_buffer_size = 1000;
protected:
    void serial_get(::serial::pByteVector);
    void serial_send(const ::explorer_msgs::explorer_agreement::ConstPtr&);
    // 通过单独的线程实现数据读写
    void message_send();
private:
    ::ros::NodeHandle   node;               // 所使用的ros节点
    ::ros::NodeHandle   node_private;       // 私有节点
    ::ros::Subscriber   segment_get;        // 接收报文段
    ::std::vector<::ros::Publisher> segment_send;// 发送报文段
    ::std::vector<bool> publisher_register; // 发送器生成判断
    ::boost::shared_ptr<::serial::SerialPort> serial_ptr;

    ::boost::shared_ptr<::boost::thread> output_therad;          // 缓冲区数据处理线程
    ::boost::mutex output_operator_mutex;   // 缓冲区处理互斥锁

    ::std::queue<uint8_t> output_buffer;    // 发送数据缓冲

    float data[2];
};
}
