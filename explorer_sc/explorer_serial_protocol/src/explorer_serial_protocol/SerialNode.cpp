#include "explorer_serial_protocol/SerialNode.h"
#include <sstream>
using ::std::cout;
using ::std::endl;
namespace explorer_serial_protocol {

ExplorerSerialProtocol::ExplorerSerialProtocol(ros::NodeHandle _nh)
    : node(_nh), node_private("~")  {
    this->serial_ptr = ::boost::make_shared<::serial::SerialPort>();

    ::serial::SerialParams param;
    node_private.param("serial_file_name", param.serialPort, ::std::string("/dev/explorer_serial"));
    this->serial_ptr->setCallbackFunc(::boost::bind(&ExplorerSerialProtocol::serial_get, this, _1));
    this->segment_get = node.subscribe("/explorer_serial", 1000, &ExplorerSerialProtocol::serial_send, this);
    this->serial_ptr->startThread();
    ROS_INFO("node 09") ;

    try {
        // 创建线程
        this->output_therad = ::boost::make_shared<::boost::thread>(::boost::thread(boost::bind(&ExplorerSerialProtocol::message_send, this)));
    } catch (std::exception &e) {
        cout << "Failed to create thread !" << endl;
        cout << "Error Info: " << e.what() << endl;
    }
}

ExplorerSerialProtocol::~ExplorerSerialProtocol() {
    this->node.shutdown();
}

void ExplorerSerialProtocol::serial_send(const ::explorer_msgs::explorer_agreement::ConstPtr &ptr) {
    ::serial::ByteVector current_data;
    current_data.resize(13, 0xDD);
    //ROS_INFO("write over!") ;

    for (int i = 0; i < ptr->msg.size(); ++i) {
        current_data[i] = ptr->msg.at(i);
    }

    serial_ptr->writeRaw(current_data);
}

void ExplorerSerialProtocol::serial_get(::serial::pByteVector stream) {
    bool need_start = output_buffer.size() < 12;
    // 将所得数据输入缓冲区
    
    for (uint8_t byte : *stream) {
        output_buffer.push(byte);
    }
    
    if (need_start){
            try {
            this->output_therad = ::boost::make_shared<::boost::thread>(::boost::thread(boost::bind(&ExplorerSerialProtocol::message_send, this)));
            
    } catch (std::exception &e) {
        cout << "Failed to create thread !" << endl;
        cout << "Error Info: " << e.what() << endl;
    }
    }

    return;
}

void ExplorerSerialProtocol::message_send() {
    cout << "message send thread start" << endl;

    while (output_buffer.size() > 11) {
        while (output_buffer.size() > max_buffer_size) {
            output_buffer.pop();
            ROS_ERROR_STREAM("buffer size out of range");
        }

        if (output_buffer.size() < 11) {
            return;
        }

        if (output_buffer.front() != 0x08) {
            output_buffer.pop();
            ROS_ERROR_STREAM("message broke");
        } else {
            output_buffer.pop();

            // 避免数据报断开
            while (output_buffer.empty()) {}

            int id = output_buffer.front();
            output_buffer.pop();

            // 避免数据报断开
            while (output_buffer.empty()) {}

            if (output_buffer.front() != 0xE0) {
                continue;
            } else {
                output_buffer.pop();
            }

            uint8_t *byte_message;
            byte_message = (uint8_t *)data;

            for (int i = 0; i < sizeof(float) * 2; ++i) {
                // 避免数据报断开
                while (output_buffer.empty()) {}

                byte_message[i] = output_buffer.front();
                output_buffer.pop();
            }
                //先在此处进行测试，看是否能收到id
                //可能usb口不对应
            ROS_INFO_STREAM("id = " << id << "\tdata1 = " << data[0] << "\tdata2 = " << data[1] << "\tqueue length" << output_buffer.size());

            if (id >= publisher_register.size()) {
                publisher_register.resize(id + 2, false);
                segment_send.resize(id + 2);
                ::std::stringstream publisher_name_stream;
                publisher_name_stream << "/explorer_serial_data/" << id;
                segment_send.at(id) = node.advertise<explorer_msgs::explorer_low_level_data>(publisher_name_stream.str(), 10);
                publisher_register.at(id) = true;
            } else if (!publisher_register.at(id)) {
                ::std::stringstream publisher_name_stream;
                publisher_name_stream << "/explorer_serial_data/" << id;//将底层数据通过/explorer_serial_data/id的pub出来
                segment_send.at(id) = node.advertise<explorer_msgs::explorer_low_level_data>(publisher_name_stream.str(), 10);
                publisher_register.at(id) = true;
            }

            explorer_msgs::explorer_low_level_data current_data ;
            current_data.low_level_id = id ;
            current_data.can_serial_data_1 = data[0];
            current_data.can_serial_data_2 = data[1];
            segment_send.at(id).publish(current_data);
        }
    }
}
}








