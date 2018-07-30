/*
 * SerialPort.h
 *
 *  Created on: 2012-4-8
 *      Author: startar
 ************************
 * Last Change: 2017-8-7
 *      Author: TheNext
 */

#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <ros/ros.h>
#include <inttypes.h>
#include <vector>
#include <queue>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

namespace serial {

// 串口通信选项

class SerialParams {
public:
    ::std::string                               serialPort;     // 串口的设备文件
    ::boost::asio::serial_port::baud_rate       baudRate;       // 波特率
    ::boost::asio::serial_port::flow_control    flowControl;    // 流控
    ::boost::asio::serial_port::parity          parity;         // 校验位
    ::boost::asio::serial_port::stop_bits       stopBits;       // 停止位
    SerialParams() :
        serialPort("/dev/explorer_serial"),
        baudRate(115200),
        flowControl((::boost::asio::serial_port::flow_control::type)0),
        parity((::boost::asio::serial_port::parity::type)0),
        stopBits((::boost::asio::serial_port::stop_bits::type)0) {
        // PS: 这里的强制转换不是什么好习惯... 我只是图省事
    }
    SerialParams(
        ::std::string _serialPort,
        unsigned int _baudRate,
        unsigned int _flowControl,
        unsigned int _parity,
        unsigned int _stopBits
    ) :
        serialPort(_serialPort),
        baudRate(_baudRate),
        flowControl(::boost::asio::serial_port::flow_control::type(_flowControl)),
        parity(::boost::asio::serial_port::parity::type(_parity)),
        stopBits(::boost::asio::serial_port::stop_bits::type(_stopBits)) {}
};

typedef ::std::vector<uint8_t> ByteVector;
typedef ::boost::shared_ptr<ByteVector> pByteVector;


class SerialPort {
/** \brief explorer robot usb can reader and sender
 *
 * This port reader and send only offer byte send and publish 
 * 
 * TODO : should be use when the can reload
 *
 */
private:
    ::boost::shared_ptr<::boost::asio::io_service>      m_pios;         // io_service对象
    ::boost::shared_ptr<::boost::asio::serial_port>     m_pSerial;      // 串口对象的指针
    ::boost::mutex          m_serialMutex;      // 串口对象的互斥锁. 按照boost官方文档, serial_port对象不是线程安全的, 而读写需要进行多线程操作 故需要此锁

    ::serial::SerialParams  m_serialParams;         // 串口的配置数据

    ByteVector              m_tempBuf;              // 数据读取的临时缓冲区
    ::std::size_t           m_DataBytesRead;        // 数据已经读取的字节数

    ::std::queue<pByteVector>      m_writeQueue;       // 待发送数据的队列
    ::boost::mutex            m_writeQueueMutex;  // 队列的互斥锁

    // c++11中function已经进入std命名空间，两者是否有区别暂时不清楚，这里按照以往代码，推测使用的应当是boost库中的function
    ::boost::function<void(pByteVector)> m_dataCallbackFunc;             // 数据回调函数
    ::boost::function<void()> m_errorCallbackFunc;                      // 错误回调函数

    // 跑io_service::run()的线程
    boost::thread m_thread;

    // 线程的主过程, 主要是在跑io_service::run()
    void mainRun();

    // // 为了方便写的函数
    void start_a_read();
    void start_a_write();

    // async_read_some的Handler
    void readHandler(const ::boost::system::error_code &ec, size_t bytesTransferred);
    // async_write_some的Handler
    void writeHandler(const ::boost::system::error_code &ec);

public:
    SerialPort();
    virtual ~SerialPort();


    bool startThread();                                     // 启动线程
    bool stopThread();                                      // 停止线程

    // 设置收到数据之后的回调函数
    // c++11中function已经进入std命名空间，两者是否有区别暂时不清楚，这里按照以往代码，推测使用的应当是boost库中的function
    void setCallbackFunc(const ::boost::function<void(pByteVector)> &func);

    // 向串口中直接写入一串数据
    bool writeRaw(const ByteVector &rawData);
};

} /* namespace XM_SerialNode */

#endif /* SERIALPORT_H_ */
