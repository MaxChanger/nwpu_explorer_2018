/*
 * SerialPort.h
 *
 *  Created on: 2012-4-8
 *      Author: startar
 ************************
 * Last Change: 2017-8-7
 *      Author: TheNext
 */

#include "explorer_serial_protocol/SerialPort.h"
#include <vector>
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>
using ::std::cout;
using ::std::endl;

namespace serial {

SerialPort::SerialPort():
    m_DataBytesRead(0),
    m_serialParams() {
    cout << "SerialPort Object created!" << endl;

    m_tempBuf.resize(1024, 0);
    m_DataBytesRead =  0 ;
}

SerialPort::~SerialPort() {
    std::cout << "exit" << endl;
    this->stopThread();
    m_thread.join();
}

void SerialPort::setCallbackFunc(const ::boost::function<void(pByteVector)> &func) {
    try {
        m_dataCallbackFunc = func;
    } catch (std::exception &e) {
        ROS_ERROR_STREAM("err find in serial port" << e.what());
    }
}

void SerialPort::mainRun() {
    cout << "SerialPort mainThread STARTED!" << endl;
    start_a_read();

    // 开始跑io_service::run()
    m_pios->run();
}
void SerialPort::start_a_read() {

    ::boost::mutex::scoped_lock lock(m_serialMutex);

    // 启动一次异步读
    m_pSerial->async_read_some(::boost::asio::buffer(m_tempBuf),
                               ::boost::bind(&SerialPort::readHandler,
                                             this,
                                             ::boost::asio::placeholders::error,
                                             ::boost::asio::placeholders::bytes_transferred
                                            ));
}

void SerialPort::readHandler(const ::boost::system::error_code &ec, size_t bytesTransferred) {

    if (bytesTransferred >= m_tempBuf.size()) {
        ROS_ERROR_STREAM("err in SerialPort the tempBuf is over write");
    } else if (ec) {
        ROS_ERROR_STREAM("err in SerialPort as:" << ec);
    } else {
        pByteVector callBack(new ByteVector);
        // 提前准备空间，提高效率
        callBack->reserve(bytesTransferred + 11);

        for (int i = 0; i < bytesTransferred; ++i) {
            callBack->push_back(m_tempBuf.at(i));
        }
        //　这里直接调用m_dataCallbackFunc一直失败，原因不明
        m_dataCallbackFunc(callBack);
    }

    start_a_read();

}
void SerialPort::writeHandler(const ::boost::system::error_code &ec) {
    if (ec) {
        // TODO: 报错
    }

    {
        ::boost::mutex::scoped_lock lock(m_writeQueueMutex);
        m_writeQueue.pop();
        //::std::cout << "have writed!!!" << ::std::endl ;

        if (m_writeQueue.empty() == false) {
            start_a_write();
        }
    }
}
void SerialPort::start_a_write() {
    ::boost::mutex::scoped_lock lock(m_serialMutex);

    // 启动一次异步写
    async_write(*m_pSerial, ::boost::asio::buffer(*(m_writeQueue.front())),
                bind(&SerialPort::writeHandler, this, ::boost::asio::placeholders::error));
}
bool SerialPort::writeRaw(const ByteVector &rawData) {
    ::boost::mutex::scoped_lock lock(m_writeQueueMutex);    // 上锁

    bool writeIdle = m_writeQueue.empty();          // 检测当前队列是否为空
    pByteVector data(new ByteVector(rawData));
    m_writeQueue.push(data);                        // 将数据拷贝一份, 并加入队列里去

    if (writeIdle) {
        start_a_write();    // 如果没有在写数据, 则启动一次异步写过程
    }

    return true;
}

bool SerialPort::startThread() {
    cout << "SerialPort::startThread() called!" << endl;

    // 创建io_service对象
    m_pios = ::boost::make_shared<::boost::asio::io_service>();

    try {
        // 创建一个serial_port对象, 替换掉原来的
        m_pSerial = ::boost::make_shared<::boost::asio::serial_port>(::boost::ref(*m_pios), m_serialParams.serialPort);

        // 设置串口通信参数
        m_pSerial->set_option(m_serialParams.baudRate);
        m_pSerial->set_option(m_serialParams.flowControl);
        m_pSerial->set_option(m_serialParams.parity);
        m_pSerial->set_option(m_serialParams.stopBits);
        m_pSerial->set_option(::boost::asio::serial_port::character_size(8));
    } catch (::std::exception &e) {
        ROS_ERROR_STREAM("Failed to open serial port !");
        ROS_ERROR_STREAM("Error Info: " << e.what());
        return false;
    }

    try {
        // 创建线程
        m_thread = boost::thread(boost::bind(&SerialPort::mainRun, this));
    } catch (std::exception &e) {
        cout << "Failed to create thread !" << endl;
        cout << "Error Info: " << e.what() << endl;
        return false;
    }

    return true;
}
bool SerialPort::stopThread() {
    m_pios->stop();
    return true;
}
}/* namespace serial */
