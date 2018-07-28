#ifndef QUSBCAM_H
#define QUSBCAM_H

#ifndef Q_MOC_RUN //moc预处理器 将不需要qt的头文件放到这里

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <string>

#endif

#include <QThread>
#include <QImage>
#include <QDebug>


namespace qusbcam {

class QUsbcam : public QThread
{
    Q_OBJECT //在加入Q_OBJECT以后 h头文件需要在cmake里面file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/qdude/*.hpp include/qdude/qusbcam.h)头文件中加入 进行头文件moc预处理
    public:
        QUsbcam(int argc, char** argv);
        QUsbcam();
//        virtual ~QUsbcam();
        ~QUsbcam();

        bool init(QString topic_name);
        void run();
        void QUsbcamCallback(const sensor_msgs::ImageConstPtr &msg);

        QString topic_name_;
        QImage swappedimg;  //最终接口 因为一开始收到的img是红蓝通道不对的 所以最后还是把红蓝通道交换得到的正常图像
    
    Q_SIGNALS:
        void UsbcamCallback();

    private:
        int init_argc;
        char** init_argv;
        QImage img;
        ros::Subscriber image_subscriber;

};

}

#endif // QUSBCAM_H
