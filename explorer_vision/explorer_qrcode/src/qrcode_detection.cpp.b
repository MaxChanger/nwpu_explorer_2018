
#include <explorer_qrcode/qrcode_detection.h>
#include <hector_worldmodel_msgs/ImagePercept.h>
#include <std_msgs/String.h>
#include <ros/ros.h> 
#include <cv.h>
//#include<opencv2/core/core.hpp>
//#include<opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <zbar.h>

#include <sensor_msgs/CompressedImage.h>

//zxing 头文件
#include "ImageReaderSource.h"
#include <zxing/common/Counted.h>
#include <zxing/Binarizer.h>
#include <zxing/MultiFormatReader.h>
#include <zxing/Result.h>
#include <zxing/ReaderException.h>
#include <zxing/common/GlobalHistogramBinarizer.h>
#include <zxing/common/HybridBinarizer.h>
#include <exception>
#include <zxing/Exception.h>
#include <zxing/common/IllegalArgumentException.h>
#include <zxing/BinaryBitmap.h>
#include <zxing/DecodeHints.h>

#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/multi/qrcode/QRCodeMultiReader.h>
#include <zxing/multi/ByQuadrantReader.h>
#include <zxing/multi/MultipleBarcodeReader.h>
#include <zxing/multi/GenericMultipleBarcodeReader.h>
#include <zxing/MatSource.h>


using namespace zbar;
using namespace std;
using namespace cv;
using namespace zxing;
using namespace zxing::multi;
using namespace zxing::qrcode;


namespace ws_qrcode_detection
{
bool flag_l=false;
int flag_r=false;
sensor_msgs::CameraInfoConstPtr right;
sensor_msgs::CameraInfoConstPtr left;


//构造函数
qrcode_detection_impl::qrcode_detection_impl(ros::NodeHandle nh, ros::NodeHandle priv_nh)
    : nh_(nh), image_transport_(nh_), listener_(0)
{
  scanner_ = new zbar::ImageScanner;

  scanner_->set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

  ROS_ERROR("In qrcode_detection.cpp    impl()");

  rotation_image_size_ = 2;
  priv_nh.getParam("rotation_source_frame", rotation_source_frame_id_);
  priv_nh.getParam("rotation_target_frame", rotation_target_frame_id_);
  priv_nh.getParam("rotation_image_size", rotation_image_size_);

  priv_nh.param("pub_qrcode_topic", pub_qrcode_topic_, std::string("image_percept"));

  priv_nh.param("pub_image_qrcode_topic", pub_image_qrcode_topic_, std::string("image/qrcode"));

  priv_nh.param("sub_image_qrcode_topic_camera1", sub_image_qrcode_topic_camera1_, std::string("/camera1/rgb/image_raw"));

  //myadd
  priv_nh.param("sub_image_qrcode_topic_camera2", sub_image_qrcode_topic_camera2_, std::string("/camera2/rgb/image_raw"));
  //
  priv_nh.param("camera_info1", camera_info1_, std::string("/camera_right/camera_info"));
  priv_nh.param("camera_info2", camera_info2_, std::string("/camera_left/camera_info"));
  ROS_ERROR("%s,%s",camera_info1_,camera_info2_);

  //myadd
  // priv_nh.param("sub_image_qrcode_topic_front_left_camera", sub_image_qrcode_topic_front_left_camera_, std::string("/front_left_camera/image_raw"));
  // priv_nh.param("sub_image_qrcode_topic_front_right_camera", sub_image_qrcode_topic_front_right_camera_, std::string("/front_right_camera/image_raw"));
  // priv_nh.param("sub_image_qrcode_topic_back_left_camera", sub_image_qrcode_topic_back_left_camera_, std::string("/back_left_camera/image_raw"));
  // priv_nh.param("sub_image_qrcode_topic_back_right_camera", sub_image_qrcode_topic_back_right_camera_, std::string("/back_right_camera/image_raw"));
  //
  priv_nh.param("pub_find_qrcode_topic", pub_find_qrcode_topic_, std::string("/find_qrcode"));

  percept_publisher_ = nh_.advertise<hector_worldmodel_msgs::ImagePercept>(pub_qrcode_topic_, 10);
  qrcode_publisher=nh_.advertise<hector_worldmodel_msgs::ImagePercept>("qrcode_info",10);
  qrcode_image_publisher_ = image_transport_.advertiseCamera(pub_image_qrcode_topic_, 10);

  //camera_subscriber_camera1_ = image_transport_.subscribeCamera(sub_image_qrcode_topic_camera1_, 10, &qrcode_detection_impl::imageCallback, this);
  //camera_subscriber_camera2_ = image_transport_.subscribeCamera(sub_image_qrcode_topic_camera2_, 10, &qrcode_detection_impl::imageCallback, this);
    sub_camera1=nh_.subscribe(sub_image_qrcode_topic_camera1_,10,&qrcode_detection_impl::imageCallback, this);
    sub_camera2=nh_.subscribe(sub_image_qrcode_topic_camera2_,10,&qrcode_detection_impl::imageCallback, this);
    info_camera1=nh_.subscribe(camera_info1_,10,&qrcode_detection_impl::rightinfoCallback,this);
    info_camera2=nh_.subscribe(camera_info2_,10,&qrcode_detection_impl::leftinfoCallback,this);

  // camera_subscriber_front_left_camera_ = image_transport_.subscribeCamera(sub_image_qrcode_topic_front_left_camera_, 10, &qrcode_detection_impl::imageCallback, this);
  // camera_subscriber_front_right_camera_ = image_transport_.subscribeCamera(sub_image_qrcode_topic_front_right_camera_, 10, &qrcode_detection_impl::imageCallback, this);
  // camera_subscriber_back_left_camera_ = image_transport_.subscribeCamera(sub_image_qrcode_topic_back_left_camera_, 10, &qrcode_detection_impl::imageCallback, this);
  // camera_subscriber_back_right_camera_ = image_transport_.subscribeCamera(sub_image_qrcode_topic_back_right_camera_, 10, &qrcode_detection_impl::imageCallback, this);
  //

  find_qrcode_ = nh_.advertise<std_msgs::String>(pub_find_qrcode_topic_, 10);

  if (!rotation_target_frame_id_.empty())
  {
    listener_ = new tf::TransformListener();
    rotated_image_publisher_ = image_transport_.advertiseCamera("image/rotated", 10);
  }

//   ROS_INFO("Successfully initialized the zbar qrcode detector for image %s", camera_subscriber_camera1_.getTopic().c_str());
//   ROS_INFO("Successfully initialized the zbar qrcode detector for image %s", camera_subscriber_camera2_.getTopic().c_str());
  // ROS_INFO("Successfully initialized the zbar qrcode detector for image %s", camera_subscriber_front_left_camera_.getTopic().c_str());
  // ROS_INFO("Successfully initialized the zbar qrcode detector for image %s", camera_subscriber_front_right_camera_.getTopic().c_str());
  // ROS_INFO("Successfully initialized the zbar qrcode detector for image %s", camera_subscriber_back_left_camera_.getTopic().c_str());
  // ROS_INFO("Successfully initialized the zbar qrcode detector for image %s", camera_subscriber_back_right_camera_.getTopic().c_str());
  ROS_INFO("Successfully initialized the zbar qrcode detector for image %s", sub_camera1.getTopic().c_str());
  ROS_INFO("Successfully initialized the zbar qrcode detector for image %s", sub_camera2.getTopic().c_str());
}

qrcode_detection_impl::~qrcode_detection_impl()
{
  delete listener_;
}

void qrcode_detection_impl::leftinfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info){
    if(!flag_l)
    {
        flag_l=true;
        left =camera_info;
        ROS_ERROR("run the left");
    }
}
void qrcode_detection_impl::rightinfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info){
    if(!flag_r)
    {
        flag_r=true;
        right=camera_info;
        ROS_ERROR("run the right");
    }
}

void qrcode_detection_impl::imageCallback(const sensor_msgs::CompressedImageConstPtr& image)
{
    ROS_ERROR("miao miaomia0");
    //if(!flag_r)
    if(!flag_r||!flag_l)
    {
        return;
    }
    //ROS_ERROR("miao miaomia0");
  ROS_ERROR_ONCE("In qrcode_detection.cpp   imageCallback");
  
  cv_bridge::CvImageConstPtr cv_image;
  cv_image = cv_bridge::toCvCopy(image, "mono8");
  cv::Mat rotation_matrix = cv::Mat::eye(2, 3, CV_32FC1);
  double rotation_angle = 0.0;


  hector_worldmodel_msgs::ImagePercept percept;
  percept.header = image->header;
  if(image->header.frame_id==right->header.frame_id)
  {
      percept.camera_info = *right;
     // ROS_ERROR("set the right frame_id");
    }
    else if(image->header.frame_id==left->header.frame_id)
    {
        percept.camera_info=*left;
    }
    else{
        ROS_ERROR("can't cpmpare the frame_id");
        return;
    }
  percept.info.class_id = "qrcode"; // !!!!!!!!!!!!!用QR码来代替victim，发布的消息，只有image_percept不一样!!!!!!!!!!!!
  ROS_INFO("In qrcode %s", percept.info.class_id.c_str());
  percept.info.class_support = 1.0;

  Mat img_org=cv_image->image.clone();
  Mat thre_img;
  int thre=100;
  int ks=15;
  //Mat kernel=getStructuringElement(MORPH_RECT,Size(ks,ks));
  Mat roi;
  int temp_max_x,temp_max_y,temp_min_x,temp_min_y;
  int baseLen=0;
 
        //threshold(img_org,thre_img,thre,255,THRESH_BINARY);
        thre_img=img_org.clone();
        Image zbar(thre_img.cols,thre_img.rows,"Y800",thre_img.data,thre_img.cols*thre_img.rows);
        scanner_->scan(zbar);
         int min_x = 99999999, min_y = 99999999, max_x = 0, max_y = 0;
         bool flag=false;
        for(Image::SymbolIterator symbol=zbar.symbol_begin();symbol!=zbar.symbol_end();++symbol)
        {
            
            string result=symbol->get_data();
                //确定扫到二维码的位置
            ROS_ERROR("qrcode info :%s",result.c_str());
            min_x = 99999999, min_y = 99999999, max_x = 0, max_y = 0;
            for (int i = 0; i < 4; ++i)
            {
                if (symbol->get_location_x(i) > max_x)
                    max_x = symbol->get_location_x(i);
                if (symbol->get_location_x(i) < min_x)
                    min_x = symbol->get_location_x(i);
                if (symbol->get_location_y(i) > max_y)
                    max_y = symbol->get_location_y(i);
                if (symbol->get_location_y(i) < min_y)
                    min_y = symbol->get_location_y(i);
            }
            //baseLen_x<max_x-min_x;
            if(baseLen<max_x-min_x)
                {
                    baseLen=max_x-min_x;
                    temp_max_x=max_x;
                    temp_max_y=max_y;
                    temp_min_x=min_x;
                    temp_min_y=min_y;
                }

            //发射检测到的数据
             //cout<<result<<endl;
            //ROS_ERROR("%d %d %d %d",index[0],index[1],index[2],index[3]);
            rectangle(img_org,Rect(Point(min_x-int(0.1*baseLen)>0?min_x-int(0.1*baseLen):0,min_y-int(0.1*baseLen)>0?min_y-int(0.1*baseLen):0)
            ,Point(max_x+int(0.1*baseLen)<img_org.cols?max_x+int(0.1*baseLen):img_org.cols,max_y+int(0.1*baseLen)<img_org.rows?max_y+int(0.1*baseLen):img_org.rows)),
            Scalar(0),3);
            percept.info.object_id=result.substr(0,5);
            percept.info.object_support=1.0;
            percept.info.name=result;
            //ROS_ERROR("QRcoode info : [%s]", percept.info.name.c_str());
            //cv::Vec3f left_top_corner(avg_x-int(distance_x*0.66)>=0?int(avg_x-distance_x*0.66):0,avg_y-int(distance_y*0.66)>=0?int(avg_y-distance_y*0.66):0, 1.0f);
            //cv::Vec3f right_bottom_corner(avg_x+int(distance_x*0.66)<img.cols?int(avg_x+distance_x*0.66):img.cols,avg_y+int(distance_y*0.66)<img.rows?int(avg_y+distance_y*0.66):img.rows, 1.0f);
            cv::Vec3f left_top_corner(min_x,min_y, 1.0f);
            cv::Vec3f right_bottom_corner(max_x,max_y, 1.0f);
           if (rotation_angle != 0.0)
            {
            ROS_ERROR("Non-zero rotations are currently not supported!");
            continue;
            }
            percept.x = (left_top_corner(0) + right_bottom_corner(0)) / 2;
            percept.y = (left_top_corner(1) + right_bottom_corner(1)) / 2;
            percept.width = right_bottom_corner(0) - left_top_corner(0);
            percept.height = right_bottom_corner(1) - left_top_corner(1);
            percept_publisher_.publish(percept);
            //qrcode_publisher.publish(percept);
            flag=true;
        }
        thre+=20;
        if(flag)
        {
            Point roi_left_up(temp_max_x<img_org.cols?temp_max_x:img_org.cols,temp_min_y-baseLen>0?temp_min_y-baseLen:0);
            Point roi_right_down(temp_max_x+2*baseLen<img_org.cols?temp_max_x+2*baseLen:img_org.cols, temp_max_y+baseLen>img_org.rows?img_org.rows:temp_max_y+baseLen);
            //Mat gray=img_org(Rect(roi_left_up,roi_right_down));
            Mat gray=img_org(Rect(roi_left_up,roi_right_down));
            //rectangle(img_org,Rect(roi_left_up,roi_right_down),Scalar(0));
            // ROS_ERROR("%d",baseLen);
            // ROS_ERROR("%d,%d,%d,%d",roi_left_up.x,roi_left_up.y,roi_right_down.x,roi_right_down.y);
            // ROS_ERROR("%d,%d",img_org.cols,img_org.rows);
            // ROS_ERROR("in the code");
            if(gray.empty())
            ROS_ERROR("error");
            int baseX=temp_max_x+int(0.5*baseLen)>=img_org.cols?img_org.cols:temp_max_x+int(0.5*baseLen);
            int baseY=temp_min_y-baseLen>0?temp_min_y-baseLen:0;
            
            //zxing扫描开始
            //Mat gray=img_org.clone();
            Mat temp;
            threshold(gray,temp,50,255,THRESH_BINARY);
            int roi_thre=22;
            Mat kernel=getStructuringElement(MORPH_RECT,Size(roi_thre,roi_thre));
            erode(temp,temp,kernel);
            vector< vector<Point> > contour;
            findContours(temp,contour,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
            for(int x=1;x<contour.size();x++)
            {
                int all_x=0;
                int all_y=0;
                for(int y=0;y<contour[x].size();y++)
                {
                all_x+=contour[x][y].x;
                all_y+=contour[x][y].y;
                }
                int avg_x=int(all_x/contour[x].size());
                int avg_y=int(all_y/contour[x].size());
                int distance_x=(abs(avg_x-contour[x][0].x))*1.5;
                int distance_y=(abs(avg_y-contour[x][0].y))*1.5;
                Point left_up(avg_x-distance_x>=0?avg_x-distance_x:0,avg_y-distance_y>=0?avg_y-distance_y:0);
                Point right_down(avg_x+distance_x<gray.cols?avg_x+distance_x:gray.cols,avg_y+distance_y<gray.rows?avg_y+distance_y:gray.rows);
                int len_x=right_down.x-left_up.x;
                int len_y=right_down.y-left_up.y;
                if(len_x*len_y<1000||len_x>int(1.5*len_y)||len_x<int(0.66*len_y))
                    continue;
                //rectangle(gray,Rect(left_up,right_down),Scalar(0));
                Mat roi=gray(Rect(left_up.x,left_up.y,right_down.x-left_up.x,right_down.y-left_up.y));
                try
                {
                    Ref<LuminanceSource> source = MatSource::create(roi);
                    MultiFormatReader delegate;
                    //GenericMultipleBarcodeReader reader(delegate);
                    Ref<Reader> reader;
                    reader.reset(new MultiFormatReader);
                    Ref<Binarizer> binarizer(new GlobalHistogramBinarizer(source));
                    Ref<BinaryBitmap> bitmap(new BinaryBitmap(binarizer));
                    Ref<Result> result=reader->decode(bitmap, DecodeHints(DecodeHints::TRYHARDER_HINT));
                    int resultPointCount = result->getResultPoints()->size();
                    if(resultPointCount==4)
                    {
                        //cout<<result->getText()->getText()<<endl;
                        string text=result->getText()->getText();
                        ROS_ERROR("zxing %s",text.c_str());
                        percept.info.object_id=text.substr(0,5);
                        percept.info.object_support=1.0;
                        percept.info.name=text.substr(0,5);
                        ROS_ERROR("maybe 2  %d",resultPointCount);
                        int min_x = 99999999, min_y = 99999999, max_x = 0, max_y = 0;

                        for (int i = 0; i < 4; ++i)
                        {
                            
                            if (result->getResultPoints()[i]->getX() > max_x)
                                max_x = int(result->getResultPoints()[i]->getX());
                            if (result->getResultPoints()[i]->getX() < min_x)
                                min_x = int(result->getResultPoints()[i]->getX());
                            if (result->getResultPoints()[i]->getY()> max_y)
                                max_y = int(result->getResultPoints()[i]->getY());
                            if (result->getResultPoints()[i]->getX() < min_y)
                                min_y = int(result->getResultPoints()[i]->getX());
                        }
                        ROS_ERROR("maybe 1");
                        cv::Vec3f left_top_corner(baseX+min_x,baseY+min_y, 1.0f);
                        cv::Vec3f right_bottom_corner(baseX+max_x,baseY+max_y, 1.0f);
                        // cv::Vec3f left_top_corner(min_x,min_y, 1.0f);
                        // cv::Vec3f right_bottom_corner(max_x,max_y, 1.0f);
                        if (rotation_angle != 0.0)
                        {
                            ROS_ERROR("Non-zero rotations are currently not supported!");
                            continue;
                        }
                        percept.x = (left_top_corner(0) + right_bottom_corner(0)) / 2;
                        percept.y = (left_top_corner(1) + right_bottom_corner(1)) / 2;
                        percept.width = right_bottom_corner(0) - left_top_corner(0);
                        percept.height = right_bottom_corner(1) - left_top_corner(1);
                        percept_publisher_.publish(percept);
                        ROS_ERROR("fin");
                    }
                    
                }catch (const ReaderException& e) {
                    //cerr << e.what() << " (ignoring)" << endl;
                } catch (const zxing::IllegalArgumentException& e) {
                    //cerr << e.what() << " (ignoring)" << endl;
                } catch (const zxing::Exception& e) {
                    //cerr << e.what() << " (ignoring)" << endl;
                } catch (const std::exception& e) {
                    //cerr << e.what() << " (ignoring)" << endl;
                }
            }
        }
    
    //抠图
    



        

//我只是一个平凡的视频输出器，这个主要用于调试，不用管
    if (qrcode_image_publisher_.getNumSubscribers() > 0&&image->header.frame_id=="camera_right")
    {
      try
      {
        //cv::Rect rect(cv::Point2i(std::max(min_x, 0), std::max(min_y, 0)), cv::Point2i(std::min(max_x, cv_image->image.cols), std::min(max_y, cv_image->image.rows)));
        cv_bridge::CvImagePtr qrcode_cv(new cv_bridge::CvImage(*cv_image));
        //qrcode_cv->image = cv_image->image(rect);
        qrcode_cv->image=img_org;

        sensor_msgs::Image qrcode_image;
        qrcode_cv->toImageMsg(qrcode_image);
        qrcode_image_publisher_.publish(qrcode_image, percept.camera_info);
      }
      catch (cv::Exception &e)
      {
        ROS_ERROR("cv::Exception: %s", e.what());
      }
    }
  
}



}
