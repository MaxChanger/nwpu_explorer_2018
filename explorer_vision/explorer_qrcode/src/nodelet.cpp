
#include <explorer_qrcode/qrcode_detection.h>
//#include <nodelet/nodelet.h>
//#include <pluginlib/class_list_macros.h>

namespace ws_qrcode_detection
{

class qrcode_detection
// : public nodelet::Nodelet
{
private:
  qrcode_detection_impl *impl;

public:
  qrcode_detection() : impl(0) {}
  virtual ~qrcode_detection() { delete impl; }

private:
  void onInit()
  {
    //impl = new qrcode_detection_impl(getNodeHandle(), getPrivateNodeHandle());
    impl = new qrcode_detection_impl(ros::NodeHandle(), ros::NodeHandle("~"));
  }
};

} // namespace hector_qrcode_detection

//PLUGINLIB_EXPORT_CLASS(ws_qrcode_detection::qrcode_detection, nodelet::Nodelet)
