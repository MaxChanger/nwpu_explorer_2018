// Generated by gencpp from file explorer_app_msgs/app_co2.msg
// DO NOT EDIT!


#ifndef EXPLORER_APP_MSGS_MESSAGE_APP_CO2_H
#define EXPLORER_APP_MSGS_MESSAGE_APP_CO2_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace explorer_app_msgs
{
template <class ContainerAllocator>
struct app_co2_
{
  typedef app_co2_<ContainerAllocator> Type;

  app_co2_()
    : head()
    , co2_data(0.0)  {
    }
  app_co2_(const ContainerAllocator& _alloc)
    : head(_alloc)
    , co2_data(0.0)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _head_type;
  _head_type head;

   typedef double _co2_data_type;
  _co2_data_type co2_data;




  typedef boost::shared_ptr< ::explorer_app_msgs::app_co2_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::explorer_app_msgs::app_co2_<ContainerAllocator> const> ConstPtr;

}; // struct app_co2_

typedef ::explorer_app_msgs::app_co2_<std::allocator<void> > app_co2;

typedef boost::shared_ptr< ::explorer_app_msgs::app_co2 > app_co2Ptr;
typedef boost::shared_ptr< ::explorer_app_msgs::app_co2 const> app_co2ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::explorer_app_msgs::app_co2_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::explorer_app_msgs::app_co2_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace explorer_app_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'explorer_app_msgs': ['/home/exbot/Document/ROS/explorer_test/src/explorer_app_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::explorer_app_msgs::app_co2_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::explorer_app_msgs::app_co2_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::explorer_app_msgs::app_co2_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::explorer_app_msgs::app_co2_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::explorer_app_msgs::app_co2_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::explorer_app_msgs::app_co2_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::explorer_app_msgs::app_co2_<ContainerAllocator> >
{
  static const char* value()
  {
    return "26624e065ce6035c1c4794394b5502a0";
  }

  static const char* value(const ::explorer_app_msgs::app_co2_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x26624e065ce6035cULL;
  static const uint64_t static_value2 = 0x1c4794394b5502a0ULL;
};

template<class ContainerAllocator>
struct DataType< ::explorer_app_msgs::app_co2_<ContainerAllocator> >
{
  static const char* value()
  {
    return "explorer_app_msgs/app_co2";
  }

  static const char* value(const ::explorer_app_msgs::app_co2_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::explorer_app_msgs::app_co2_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header head\n\
float64 co2_data\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::explorer_app_msgs::app_co2_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::explorer_app_msgs::app_co2_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.head);
      stream.next(m.co2_data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct app_co2_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::explorer_app_msgs::app_co2_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::explorer_app_msgs::app_co2_<ContainerAllocator>& v)
  {
    s << indent << "head: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.head);
    s << indent << "co2_data: ";
    Printer<double>::stream(s, indent + "  ", v.co2_data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // EXPLORER_APP_MSGS_MESSAGE_APP_CO2_H
