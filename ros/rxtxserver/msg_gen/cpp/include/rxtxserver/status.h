/* Auto-generated by genmsg_cpp for file /root/ros_workspace/rxtxserver/msg/status.msg */
#ifndef RXTXSERVER_MESSAGE_STATUS_H
#define RXTXSERVER_MESSAGE_STATUS_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace rxtxserver
{
template <class ContainerAllocator>
struct status_ {
  typedef status_<ContainerAllocator> Type;

  status_()
  : status(0)
  {
  }

  status_(const ContainerAllocator& _alloc)
  : status(0)
  {
  }

  typedef uint8_t _status_type;
  uint8_t status;


  typedef boost::shared_ptr< ::rxtxserver::status_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rxtxserver::status_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct status
typedef  ::rxtxserver::status_<std::allocator<void> > status;

typedef boost::shared_ptr< ::rxtxserver::status> statusPtr;
typedef boost::shared_ptr< ::rxtxserver::status const> statusConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::rxtxserver::status_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::rxtxserver::status_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace rxtxserver

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rxtxserver::status_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rxtxserver::status_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rxtxserver::status_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8695a687dd99fd6c4e83bb483ce1c397";
  }

  static const char* value(const  ::rxtxserver::status_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8695a687dd99fd6cULL;
  static const uint64_t static_value2 = 0x4e83bb483ce1c397ULL;
};

template<class ContainerAllocator>
struct DataType< ::rxtxserver::status_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rxtxserver/status";
  }

  static const char* value(const  ::rxtxserver::status_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rxtxserver::status_<ContainerAllocator> > {
  static const char* value() 
  {
    return "char status\n\
\n\
";
  }

  static const char* value(const  ::rxtxserver::status_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::rxtxserver::status_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rxtxserver::status_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.status);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct status_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rxtxserver::status_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::rxtxserver::status_<ContainerAllocator> & v) 
  {
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
  }
};


} // namespace message_operations
} // namespace ros

#endif // RXTXSERVER_MESSAGE_STATUS_H
