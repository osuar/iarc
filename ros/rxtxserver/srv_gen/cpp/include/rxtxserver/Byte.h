/* Auto-generated by genmsg_cpp for file /root/ros_workspace/rxtxserver/srv/Byte.srv */
#ifndef RXTXSERVER_SERVICE_BYTE_H
#define RXTXSERVER_SERVICE_BYTE_H
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

#include "ros/service_traits.h"




namespace rxtxserver
{
template <class ContainerAllocator>
struct ByteRequest_ {
  typedef ByteRequest_<ContainerAllocator> Type;

  ByteRequest_()
  : outData(0)
  {
  }

  ByteRequest_(const ContainerAllocator& _alloc)
  : outData(0)
  {
  }

  typedef int8_t _outData_type;
  int8_t outData;


  typedef boost::shared_ptr< ::rxtxserver::ByteRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rxtxserver::ByteRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ByteRequest
typedef  ::rxtxserver::ByteRequest_<std::allocator<void> > ByteRequest;

typedef boost::shared_ptr< ::rxtxserver::ByteRequest> ByteRequestPtr;
typedef boost::shared_ptr< ::rxtxserver::ByteRequest const> ByteRequestConstPtr;


template <class ContainerAllocator>
struct ByteResponse_ {
  typedef ByteResponse_<ContainerAllocator> Type;

  ByteResponse_()
  : outResponse(0)
  {
  }

  ByteResponse_(const ContainerAllocator& _alloc)
  : outResponse(0)
  {
  }

  typedef int8_t _outResponse_type;
  int8_t outResponse;


  typedef boost::shared_ptr< ::rxtxserver::ByteResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rxtxserver::ByteResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ByteResponse
typedef  ::rxtxserver::ByteResponse_<std::allocator<void> > ByteResponse;

typedef boost::shared_ptr< ::rxtxserver::ByteResponse> ByteResponsePtr;
typedef boost::shared_ptr< ::rxtxserver::ByteResponse const> ByteResponseConstPtr;

struct Byte
{

typedef ByteRequest Request;
typedef ByteResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Byte
} // namespace rxtxserver

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rxtxserver::ByteRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rxtxserver::ByteRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rxtxserver::ByteRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "10545a036ebd67aa10ae2e15db0715dd";
  }

  static const char* value(const  ::rxtxserver::ByteRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x10545a036ebd67aaULL;
  static const uint64_t static_value2 = 0x10ae2e15db0715ddULL;
};

template<class ContainerAllocator>
struct DataType< ::rxtxserver::ByteRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rxtxserver/ByteRequest";
  }

  static const char* value(const  ::rxtxserver::ByteRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rxtxserver::ByteRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "byte outData\n\
\n\
";
  }

  static const char* value(const  ::rxtxserver::ByteRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::rxtxserver::ByteRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rxtxserver::ByteResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rxtxserver::ByteResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rxtxserver::ByteResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6f6260512b0312b7a3896fe1d9bd80ab";
  }

  static const char* value(const  ::rxtxserver::ByteResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6f6260512b0312b7ULL;
  static const uint64_t static_value2 = 0xa3896fe1d9bd80abULL;
};

template<class ContainerAllocator>
struct DataType< ::rxtxserver::ByteResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rxtxserver/ByteResponse";
  }

  static const char* value(const  ::rxtxserver::ByteResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rxtxserver::ByteResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "byte outResponse\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::rxtxserver::ByteResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::rxtxserver::ByteResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rxtxserver::ByteRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.outData);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ByteRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rxtxserver::ByteResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.outResponse);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ByteResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<rxtxserver::Byte> {
  static const char* value() 
  {
    return "1363ee3da7a1536cdcdcdbac42798f92";
  }

  static const char* value(const rxtxserver::Byte&) { return value(); } 
};

template<>
struct DataType<rxtxserver::Byte> {
  static const char* value() 
  {
    return "rxtxserver/Byte";
  }

  static const char* value(const rxtxserver::Byte&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rxtxserver::ByteRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1363ee3da7a1536cdcdcdbac42798f92";
  }

  static const char* value(const rxtxserver::ByteRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rxtxserver::ByteRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rxtxserver/Byte";
  }

  static const char* value(const rxtxserver::ByteRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rxtxserver::ByteResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1363ee3da7a1536cdcdcdbac42798f92";
  }

  static const char* value(const rxtxserver::ByteResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rxtxserver::ByteResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rxtxserver/Byte";
  }

  static const char* value(const rxtxserver::ByteResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // RXTXSERVER_SERVICE_BYTE_H
