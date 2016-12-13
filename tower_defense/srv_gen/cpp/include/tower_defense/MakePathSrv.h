/* Auto-generated by genmsg_cpp for file /home/roman/robotics_ws/src/TowerDefense/tower_defense/srv/MakePathSrv.srv */
#ifndef TOWER_DEFENSE_SERVICE_MAKEPATHSRV_H
#define TOWER_DEFENSE_SERVICE_MAKEPATHSRV_H
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

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point32.h"


#include "geometry_msgs/Point32.h"

namespace tower_defense
{
template <class ContainerAllocator>
struct MakePathSrvRequest_ {
  typedef MakePathSrvRequest_<ContainerAllocator> Type;

  MakePathSrvRequest_()
  : point_cloud()
  , start()
  , end()
  {
  }

  MakePathSrvRequest_(const ContainerAllocator& _alloc)
  : point_cloud(_alloc)
  , start(_alloc)
  , end(_alloc)
  {
  }

  typedef std::vector< ::geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point32_<ContainerAllocator> >::other >  _point_cloud_type;
  std::vector< ::geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point32_<ContainerAllocator> >::other >  point_cloud;

  typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _start_type;
   ::geometry_msgs::Point32_<ContainerAllocator>  start;

  typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _end_type;
   ::geometry_msgs::Point32_<ContainerAllocator>  end;


  typedef boost::shared_ptr< ::tower_defense::MakePathSrvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tower_defense::MakePathSrvRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct MakePathSrvRequest
typedef  ::tower_defense::MakePathSrvRequest_<std::allocator<void> > MakePathSrvRequest;

typedef boost::shared_ptr< ::tower_defense::MakePathSrvRequest> MakePathSrvRequestPtr;
typedef boost::shared_ptr< ::tower_defense::MakePathSrvRequest const> MakePathSrvRequestConstPtr;



template <class ContainerAllocator>
struct MakePathSrvResponse_ {
  typedef MakePathSrvResponse_<ContainerAllocator> Type;

  MakePathSrvResponse_()
  : path()
  {
  }

  MakePathSrvResponse_(const ContainerAllocator& _alloc)
  : path(_alloc)
  {
  }

  typedef std::vector< ::geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point32_<ContainerAllocator> >::other >  _path_type;
  std::vector< ::geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point32_<ContainerAllocator> >::other >  path;


  typedef boost::shared_ptr< ::tower_defense::MakePathSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tower_defense::MakePathSrvResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct MakePathSrvResponse
typedef  ::tower_defense::MakePathSrvResponse_<std::allocator<void> > MakePathSrvResponse;

typedef boost::shared_ptr< ::tower_defense::MakePathSrvResponse> MakePathSrvResponsePtr;
typedef boost::shared_ptr< ::tower_defense::MakePathSrvResponse const> MakePathSrvResponseConstPtr;


struct MakePathSrv
{

typedef MakePathSrvRequest Request;
typedef MakePathSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct MakePathSrv
} // namespace tower_defense

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::tower_defense::MakePathSrvRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::tower_defense::MakePathSrvRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::tower_defense::MakePathSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ea0496f3f9a3d3f90bd5b9247dd93b86";
  }

  static const char* value(const  ::tower_defense::MakePathSrvRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xea0496f3f9a3d3f9ULL;
  static const uint64_t static_value2 = 0x0bd5b9247dd93b86ULL;
};

template<class ContainerAllocator>
struct DataType< ::tower_defense::MakePathSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tower_defense/MakePathSrvRequest";
  }

  static const char* value(const  ::tower_defense::MakePathSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::tower_defense::MakePathSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Point32[] point_cloud\n\
geometry_msgs/Point32 start\n\
geometry_msgs/Point32 end\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const  ::tower_defense::MakePathSrvRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::tower_defense::MakePathSrvResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::tower_defense::MakePathSrvResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::tower_defense::MakePathSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6b486115390018cf18c53ea7f1a78dca";
  }

  static const char* value(const  ::tower_defense::MakePathSrvResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6b486115390018cfULL;
  static const uint64_t static_value2 = 0x18c53ea7f1a78dcaULL;
};

template<class ContainerAllocator>
struct DataType< ::tower_defense::MakePathSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tower_defense/MakePathSrvResponse";
  }

  static const char* value(const  ::tower_defense::MakePathSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::tower_defense::MakePathSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Point32[] path\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const  ::tower_defense::MakePathSrvResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::tower_defense::MakePathSrvRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.point_cloud);
    stream.next(m.start);
    stream.next(m.end);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct MakePathSrvRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::tower_defense::MakePathSrvResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.path);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct MakePathSrvResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<tower_defense::MakePathSrv> {
  static const char* value() 
  {
    return "9affbb1cb020b95f89d957afa255e3f1";
  }

  static const char* value(const tower_defense::MakePathSrv&) { return value(); } 
};

template<>
struct DataType<tower_defense::MakePathSrv> {
  static const char* value() 
  {
    return "tower_defense/MakePathSrv";
  }

  static const char* value(const tower_defense::MakePathSrv&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<tower_defense::MakePathSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9affbb1cb020b95f89d957afa255e3f1";
  }

  static const char* value(const tower_defense::MakePathSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<tower_defense::MakePathSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tower_defense/MakePathSrv";
  }

  static const char* value(const tower_defense::MakePathSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<tower_defense::MakePathSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9affbb1cb020b95f89d957afa255e3f1";
  }

  static const char* value(const tower_defense::MakePathSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<tower_defense::MakePathSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tower_defense/MakePathSrv";
  }

  static const char* value(const tower_defense::MakePathSrvResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // TOWER_DEFENSE_SERVICE_MAKEPATHSRV_H

