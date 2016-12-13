/* Auto-generated by genmsg_cpp for file /media/sf_403/catkin_ws/src/TowerDefense/tower_defense/srv/HurtCreeperSrv.srv */
#ifndef TOWER_DEFENSE_SERVICE_HURTCREEPERSRV_H
#define TOWER_DEFENSE_SERVICE_HURTCREEPERSRV_H
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

namespace tower_defense
{
template <class ContainerAllocator>
struct HurtCreeperSrvRequest_ {
  typedef HurtCreeperSrvRequest_<ContainerAllocator> Type;

  HurtCreeperSrvRequest_()
  : damage()
  , location()
  {
  }

  HurtCreeperSrvRequest_(const ContainerAllocator& _alloc)
  : damage(_alloc)
  , location(_alloc)
  {
  }

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _damage_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  damage;

  typedef std::vector< ::geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point32_<ContainerAllocator> >::other >  _location_type;
  std::vector< ::geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point32_<ContainerAllocator> >::other >  location;


  typedef boost::shared_ptr< ::tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tower_defense::HurtCreeperSrvRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct HurtCreeperSrvRequest
typedef  ::tower_defense::HurtCreeperSrvRequest_<std::allocator<void> > HurtCreeperSrvRequest;

typedef boost::shared_ptr< ::tower_defense::HurtCreeperSrvRequest> HurtCreeperSrvRequestPtr;
typedef boost::shared_ptr< ::tower_defense::HurtCreeperSrvRequest const> HurtCreeperSrvRequestConstPtr;



template <class ContainerAllocator>
struct HurtCreeperSrvResponse_ {
  typedef HurtCreeperSrvResponse_<ContainerAllocator> Type;

  HurtCreeperSrvResponse_()
  : creeper_locations()
  {
  }

  HurtCreeperSrvResponse_(const ContainerAllocator& _alloc)
  : creeper_locations(_alloc)
  {
  }

  typedef std::vector< ::geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point32_<ContainerAllocator> >::other >  _creeper_locations_type;
  std::vector< ::geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point32_<ContainerAllocator> >::other >  creeper_locations;


  typedef boost::shared_ptr< ::tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tower_defense::HurtCreeperSrvResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct HurtCreeperSrvResponse
typedef  ::tower_defense::HurtCreeperSrvResponse_<std::allocator<void> > HurtCreeperSrvResponse;

typedef boost::shared_ptr< ::tower_defense::HurtCreeperSrvResponse> HurtCreeperSrvResponsePtr;
typedef boost::shared_ptr< ::tower_defense::HurtCreeperSrvResponse const> HurtCreeperSrvResponseConstPtr;


struct HurtCreeperSrv
{

typedef HurtCreeperSrvRequest Request;
typedef HurtCreeperSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct HurtCreeperSrv
} // namespace tower_defense

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::tower_defense::HurtCreeperSrvRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9405ec177ba9539e8cf06f85753d7611";
  }

  static const char* value(const  ::tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9405ec177ba9539eULL;
  static const uint64_t static_value2 = 0x8cf06f85753d7611ULL;
};

template<class ContainerAllocator>
struct DataType< ::tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tower_defense/HurtCreeperSrvRequest";
  }

  static const char* value(const  ::tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32[] damage\n\
geometry_msgs/Point32[] location\n\
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

  static const char* value(const  ::tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::tower_defense::HurtCreeperSrvResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8ebb2c96323ccede5f4a491e0b8a8ec2";
  }

  static const char* value(const  ::tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8ebb2c96323ccedeULL;
  static const uint64_t static_value2 = 0x5f4a491e0b8a8ec2ULL;
};

template<class ContainerAllocator>
struct DataType< ::tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tower_defense/HurtCreeperSrvResponse";
  }

  static const char* value(const  ::tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Point32[] creeper_locations\n\
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

  static const char* value(const  ::tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.damage);
    stream.next(m.location);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct HurtCreeperSrvRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.creeper_locations);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct HurtCreeperSrvResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<tower_defense::HurtCreeperSrv> {
  static const char* value() 
  {
    return "b7fae7efe14208826ad348c6bf418130";
  }

  static const char* value(const tower_defense::HurtCreeperSrv&) { return value(); } 
};

template<>
struct DataType<tower_defense::HurtCreeperSrv> {
  static const char* value() 
  {
    return "tower_defense/HurtCreeperSrv";
  }

  static const char* value(const tower_defense::HurtCreeperSrv&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b7fae7efe14208826ad348c6bf418130";
  }

  static const char* value(const tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tower_defense/HurtCreeperSrv";
  }

  static const char* value(const tower_defense::HurtCreeperSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b7fae7efe14208826ad348c6bf418130";
  }

  static const char* value(const tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tower_defense/HurtCreeperSrv";
  }

  static const char* value(const tower_defense::HurtCreeperSrvResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // TOWER_DEFENSE_SERVICE_HURTCREEPERSRV_H

