/* Auto-generated by genmsg_cpp for file /home/roman/robotics_ws/src/TowerDefense/tower_defense/srv/RandomConfigSrv.srv */
#ifndef TOWER_DEFENSE_SERVICE_RANDOMCONFIGSRV_H
#define TOWER_DEFENSE_SERVICE_RANDOMCONFIGSRV_H
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



#include "geometry_msgs/Point.h"

namespace tower_defense
{
template <class ContainerAllocator>
struct RandomConfigSrvRequest_ {
  typedef RandomConfigSrvRequest_<ContainerAllocator> Type;

  RandomConfigSrvRequest_()
  {
  }

  RandomConfigSrvRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::tower_defense::RandomConfigSrvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tower_defense::RandomConfigSrvRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct RandomConfigSrvRequest
typedef  ::tower_defense::RandomConfigSrvRequest_<std::allocator<void> > RandomConfigSrvRequest;

typedef boost::shared_ptr< ::tower_defense::RandomConfigSrvRequest> RandomConfigSrvRequestPtr;
typedef boost::shared_ptr< ::tower_defense::RandomConfigSrvRequest const> RandomConfigSrvRequestConstPtr;



template <class ContainerAllocator>
struct RandomConfigSrvResponse_ {
  typedef RandomConfigSrvResponse_<ContainerAllocator> Type;

  RandomConfigSrvResponse_()
  : q_rand()
  {
  }

  RandomConfigSrvResponse_(const ContainerAllocator& _alloc)
  : q_rand(_alloc)
  {
  }

  typedef  ::geometry_msgs::Point_<ContainerAllocator>  _q_rand_type;
   ::geometry_msgs::Point_<ContainerAllocator>  q_rand;


  typedef boost::shared_ptr< ::tower_defense::RandomConfigSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tower_defense::RandomConfigSrvResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct RandomConfigSrvResponse
typedef  ::tower_defense::RandomConfigSrvResponse_<std::allocator<void> > RandomConfigSrvResponse;

typedef boost::shared_ptr< ::tower_defense::RandomConfigSrvResponse> RandomConfigSrvResponsePtr;
typedef boost::shared_ptr< ::tower_defense::RandomConfigSrvResponse const> RandomConfigSrvResponseConstPtr;


struct RandomConfigSrv
{

typedef RandomConfigSrvRequest Request;
typedef RandomConfigSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct RandomConfigSrv
} // namespace tower_defense

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::tower_defense::RandomConfigSrvRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::tower_defense::RandomConfigSrvRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::tower_defense::RandomConfigSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::tower_defense::RandomConfigSrvRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::tower_defense::RandomConfigSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tower_defense/RandomConfigSrvRequest";
  }

  static const char* value(const  ::tower_defense::RandomConfigSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::tower_defense::RandomConfigSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::tower_defense::RandomConfigSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::tower_defense::RandomConfigSrvRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::tower_defense::RandomConfigSrvResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::tower_defense::RandomConfigSrvResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::tower_defense::RandomConfigSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ebedd2ab5ada3637b4212236735875be";
  }

  static const char* value(const  ::tower_defense::RandomConfigSrvResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xebedd2ab5ada3637ULL;
  static const uint64_t static_value2 = 0xb4212236735875beULL;
};

template<class ContainerAllocator>
struct DataType< ::tower_defense::RandomConfigSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tower_defense/RandomConfigSrvResponse";
  }

  static const char* value(const  ::tower_defense::RandomConfigSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::tower_defense::RandomConfigSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Point q_rand\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
";
  }

  static const char* value(const  ::tower_defense::RandomConfigSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::tower_defense::RandomConfigSrvResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::tower_defense::RandomConfigSrvRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct RandomConfigSrvRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::tower_defense::RandomConfigSrvResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.q_rand);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct RandomConfigSrvResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<tower_defense::RandomConfigSrv> {
  static const char* value() 
  {
    return "ebedd2ab5ada3637b4212236735875be";
  }

  static const char* value(const tower_defense::RandomConfigSrv&) { return value(); } 
};

template<>
struct DataType<tower_defense::RandomConfigSrv> {
  static const char* value() 
  {
    return "tower_defense/RandomConfigSrv";
  }

  static const char* value(const tower_defense::RandomConfigSrv&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<tower_defense::RandomConfigSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ebedd2ab5ada3637b4212236735875be";
  }

  static const char* value(const tower_defense::RandomConfigSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<tower_defense::RandomConfigSrvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tower_defense/RandomConfigSrv";
  }

  static const char* value(const tower_defense::RandomConfigSrvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<tower_defense::RandomConfigSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ebedd2ab5ada3637b4212236735875be";
  }

  static const char* value(const tower_defense::RandomConfigSrvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<tower_defense::RandomConfigSrvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tower_defense/RandomConfigSrv";
  }

  static const char* value(const tower_defense::RandomConfigSrvResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // TOWER_DEFENSE_SERVICE_RANDOMCONFIGSRV_H

