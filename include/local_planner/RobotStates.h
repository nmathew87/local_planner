/* Auto-generated by genmsg_cpp for file /home/mowgli/Dropbox/NASAstuff/NASAgit/uwrobotics/NASA/nasa_simulator/simBotCommon/msg/RobotStates.msg */
#ifndef SIMBOTCOMMON_MESSAGE_ROBOTSTATES_H
#define SIMBOTCOMMON_MESSAGE_ROBOTSTATES_H
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

#include "std_msgs/Header.h"

namespace simBotCommon
{
template <class ContainerAllocator>
struct RobotStates_ {
  typedef RobotStates_<ContainerAllocator> Type;

  RobotStates_()
  : header()
  , x(0.0)
  , y(0.0)
  , z(0.0)
  , x_dot(0.0)
  , y_dot(0.0)
  , z_dot(0.0)
  , roll_phi(0.0)
  , pitch_theta(0.0)
  , yaw_psi(0.0)
  , p_rate(0.0)
  , q_rate(0.0)
  , r_rate(0.0)
  , steerAngle(0.0)
  {
  }

  RobotStates_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , x(0.0)
  , y(0.0)
  , z(0.0)
  , x_dot(0.0)
  , y_dot(0.0)
  , z_dot(0.0)
  , roll_phi(0.0)
  , pitch_theta(0.0)
  , yaw_psi(0.0)
  , p_rate(0.0)
  , q_rate(0.0)
  , r_rate(0.0)
  , steerAngle(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef double _x_type;
  double x;

  typedef double _y_type;
  double y;

  typedef double _z_type;
  double z;

  typedef double _x_dot_type;
  double x_dot;

  typedef double _y_dot_type;
  double y_dot;

  typedef double _z_dot_type;
  double z_dot;

  typedef double _roll_phi_type;
  double roll_phi;

  typedef double _pitch_theta_type;
  double pitch_theta;

  typedef double _yaw_psi_type;
  double yaw_psi;

  typedef double _p_rate_type;
  double p_rate;

  typedef double _q_rate_type;
  double q_rate;

  typedef double _r_rate_type;
  double r_rate;

  typedef double _steerAngle_type;
  double steerAngle;


  typedef boost::shared_ptr< ::simBotCommon::RobotStates_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::simBotCommon::RobotStates_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RobotStates
typedef  ::simBotCommon::RobotStates_<std::allocator<void> > RobotStates;

typedef boost::shared_ptr< ::simBotCommon::RobotStates> RobotStatesPtr;
typedef boost::shared_ptr< ::simBotCommon::RobotStates const> RobotStatesConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::simBotCommon::RobotStates_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::simBotCommon::RobotStates_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace simBotCommon

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::simBotCommon::RobotStates_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::simBotCommon::RobotStates_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::simBotCommon::RobotStates_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fb942e346c1637f25d687c78ffbf0c44";
  }

  static const char* value(const  ::simBotCommon::RobotStates_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xfb942e346c1637f2ULL;
  static const uint64_t static_value2 = 0x5d687c78ffbf0c44ULL;
};

template<class ContainerAllocator>
struct DataType< ::simBotCommon::RobotStates_<ContainerAllocator> > {
  static const char* value() 
  {
    return "simBotCommon/RobotStates";
  }

  static const char* value(const  ::simBotCommon::RobotStates_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::simBotCommon::RobotStates_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float64 x # position of the robot in x, relative to inertial frame\n\
float64 y # position of the robot in y, relative to inertial frame\n\
float64 z # position of the robot in z, relative to inertial frame\n\
float64 x_dot # linear velocity in x\n\
float64 y_dot # linear velocity in y\n\
float64 z_dot # linear velocity in z\n\
float64 roll_phi # euler angle - roll, orientation relative to inertial frame, in radians\n\
float64 pitch_theta # euler angle - pitch, orientation relative to inertial frame, in radians\n\
float64 yaw_psi # euler angle - yaw, orientation relative to inertial frame, in radians\n\
float64 p_rate # body angular velocity around roll\n\
float64 q_rate # body angular velocity around pitch\n\
float64 r_rate # body angular velocity around yaw\n\
float64 steerAngle # in radians\n\
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
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::simBotCommon::RobotStates_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::simBotCommon::RobotStates_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::simBotCommon::RobotStates_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::simBotCommon::RobotStates_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
    stream.next(m.x_dot);
    stream.next(m.y_dot);
    stream.next(m.z_dot);
    stream.next(m.roll_phi);
    stream.next(m.pitch_theta);
    stream.next(m.yaw_psi);
    stream.next(m.p_rate);
    stream.next(m.q_rate);
    stream.next(m.r_rate);
    stream.next(m.steerAngle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RobotStates_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::simBotCommon::RobotStates_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::simBotCommon::RobotStates_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "x_dot: ";
    Printer<double>::stream(s, indent + "  ", v.x_dot);
    s << indent << "y_dot: ";
    Printer<double>::stream(s, indent + "  ", v.y_dot);
    s << indent << "z_dot: ";
    Printer<double>::stream(s, indent + "  ", v.z_dot);
    s << indent << "roll_phi: ";
    Printer<double>::stream(s, indent + "  ", v.roll_phi);
    s << indent << "pitch_theta: ";
    Printer<double>::stream(s, indent + "  ", v.pitch_theta);
    s << indent << "yaw_psi: ";
    Printer<double>::stream(s, indent + "  ", v.yaw_psi);
    s << indent << "p_rate: ";
    Printer<double>::stream(s, indent + "  ", v.p_rate);
    s << indent << "q_rate: ";
    Printer<double>::stream(s, indent + "  ", v.q_rate);
    s << indent << "r_rate: ";
    Printer<double>::stream(s, indent + "  ", v.r_rate);
    s << indent << "steerAngle: ";
    Printer<double>::stream(s, indent + "  ", v.steerAngle);
  }
};


} // namespace message_operations
} // namespace ros

#endif // SIMBOTCOMMON_MESSAGE_ROBOTSTATES_H

