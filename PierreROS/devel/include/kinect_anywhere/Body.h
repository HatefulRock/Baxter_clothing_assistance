// Generated by gencpp from file kinect_anywhere/Body.msg
// DO NOT EDIT!


#ifndef KINECT_ANYWHERE_MESSAGE_BODY_H
#define KINECT_ANYWHERE_MESSAGE_BODY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <kinect_anywhere/JointPositionAndState.h>

namespace kinect_anywhere
{
template <class ContainerAllocator>
struct Body_
{
  typedef Body_<ContainerAllocator> Type;

  Body_()
    : header()
    , trackingId(0)
    , jointPositions()  {
    }
  Body_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , trackingId(0)
    , jointPositions(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint64_t _trackingId_type;
  _trackingId_type trackingId;

   typedef std::vector< ::kinect_anywhere::JointPositionAndState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::kinect_anywhere::JointPositionAndState_<ContainerAllocator> >::other >  _jointPositions_type;
  _jointPositions_type jointPositions;




  typedef boost::shared_ptr< ::kinect_anywhere::Body_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kinect_anywhere::Body_<ContainerAllocator> const> ConstPtr;

}; // struct Body_

typedef ::kinect_anywhere::Body_<std::allocator<void> > Body;

typedef boost::shared_ptr< ::kinect_anywhere::Body > BodyPtr;
typedef boost::shared_ptr< ::kinect_anywhere::Body const> BodyConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kinect_anywhere::Body_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kinect_anywhere::Body_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace kinect_anywhere

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'kinect_anywhere': ['/home/baxter/Desktop/PierreROS/src/kinect_anywhere/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::kinect_anywhere::Body_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kinect_anywhere::Body_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kinect_anywhere::Body_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kinect_anywhere::Body_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinect_anywhere::Body_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinect_anywhere::Body_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kinect_anywhere::Body_<ContainerAllocator> >
{
  static const char* value()
  {
    return "03172da3525832b16f0958db416a2754";
  }

  static const char* value(const ::kinect_anywhere::Body_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x03172da3525832b1ULL;
  static const uint64_t static_value2 = 0x6f0958db416a2754ULL;
};

template<class ContainerAllocator>
struct DataType< ::kinect_anywhere::Body_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kinect_anywhere/Body";
  }

  static const char* value(const ::kinect_anywhere::Body_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kinect_anywhere::Body_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
uint64 trackingId\n\
JointPositionAndState[] jointPositions\n\
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
\n\
================================================================================\n\
MSG: kinect_anywhere/JointPositionAndState\n\
int32 trackingState\n\
geometry_msgs/Point position\n\
int32 jointType\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::kinect_anywhere::Body_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kinect_anywhere::Body_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.trackingId);
      stream.next(m.jointPositions);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Body_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kinect_anywhere::Body_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kinect_anywhere::Body_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "trackingId: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.trackingId);
    s << indent << "jointPositions[]" << std::endl;
    for (size_t i = 0; i < v.jointPositions.size(); ++i)
    {
      s << indent << "  jointPositions[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::kinect_anywhere::JointPositionAndState_<ContainerAllocator> >::stream(s, indent + "    ", v.jointPositions[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // KINECT_ANYWHERE_MESSAGE_BODY_H
