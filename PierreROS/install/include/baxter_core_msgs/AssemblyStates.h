// Generated by gencpp from file baxter_core_msgs/AssemblyStates.msg
// DO NOT EDIT!


#ifndef BAXTER_CORE_MSGS_MESSAGE_ASSEMBLYSTATES_H
#define BAXTER_CORE_MSGS_MESSAGE_ASSEMBLYSTATES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <baxter_core_msgs/AssemblyState.h>

namespace baxter_core_msgs
{
template <class ContainerAllocator>
struct AssemblyStates_
{
  typedef AssemblyStates_<ContainerAllocator> Type;

  AssemblyStates_()
    : names()
    , states()  {
    }
  AssemblyStates_(const ContainerAllocator& _alloc)
    : names(_alloc)
    , states(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _names_type;
  _names_type names;

   typedef std::vector< ::baxter_core_msgs::AssemblyState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::baxter_core_msgs::AssemblyState_<ContainerAllocator> >::other >  _states_type;
  _states_type states;




  typedef boost::shared_ptr< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> const> ConstPtr;

}; // struct AssemblyStates_

typedef ::baxter_core_msgs::AssemblyStates_<std::allocator<void> > AssemblyStates;

typedef boost::shared_ptr< ::baxter_core_msgs::AssemblyStates > AssemblyStatesPtr;
typedef boost::shared_ptr< ::baxter_core_msgs::AssemblyStates const> AssemblyStatesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace baxter_core_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'baxter_core_msgs': ['/home/baxter/Desktop/PierreROS/src/baxter_common/baxter_core_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> >
{
  static const char* value()
  {
    return "63427318d41dbd2077c105027ad82a2b";
  }

  static const char* value(const ::baxter_core_msgs::AssemblyStates_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x63427318d41dbd20ULL;
  static const uint64_t static_value2 = 0x77c105027ad82a2bULL;
};

template<class ContainerAllocator>
struct DataType< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> >
{
  static const char* value()
  {
    return "baxter_core_msgs/AssemblyStates";
  }

  static const char* value(const ::baxter_core_msgs::AssemblyStates_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] names\n\
AssemblyState[] states\n\
================================================================================\n\
MSG: baxter_core_msgs/AssemblyState\n\
bool ready               # true if enabled and ready to operate, e.g., not homing\n\
bool enabled             # true if enabled\n\
bool stopped             # true if stopped -- e-stop asserted\n\
bool error               # true if a component of the assembly has an error\n\
#\n\
# The following are specific to the robot top-level assembly:\n\
uint8  estop_button      # One of the following:\n\
  uint8   ESTOP_BUTTON_UNPRESSED = 0   # Robot is not stopped and button is not pressed\n\
  uint8   ESTOP_BUTTON_PRESSED   = 1\n\
  uint8   ESTOP_BUTTON_UNKNOWN   = 2   # STATE_UNKNOWN when estop was asserted by a non-user source\n\
  uint8   ESTOP_BUTTON_RELEASED  = 3   # Was pressed, is now known to be released, but robot is still stopped.\n\
#\n\
uint8  estop_source      # If stopped is true, the source of the e-stop.  One of the following:\n\
  uint8  ESTOP_SOURCE_NONE      = 0   # e-stop is not asserted\n\
  uint8  ESTOP_SOURCE_USER      = 1   # e-stop source is user input (the red button)\n\
  uint8  ESTOP_SOURCE_UNKNOWN   = 2   # e-stop source is unknown\n\
  uint8  ESTOP_SOURCE_FAULT     = 3   # MotorController asserted e-stop in response to a joint fault\n\
  uint8  ESTOP_SOURCE_BRAIN     = 4   # MotorController asserted e-stop in response to a lapse of the brain heartbeat\n\
";
  }

  static const char* value(const ::baxter_core_msgs::AssemblyStates_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.names);
      stream.next(m.states);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AssemblyStates_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::baxter_core_msgs::AssemblyStates_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::baxter_core_msgs::AssemblyStates_<ContainerAllocator>& v)
  {
    s << indent << "names[]" << std::endl;
    for (size_t i = 0; i < v.names.size(); ++i)
    {
      s << indent << "  names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.names[i]);
    }
    s << indent << "states[]" << std::endl;
    for (size_t i = 0; i < v.states.size(); ++i)
    {
      s << indent << "  states[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::baxter_core_msgs::AssemblyState_<ContainerAllocator> >::stream(s, indent + "    ", v.states[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // BAXTER_CORE_MSGS_MESSAGE_ASSEMBLYSTATES_H
