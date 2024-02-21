// Generated by gencpp from file custom_msgs/XL320Command.msg
// DO NOT EDIT!


#ifndef CUSTOM_MSGS_MESSAGE_XL320COMMAND_H
#define CUSTOM_MSGS_MESSAGE_XL320COMMAND_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace custom_msgs
{
template <class ContainerAllocator>
struct XL320Command_
{
  typedef XL320Command_<ContainerAllocator> Type;

  XL320Command_()
    : setpoints()
    , servo_names()  {
    }
  XL320Command_(const ContainerAllocator& _alloc)
    : setpoints(_alloc)
    , servo_names(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _setpoints_type;
  _setpoints_type setpoints;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _servo_names_type;
  _servo_names_type servo_names;





  typedef boost::shared_ptr< ::custom_msgs::XL320Command_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::custom_msgs::XL320Command_<ContainerAllocator> const> ConstPtr;

}; // struct XL320Command_

typedef ::custom_msgs::XL320Command_<std::allocator<void> > XL320Command;

typedef boost::shared_ptr< ::custom_msgs::XL320Command > XL320CommandPtr;
typedef boost::shared_ptr< ::custom_msgs::XL320Command const> XL320CommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::custom_msgs::XL320Command_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::custom_msgs::XL320Command_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::custom_msgs::XL320Command_<ContainerAllocator1> & lhs, const ::custom_msgs::XL320Command_<ContainerAllocator2> & rhs)
{
  return lhs.setpoints == rhs.setpoints &&
    lhs.servo_names == rhs.servo_names;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::custom_msgs::XL320Command_<ContainerAllocator1> & lhs, const ::custom_msgs::XL320Command_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace custom_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::custom_msgs::XL320Command_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::custom_msgs::XL320Command_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msgs::XL320Command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msgs::XL320Command_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msgs::XL320Command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msgs::XL320Command_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::custom_msgs::XL320Command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0f91075e074481ff3aec0e2096d7e076";
  }

  static const char* value(const ::custom_msgs::XL320Command_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0f91075e074481ffULL;
  static const uint64_t static_value2 = 0x3aec0e2096d7e076ULL;
};

template<class ContainerAllocator>
struct DataType< ::custom_msgs::XL320Command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "custom_msgs/XL320Command";
  }

  static const char* value(const ::custom_msgs::XL320Command_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::custom_msgs::XL320Command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[] setpoints\n"
"string[] servo_names\n"
;
  }

  static const char* value(const ::custom_msgs::XL320Command_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::custom_msgs::XL320Command_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.setpoints);
      stream.next(m.servo_names);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct XL320Command_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::custom_msgs::XL320Command_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::custom_msgs::XL320Command_<ContainerAllocator>& v)
  {
    s << indent << "setpoints[]" << std::endl;
    for (size_t i = 0; i < v.setpoints.size(); ++i)
    {
      s << indent << "  setpoints[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.setpoints[i]);
    }
    s << indent << "servo_names[]" << std::endl;
    for (size_t i = 0; i < v.servo_names.size(); ++i)
    {
      s << indent << "  servo_names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.servo_names[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CUSTOM_MSGS_MESSAGE_XL320COMMAND_H