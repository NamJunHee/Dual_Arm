// Generated by gencpp from file serial_mcu/Mt2Serial_msg.msg
// DO NOT EDIT!


#ifndef SERIAL_MCU_MESSAGE_MT2SERIAL_MSG_H
#define SERIAL_MCU_MESSAGE_MT2SERIAL_MSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace serial_mcu
{
template <class ContainerAllocator>
struct Mt2Serial_msg_
{
  typedef Mt2Serial_msg_<ContainerAllocator> Type;

  Mt2Serial_msg_()
    : Motion_Mode(0)
    , Motion_Num(0)  {
    }
  Mt2Serial_msg_(const ContainerAllocator& _alloc)
    : Motion_Mode(0)
    , Motion_Num(0)  {
  (void)_alloc;
    }



   typedef int32_t _Motion_Mode_type;
  _Motion_Mode_type Motion_Mode;

   typedef int32_t _Motion_Num_type;
  _Motion_Num_type Motion_Num;





  typedef boost::shared_ptr< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> const> ConstPtr;

}; // struct Mt2Serial_msg_

typedef ::serial_mcu::Mt2Serial_msg_<std::allocator<void> > Mt2Serial_msg;

typedef boost::shared_ptr< ::serial_mcu::Mt2Serial_msg > Mt2Serial_msgPtr;
typedef boost::shared_ptr< ::serial_mcu::Mt2Serial_msg const> Mt2Serial_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace serial_mcu

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'serial_mcu': ['/home/robit/catkin_ws/src/serial_mcu/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e3f9ec431b0fdc81956c12a8b317c691";
  }

  static const char* value(const ::serial_mcu::Mt2Serial_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe3f9ec431b0fdc81ULL;
  static const uint64_t static_value2 = 0x956c12a8b317c691ULL;
};

template<class ContainerAllocator>
struct DataType< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "serial_mcu/Mt2Serial_msg";
  }

  static const char* value(const ::serial_mcu::Mt2Serial_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 Motion_Mode\n\
int32 Motion_Num\n\
";
  }

  static const char* value(const ::serial_mcu::Mt2Serial_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Motion_Mode);
      stream.next(m.Motion_Num);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Mt2Serial_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::serial_mcu::Mt2Serial_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::serial_mcu::Mt2Serial_msg_<ContainerAllocator>& v)
  {
    s << indent << "Motion_Mode: ";
    Printer<int32_t>::stream(s, indent + "  ", v.Motion_Mode);
    s << indent << "Motion_Num: ";
    Printer<int32_t>::stream(s, indent + "  ", v.Motion_Num);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SERIAL_MCU_MESSAGE_MT2SERIAL_MSG_H
