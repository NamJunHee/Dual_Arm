// Generated by gencpp from file order_kuro/motion_name.msg
// DO NOT EDIT!


#ifndef ORDER_KURO_MESSAGE_MOTION_NAME_H
#define ORDER_KURO_MESSAGE_MOTION_NAME_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace order_kuro
{
template <class ContainerAllocator>
struct motion_name_
{
  typedef motion_name_<ContainerAllocator> Type;

  motion_name_()
    : now_motion_name()
    , before_motion_name()  {
    }
  motion_name_(const ContainerAllocator& _alloc)
    : now_motion_name(_alloc)
    , before_motion_name(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _now_motion_name_type;
  _now_motion_name_type now_motion_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _before_motion_name_type;
  _before_motion_name_type before_motion_name;





  typedef boost::shared_ptr< ::order_kuro::motion_name_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::order_kuro::motion_name_<ContainerAllocator> const> ConstPtr;

}; // struct motion_name_

typedef ::order_kuro::motion_name_<std::allocator<void> > motion_name;

typedef boost::shared_ptr< ::order_kuro::motion_name > motion_namePtr;
typedef boost::shared_ptr< ::order_kuro::motion_name const> motion_nameConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::order_kuro::motion_name_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::order_kuro::motion_name_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::order_kuro::motion_name_<ContainerAllocator1> & lhs, const ::order_kuro::motion_name_<ContainerAllocator2> & rhs)
{
  return lhs.now_motion_name == rhs.now_motion_name &&
    lhs.before_motion_name == rhs.before_motion_name;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::order_kuro::motion_name_<ContainerAllocator1> & lhs, const ::order_kuro::motion_name_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace order_kuro

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::order_kuro::motion_name_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::order_kuro::motion_name_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::order_kuro::motion_name_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::order_kuro::motion_name_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::order_kuro::motion_name_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::order_kuro::motion_name_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::order_kuro::motion_name_<ContainerAllocator> >
{
  static const char* value()
  {
    return "970b047434aaebce479cac5494e51bce";
  }

  static const char* value(const ::order_kuro::motion_name_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x970b047434aaebceULL;
  static const uint64_t static_value2 = 0x479cac5494e51bceULL;
};

template<class ContainerAllocator>
struct DataType< ::order_kuro::motion_name_<ContainerAllocator> >
{
  static const char* value()
  {
    return "order_kuro/motion_name";
  }

  static const char* value(const ::order_kuro::motion_name_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::order_kuro::motion_name_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string now_motion_name\n"
"string before_motion_name\n"
;
  }

  static const char* value(const ::order_kuro::motion_name_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::order_kuro::motion_name_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.now_motion_name);
      stream.next(m.before_motion_name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct motion_name_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::order_kuro::motion_name_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::order_kuro::motion_name_<ContainerAllocator>& v)
  {
    s << indent << "now_motion_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.now_motion_name);
    s << indent << "before_motion_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.before_motion_name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ORDER_KURO_MESSAGE_MOTION_NAME_H
