// Generated by gencpp from file move_DualArm/fk_loca.msg
// DO NOT EDIT!


#ifndef MOVE_DUALARM_MESSAGE_FK_LOCA_H
#define MOVE_DUALARM_MESSAGE_FK_LOCA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace move_DualArm
{
template <class ContainerAllocator>
struct fk_loca_
{
  typedef fk_loca_<ContainerAllocator> Type;

  fk_loca_()
    : r_fk_pX(0.0)
    , r_fk_pY(0.0)
    , r_fk_pZ(0.0)
    , l_fk_pX(0.0)
    , l_fk_pY(0.0)
    , l_fk_pZ(0.0)  {
    }
  fk_loca_(const ContainerAllocator& _alloc)
    : r_fk_pX(0.0)
    , r_fk_pY(0.0)
    , r_fk_pZ(0.0)
    , l_fk_pX(0.0)
    , l_fk_pY(0.0)
    , l_fk_pZ(0.0)  {
  (void)_alloc;
    }



   typedef float _r_fk_pX_type;
  _r_fk_pX_type r_fk_pX;

   typedef float _r_fk_pY_type;
  _r_fk_pY_type r_fk_pY;

   typedef float _r_fk_pZ_type;
  _r_fk_pZ_type r_fk_pZ;

   typedef float _l_fk_pX_type;
  _l_fk_pX_type l_fk_pX;

   typedef float _l_fk_pY_type;
  _l_fk_pY_type l_fk_pY;

   typedef float _l_fk_pZ_type;
  _l_fk_pZ_type l_fk_pZ;





  typedef boost::shared_ptr< ::move_DualArm::fk_loca_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::move_DualArm::fk_loca_<ContainerAllocator> const> ConstPtr;

}; // struct fk_loca_

typedef ::move_DualArm::fk_loca_<std::allocator<void> > fk_loca;

typedef boost::shared_ptr< ::move_DualArm::fk_loca > fk_locaPtr;
typedef boost::shared_ptr< ::move_DualArm::fk_loca const> fk_locaConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::move_DualArm::fk_loca_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::move_DualArm::fk_loca_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::move_DualArm::fk_loca_<ContainerAllocator1> & lhs, const ::move_DualArm::fk_loca_<ContainerAllocator2> & rhs)
{
  return lhs.r_fk_pX == rhs.r_fk_pX &&
    lhs.r_fk_pY == rhs.r_fk_pY &&
    lhs.r_fk_pZ == rhs.r_fk_pZ &&
    lhs.l_fk_pX == rhs.l_fk_pX &&
    lhs.l_fk_pY == rhs.l_fk_pY &&
    lhs.l_fk_pZ == rhs.l_fk_pZ;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::move_DualArm::fk_loca_<ContainerAllocator1> & lhs, const ::move_DualArm::fk_loca_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace move_DualArm

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::move_DualArm::fk_loca_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::move_DualArm::fk_loca_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::move_DualArm::fk_loca_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::move_DualArm::fk_loca_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::move_DualArm::fk_loca_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::move_DualArm::fk_loca_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::move_DualArm::fk_loca_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ec9de62b0316327a7f28eb1c29eba036";
  }

  static const char* value(const ::move_DualArm::fk_loca_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xec9de62b0316327aULL;
  static const uint64_t static_value2 = 0x7f28eb1c29eba036ULL;
};

template<class ContainerAllocator>
struct DataType< ::move_DualArm::fk_loca_<ContainerAllocator> >
{
  static const char* value()
  {
    return "move_DualArm/fk_loca";
  }

  static const char* value(const ::move_DualArm::fk_loca_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::move_DualArm::fk_loca_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 r_fk_pX\n"
"float32 r_fk_pY\n"
"float32 r_fk_pZ\n"
"float32 l_fk_pX\n"
"float32 l_fk_pY\n"
"float32 l_fk_pZ\n"
;
  }

  static const char* value(const ::move_DualArm::fk_loca_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::move_DualArm::fk_loca_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.r_fk_pX);
      stream.next(m.r_fk_pY);
      stream.next(m.r_fk_pZ);
      stream.next(m.l_fk_pX);
      stream.next(m.l_fk_pY);
      stream.next(m.l_fk_pZ);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct fk_loca_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::move_DualArm::fk_loca_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::move_DualArm::fk_loca_<ContainerAllocator>& v)
  {
    s << indent << "r_fk_pX: ";
    Printer<float>::stream(s, indent + "  ", v.r_fk_pX);
    s << indent << "r_fk_pY: ";
    Printer<float>::stream(s, indent + "  ", v.r_fk_pY);
    s << indent << "r_fk_pZ: ";
    Printer<float>::stream(s, indent + "  ", v.r_fk_pZ);
    s << indent << "l_fk_pX: ";
    Printer<float>::stream(s, indent + "  ", v.l_fk_pX);
    s << indent << "l_fk_pY: ";
    Printer<float>::stream(s, indent + "  ", v.l_fk_pY);
    s << indent << "l_fk_pZ: ";
    Printer<float>::stream(s, indent + "  ", v.l_fk_pZ);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVE_DUALARM_MESSAGE_FK_LOCA_H
