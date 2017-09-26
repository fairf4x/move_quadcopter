// Generated by gencpp from file move_quadcopter/NavigateGoal.msg
// DO NOT EDIT!


#ifndef MOVE_QUADCOPTER_MESSAGE_NAVIGATEGOAL_H
#define MOVE_QUADCOPTER_MESSAGE_NAVIGATEGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace move_quadcopter
{
template <class ContainerAllocator>
struct NavigateGoal_
{
  typedef NavigateGoal_<ContainerAllocator> Type;

  NavigateGoal_()
    : target_pos()  {
    }
  NavigateGoal_(const ContainerAllocator& _alloc)
    : target_pos(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _target_pos_type;
  _target_pos_type target_pos;




  typedef boost::shared_ptr< ::move_quadcopter::NavigateGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::move_quadcopter::NavigateGoal_<ContainerAllocator> const> ConstPtr;

}; // struct NavigateGoal_

typedef ::move_quadcopter::NavigateGoal_<std::allocator<void> > NavigateGoal;

typedef boost::shared_ptr< ::move_quadcopter::NavigateGoal > NavigateGoalPtr;
typedef boost::shared_ptr< ::move_quadcopter::NavigateGoal const> NavigateGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::move_quadcopter::NavigateGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::move_quadcopter::NavigateGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace move_quadcopter

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'move_quadcopter': ['/home/fairfax/ROS/bebop/devel/share/move_quadcopter/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::move_quadcopter::NavigateGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::move_quadcopter::NavigateGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::move_quadcopter::NavigateGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::move_quadcopter::NavigateGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::move_quadcopter::NavigateGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::move_quadcopter::NavigateGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::move_quadcopter::NavigateGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "332449a2fcdc11c82bafadb9df2b64d8";
  }

  static const char* value(const ::move_quadcopter::NavigateGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x332449a2fcdc11c8ULL;
  static const uint64_t static_value2 = 0x2bafadb9df2b64d8ULL;
};

template<class ContainerAllocator>
struct DataType< ::move_quadcopter::NavigateGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "move_quadcopter/NavigateGoal";
  }

  static const char* value(const ::move_quadcopter::NavigateGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::move_quadcopter::NavigateGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# goal point for navigation action\n\
geometry_msgs/Point target_pos\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::move_quadcopter::NavigateGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::move_quadcopter::NavigateGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.target_pos);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavigateGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::move_quadcopter::NavigateGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::move_quadcopter::NavigateGoal_<ContainerAllocator>& v)
  {
    s << indent << "target_pos: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.target_pos);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVE_QUADCOPTER_MESSAGE_NAVIGATEGOAL_H
