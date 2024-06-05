// Generated by gencpp from file costmap_display/GetApproachingPoseRequest.msg
// DO NOT EDIT!


#ifndef COSTMAP_DISPLAY_MESSAGE_GETAPPROACHINGPOSEREQUEST_H
#define COSTMAP_DISPLAY_MESSAGE_GETAPPROACHINGPOSEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace costmap_display
{
template <class ContainerAllocator>
struct GetApproachingPoseRequest_
{
  typedef GetApproachingPoseRequest_<ContainerAllocator> Type;

  GetApproachingPoseRequest_()
    {
    }
  GetApproachingPoseRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetApproachingPoseRequest_

typedef ::costmap_display::GetApproachingPoseRequest_<std::allocator<void> > GetApproachingPoseRequest;

typedef boost::shared_ptr< ::costmap_display::GetApproachingPoseRequest > GetApproachingPoseRequestPtr;
typedef boost::shared_ptr< ::costmap_display::GetApproachingPoseRequest const> GetApproachingPoseRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace costmap_display

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "costmap_display/GetApproachingPoseRequest";
  }

  static const char* value(const ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetApproachingPoseRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::costmap_display::GetApproachingPoseRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // COSTMAP_DISPLAY_MESSAGE_GETAPPROACHINGPOSEREQUEST_H