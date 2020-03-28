// Generated by gencpp from file comp0037_mapper/MapUpdate.msg
// DO NOT EDIT!


#ifndef COMP0037_MAPPER_MESSAGE_MAPUPDATE_H
#define COMP0037_MAPPER_MESSAGE_MAPUPDATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace comp0037_mapper
{
template <class ContainerAllocator>
struct MapUpdate_
{
  typedef MapUpdate_<ContainerAllocator> Type;

  MapUpdate_()
    : header()
    , isPriorMap(false)
    , scale(0.0)
    , extentInCells()
    , resolution(0.0)
    , occupancyGrid()
    , deltaOccupancyGrid()  {
    }
  MapUpdate_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , isPriorMap(false)
    , scale(0.0)
    , extentInCells(_alloc)
    , resolution(0.0)
    , occupancyGrid(_alloc)
    , deltaOccupancyGrid(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _isPriorMap_type;
  _isPriorMap_type isPriorMap;

   typedef float _scale_type;
  _scale_type scale;

   typedef std::vector<int16_t, typename ContainerAllocator::template rebind<int16_t>::other >  _extentInCells_type;
  _extentInCells_type extentInCells;

   typedef float _resolution_type;
  _resolution_type resolution;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _occupancyGrid_type;
  _occupancyGrid_type occupancyGrid;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _deltaOccupancyGrid_type;
  _deltaOccupancyGrid_type deltaOccupancyGrid;





  typedef boost::shared_ptr< ::comp0037_mapper::MapUpdate_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::comp0037_mapper::MapUpdate_<ContainerAllocator> const> ConstPtr;

}; // struct MapUpdate_

typedef ::comp0037_mapper::MapUpdate_<std::allocator<void> > MapUpdate;

typedef boost::shared_ptr< ::comp0037_mapper::MapUpdate > MapUpdatePtr;
typedef boost::shared_ptr< ::comp0037_mapper::MapUpdate const> MapUpdateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::comp0037_mapper::MapUpdate_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::comp0037_mapper::MapUpdate_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace comp0037_mapper

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'comp0037_mapper': ['/home/ros_user/COMP0037_CW2/src/comp0037_mapper/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::comp0037_mapper::MapUpdate_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::comp0037_mapper::MapUpdate_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::comp0037_mapper::MapUpdate_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::comp0037_mapper::MapUpdate_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::comp0037_mapper::MapUpdate_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::comp0037_mapper::MapUpdate_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::comp0037_mapper::MapUpdate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bb9eab5859acbeac865abd611e41d4b8";
  }

  static const char* value(const ::comp0037_mapper::MapUpdate_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbb9eab5859acbeacULL;
  static const uint64_t static_value2 = 0x865abd611e41d4b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::comp0037_mapper::MapUpdate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "comp0037_mapper/MapUpdate";
  }

  static const char* value(const ::comp0037_mapper::MapUpdate_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::comp0037_mapper::MapUpdate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
bool isPriorMap\n\
\n\
float32 scale\n\
int16[] extentInCells\n\
float32 resolution\n\
\n\
float32[] occupancyGrid\n\
float32[] deltaOccupancyGrid\n\
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
";
  }

  static const char* value(const ::comp0037_mapper::MapUpdate_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::comp0037_mapper::MapUpdate_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.isPriorMap);
      stream.next(m.scale);
      stream.next(m.extentInCells);
      stream.next(m.resolution);
      stream.next(m.occupancyGrid);
      stream.next(m.deltaOccupancyGrid);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MapUpdate_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::comp0037_mapper::MapUpdate_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::comp0037_mapper::MapUpdate_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "isPriorMap: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isPriorMap);
    s << indent << "scale: ";
    Printer<float>::stream(s, indent + "  ", v.scale);
    s << indent << "extentInCells[]" << std::endl;
    for (size_t i = 0; i < v.extentInCells.size(); ++i)
    {
      s << indent << "  extentInCells[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.extentInCells[i]);
    }
    s << indent << "resolution: ";
    Printer<float>::stream(s, indent + "  ", v.resolution);
    s << indent << "occupancyGrid[]" << std::endl;
    for (size_t i = 0; i < v.occupancyGrid.size(); ++i)
    {
      s << indent << "  occupancyGrid[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.occupancyGrid[i]);
    }
    s << indent << "deltaOccupancyGrid[]" << std::endl;
    for (size_t i = 0; i < v.deltaOccupancyGrid.size(); ++i)
    {
      s << indent << "  deltaOccupancyGrid[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.deltaOccupancyGrid[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // COMP0037_MAPPER_MESSAGE_MAPUPDATE_H