//
// Created by zhibo on 18-4-17.
//

#ifndef GMAPPING_JAVA_CPP_MAPMETADATA_H
#define GMAPPING_JAVA_CPP_MAPMETADATA_H

#include <msg/common.h>
#include <msg/geometry_msgs/Pose.h>

namespace nav_msgs
{
    template <class ContainerAllocator>
    struct MapMetaData_
    {
        typedef MapMetaData_<ContainerAllocator> Type;

        MapMetaData_()
                : map_load_time()
                , resolution(0.0)
                , width(0)
                , height(0)
                , origin()  {
        }
        MapMetaData_(const ContainerAllocator& _alloc)
                : map_load_time()
                , resolution(0.0)
                , width(0)
                , height(0)
                , origin(_alloc)  {
            (void)_alloc;
        }



//        typedef ros::Time _map_load_time_type;
        typedef std::time_t _map_load_time_type;
        _map_load_time_type map_load_time;

        typedef float _resolution_type;
        _resolution_type resolution;

        typedef uint32_t _width_type;
        _width_type width;

        typedef uint32_t _height_type;
        _height_type height;

        typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _origin_type;
        _origin_type origin;




        typedef boost::shared_ptr< ::nav_msgs::MapMetaData_<ContainerAllocator> > Ptr;
        typedef boost::shared_ptr< ::nav_msgs::MapMetaData_<ContainerAllocator> const> ConstPtr;

    }; // struct MapMetaData_

    typedef ::nav_msgs::MapMetaData_<std::allocator<void> > MapMetaData;

    typedef boost::shared_ptr< ::nav_msgs::MapMetaData > MapMetaDataPtr;
    typedef boost::shared_ptr< ::nav_msgs::MapMetaData const> MapMetaDataConstPtr;

// constants requiring out of line definition



    template<typename ContainerAllocator>
    std::ostream& operator<<(std::ostream& s, const ::nav_msgs::MapMetaData_<ContainerAllocator> & v)
    {
//        ros::message_operations::Printer< ::nav_msgs::MapMetaData_<ContainerAllocator> >::stream(s, "", v);
        return s;
    }

} // namespace nav_msgs
#endif //GMAPPING_JAVA_CPP_MAPMETADATA_H
