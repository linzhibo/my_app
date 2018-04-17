//
// Created by zhibo on 18-4-17.
//

#ifndef GMAPPING_JAVA_CPP_OCCUPANCYGRID_H
#define GMAPPING_JAVA_CPP_OCCUPANCYGRID_H

#include <msg/Header.h>
#include <msg/MapMetaData.h>

namespace nav_msgs
{
    template <class ContainerAllocator>
    struct OccupancyGrid_
    {
        typedef OccupancyGrid_<ContainerAllocator> Type;

        OccupancyGrid_()
                : header()
                , info()
                , data()  {
        }
        OccupancyGrid_(const ContainerAllocator& _alloc)
                : header(_alloc)
                , info(_alloc)
                , data(_alloc)  {
            (void)_alloc;
        }



        typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
        _header_type header;

        typedef  ::nav_msgs::MapMetaData_<ContainerAllocator>  _info_type;
        _info_type info;

        typedef std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  _data_type;
        _data_type data;




        typedef boost::shared_ptr< ::nav_msgs::OccupancyGrid_<ContainerAllocator> > Ptr;
        typedef boost::shared_ptr< ::nav_msgs::OccupancyGrid_<ContainerAllocator> const> ConstPtr;

    }; // struct OccupancyGrid_

    typedef ::nav_msgs::OccupancyGrid_<std::allocator<void> > OccupancyGrid;

    typedef boost::shared_ptr< ::nav_msgs::OccupancyGrid > OccupancyGridPtr;
    typedef boost::shared_ptr< ::nav_msgs::OccupancyGrid const> OccupancyGridConstPtr;

// constants requiring out of line definition



    template<typename ContainerAllocator>
    std::ostream& operator<<(std::ostream& s, const ::nav_msgs::OccupancyGrid_<ContainerAllocator> & v)
    {
//        ros::message_operations::Printer< ::nav_msgs::OccupancyGrid_<ContainerAllocator> >::stream(s, "", v);
        return s;
    }

} // namespace nav_msgs

#endif //GMAPPING_JAVA_CPP_OCCUPANCYGRID_H
