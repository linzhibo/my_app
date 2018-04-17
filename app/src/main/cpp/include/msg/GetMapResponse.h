//
// Created by zhibo on 18-4-17.
//

#ifndef GMAPPING_JAVA_CPP_GETMAPRESPONSE_H
#define GMAPPING_JAVA_CPP_GETMAPRESPONSE_H

#include <msg/common.h>
#include <msg/OccupancyGrid.h>

namespace nav_msgs
{
    template <class ContainerAllocator>
    struct GetMapResponse_
    {
        typedef GetMapResponse_<ContainerAllocator> Type;

        GetMapResponse_()
                : map()  {
        }
        GetMapResponse_(const ContainerAllocator& _alloc)
                : map(_alloc)  {
            (void)_alloc;
        }



        typedef  ::nav_msgs::OccupancyGrid_<ContainerAllocator>  _map_type;
        _map_type map;




        typedef boost::shared_ptr< ::nav_msgs::GetMapResponse_<ContainerAllocator> > Ptr;
        typedef boost::shared_ptr< ::nav_msgs::GetMapResponse_<ContainerAllocator> const> ConstPtr;

    }; // struct GetMapResponse_

    typedef ::nav_msgs::GetMapResponse_<std::allocator<void> > GetMapResponse;

    typedef boost::shared_ptr< ::nav_msgs::GetMapResponse > GetMapResponsePtr;
    typedef boost::shared_ptr< ::nav_msgs::GetMapResponse const> GetMapResponseConstPtr;

// constants requiring out of line definition



    template<typename ContainerAllocator>
    std::ostream& operator<<(std::ostream& s, const ::nav_msgs::GetMapResponse_<ContainerAllocator> & v)
    {
//        ros::message_operations::Printer< ::nav_msgs::GetMapResponse_<ContainerAllocator> >::stream(s, "", v);
        return s;
    }

} // namespace nav_msgs
#endif //GMAPPING_JAVA_CPP_GETMAPRESPONSE_H
