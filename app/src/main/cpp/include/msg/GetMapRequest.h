//
// Created by zhibo on 18-4-17.
//

#ifndef GMAPPING_JAVA_CPP_GETMAPREQUEST_H
#define GMAPPING_JAVA_CPP_GETMAPREQUEST_H

#include <msg/common.h>


namespace nav_msgs
{
    template <class ContainerAllocator>
    struct GetMapRequest_
    {
        typedef GetMapRequest_<ContainerAllocator> Type;

        GetMapRequest_()
        {
        }
        GetMapRequest_(const ContainerAllocator& _alloc)
        {
            (void)_alloc;
        }






        typedef boost::shared_ptr< ::nav_msgs::GetMapRequest_<ContainerAllocator> > Ptr;
        typedef boost::shared_ptr< ::nav_msgs::GetMapRequest_<ContainerAllocator> const> ConstPtr;

    }; // struct GetMapRequest_

    typedef ::nav_msgs::GetMapRequest_<std::allocator<void> > GetMapRequest;

    typedef boost::shared_ptr< ::nav_msgs::GetMapRequest > GetMapRequestPtr;
    typedef boost::shared_ptr< ::nav_msgs::GetMapRequest const> GetMapRequestConstPtr;

// constants requiring out of line definition



    template<typename ContainerAllocator>
    std::ostream& operator<<(std::ostream& s, const ::nav_msgs::GetMapRequest_<ContainerAllocator> & v)
    {
//        ros::message_operations::Printer< ::nav_msgs::GetMapRequest_<ContainerAllocator> >::stream(s, "", v);
        return s;
    }

} // namespace nav_msgs
#endif //GMAPPING_JAVA_CPP_GETMAPREQUEST_H
