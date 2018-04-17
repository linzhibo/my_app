//
// Created by zhibo on 18-4-17.
//

#ifndef GMAPPING_JAVA_CPP_POSEWITHCOVARIANCESTAMPED_H
#define GMAPPING_JAVA_CPP_POSEWITHCOVARIANCESTAMPED_H

#include <msg/Header.h>
#include <msg/geometry_msgs/PoseWithCovariance.h>

namespace geometry_msgs
{
    template <class ContainerAllocator>
    struct PoseWithCovarianceStamped_
    {
        typedef PoseWithCovarianceStamped_<ContainerAllocator> Type;

        PoseWithCovarianceStamped_()
                : header()
                , pose()  {
        }
        PoseWithCovarianceStamped_(const ContainerAllocator& _alloc)
                : header(_alloc)
                , pose(_alloc)  {
            (void)_alloc;
        }



        typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
        _header_type header;

        typedef  ::geometry_msgs::PoseWithCovariance_<ContainerAllocator>  _pose_type;
        _pose_type pose;




        typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator> > Ptr;
        typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator> const> ConstPtr;

    }; // struct PoseWithCovarianceStamped_

    typedef ::geometry_msgs::PoseWithCovarianceStamped_<std::allocator<void> > PoseWithCovarianceStamped;

    typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovarianceStamped > PoseWithCovarianceStampedPtr;
    typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovarianceStamped const> PoseWithCovarianceStampedConstPtr;

// constants requiring out of line definition



    template<typename ContainerAllocator>
    std::ostream& operator<<(std::ostream& s, const ::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator> & v)
    {
//        ros::message_operations::Printer< ::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator> >::stream(s, "", v);
        return s;
    }

} // namespace geometry_msgs
#endif //GMAPPING_JAVA_CPP_POSEWITHCOVARIANCESTAMPED_H
