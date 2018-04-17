//
// Created by zhibo on 18-4-17.
//

#ifndef GMAPPING_JAVA_CPP_POSEWITHCOVARIANCE_H
#define GMAPPING_JAVA_CPP_POSEWITHCOVARIANCE_H
#include <msg/geometry_msgs/Pose.h>

namespace geometry_msgs
{
    template <class ContainerAllocator>
    struct PoseWithCovariance_
    {
        typedef PoseWithCovariance_<ContainerAllocator> Type;

        PoseWithCovariance_()
                : pose()
                , covariance()  {
            covariance.assign(0.0);
        }
        PoseWithCovariance_(const ContainerAllocator& _alloc)
                : pose(_alloc)
                , covariance()  {
            (void)_alloc;
            covariance.assign(0.0);
        }



        typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
        _pose_type pose;

        typedef boost::array<double, 36>  _covariance_type;
        _covariance_type covariance;




        typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> > Ptr;
        typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> const> ConstPtr;

    }; // struct PoseWithCovariance_

    typedef ::geometry_msgs::PoseWithCovariance_<std::allocator<void> > PoseWithCovariance;

    typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovariance > PoseWithCovariancePtr;
    typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovariance const> PoseWithCovarianceConstPtr;

// constants requiring out of line definition



    template<typename ContainerAllocator>
    std::ostream& operator<<(std::ostream& s, const ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> & v)
    {
//        ros::message_operations::Printer< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> >::stream(s, "", v);
        return s;
    }

} // namespace geometry_msgs
#endif //GMAPPING_JAVA_CPP_POSEWITHCOVARIANCE_H
