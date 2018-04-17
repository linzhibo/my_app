//
// Created by zhibo on 18-4-17.
//

#ifndef GMAPPING_JAVA_CPP_POSE_H
#define GMAPPING_JAVA_CPP_POSE_H

#include <msg/common.h>
#include <msg/geometry_msgs/Point.h>
#include <msg/geometry_msgs/Quaternion.h>

namespace geometry_msgs
{
    template <class ContainerAllocator>
    struct Pose_
    {
        typedef Pose_<ContainerAllocator> Type;

        Pose_()
                : position()
                , orientation()  {
        }
        Pose_(const ContainerAllocator& _alloc)
                : position(_alloc)
                , orientation(_alloc)  {
            (void)_alloc;
        }



        typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
        _position_type position;

        typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _orientation_type;
        _orientation_type orientation;




        typedef boost::shared_ptr< ::geometry_msgs::Pose_<ContainerAllocator> > Ptr;
        typedef boost::shared_ptr< ::geometry_msgs::Pose_<ContainerAllocator> const> ConstPtr;

    }; // struct Pose_

    typedef ::geometry_msgs::Pose_<std::allocator<void> > Pose;

    typedef boost::shared_ptr< ::geometry_msgs::Pose > PosePtr;
    typedef boost::shared_ptr< ::geometry_msgs::Pose const> PoseConstPtr;

// constants requiring out of line definition



    template<typename ContainerAllocator>
    std::ostream& operator<<(std::ostream& s, const ::geometry_msgs::Pose_<ContainerAllocator> & v)
    {
//        ros::message_operations::Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, "", v);
        return s;
    }

} // namespace geometry_msgs
#endif //GMAPPING_JAVA_CPP_POSE_H
