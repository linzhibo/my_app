//
// Created by zhibo on 18-4-17.
//

#ifndef GMAPPING_JAVA_CPP_QUATERNION_H
#define GMAPPING_JAVA_CPP_QUATERNION_H

#include <msg/common.h>
namespace geometry_msgs
{
    template <class ContainerAllocator>
    struct Quaternion_
    {
        typedef Quaternion_<ContainerAllocator> Type;

        Quaternion_()
                : x(0.0)
                , y(0.0)
                , z(0.0)
                , w(0.0)  {
        }
        Quaternion_(const ContainerAllocator& _alloc)
                : x(0.0)
                , y(0.0)
                , z(0.0)
                , w(0.0)  {
            (void)_alloc;
        }



        typedef double _x_type;
        _x_type x;

        typedef double _y_type;
        _y_type y;

        typedef double _z_type;
        _z_type z;

        typedef double _w_type;
        _w_type w;




        typedef boost::shared_ptr< ::geometry_msgs::Quaternion_<ContainerAllocator> > Ptr;
        typedef boost::shared_ptr< ::geometry_msgs::Quaternion_<ContainerAllocator> const> ConstPtr;

    }; // struct Quaternion_

    typedef ::geometry_msgs::Quaternion_<std::allocator<void> > Quaternion;

    typedef boost::shared_ptr< ::geometry_msgs::Quaternion > QuaternionPtr;
    typedef boost::shared_ptr< ::geometry_msgs::Quaternion const> QuaternionConstPtr;

// constants requiring out of line definition



    template<typename ContainerAllocator>
    std::ostream& operator<<(std::ostream& s, const ::geometry_msgs::Quaternion_<ContainerAllocator> & v)
    {
//        ros::message_operations::Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, "", v);
        return s;
    }

} // namespace geometry_msgs
#endif //GMAPPING_JAVA_CPP_QUATERNION_H
