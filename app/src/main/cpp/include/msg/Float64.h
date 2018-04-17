//
// Created by zhibo on 18-4-17.
//

#ifndef GMAPPING_JAVA_CPP_FLOAT64_H
#define GMAPPING_JAVA_CPP_FLOAT64_H

#include <msg/common.h>
namespace std_msgs
{
    template <class ContainerAllocator>
    struct Float64_
    {
        typedef Float64_<ContainerAllocator> Type;

        Float64_()
                : data(0.0)  {
        }
        Float64_(const ContainerAllocator& _alloc)
                : data(0.0)  {
            (void)_alloc;
        }



        typedef double _data_type;
        _data_type data;




        typedef boost::shared_ptr< ::std_msgs::Float64_<ContainerAllocator> > Ptr;
        typedef boost::shared_ptr< ::std_msgs::Float64_<ContainerAllocator> const> ConstPtr;

    }; // struct Float64_

    typedef ::std_msgs::Float64_<std::allocator<void> > Float64;

    typedef boost::shared_ptr< ::std_msgs::Float64 > Float64Ptr;
    typedef boost::shared_ptr< ::std_msgs::Float64 const> Float64ConstPtr;

// constants requiring out of line definition



    template<typename ContainerAllocator>
    std::ostream& operator<<(std::ostream& s, const ::std_msgs::Float64_<ContainerAllocator> & v)
    {
//        ros::message_operations::Printer< ::std_msgs::Float64_<ContainerAllocator> >::stream(s, "", v);
        return s;
    }

} // namespace std_msgs
#endif //GMAPPING_JAVA_CPP_FLOAT64_H
