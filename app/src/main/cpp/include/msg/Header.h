//
// Created by zhibo on 18-4-17.
//

#ifndef GMAPPING_JAVA_CPP_HEADER_H
#define GMAPPING_JAVA_CPP_HEADER_H
#include <msg/common.h>



namespace std_msgs
{
    template <class ContainerAllocator>
    struct Header_
    {
        typedef Header_<ContainerAllocator> Type;

        Header_()
                : seq(0)
                , stamp()
                , frame_id()  {
        }
        Header_(const ContainerAllocator& _alloc)
                : seq(0)
                , stamp()
                , frame_id(_alloc)  {
            (void)_alloc;
        }



        typedef uint32_t _seq_type;
        _seq_type seq;

//        typedef ros::Time _stamp_type;
        typedef std::time_t _stamp_type;
        _stamp_type stamp;

        typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _frame_id_type;
        _frame_id_type frame_id;




        typedef boost::shared_ptr< ::std_msgs::Header_<ContainerAllocator> > Ptr;
        typedef boost::shared_ptr< ::std_msgs::Header_<ContainerAllocator> const> ConstPtr;

    }; // struct Header_

    typedef ::std_msgs::Header_<std::allocator<void> > Header;

    typedef boost::shared_ptr< ::std_msgs::Header > HeaderPtr;
    typedef boost::shared_ptr< ::std_msgs::Header const> HeaderConstPtr;

// constants requiring out of line definition



    template<typename ContainerAllocator>
    std::ostream& operator<<(std::ostream& s, const ::std_msgs::Header_<ContainerAllocator> & v)
    {
        return s;
    }

} // namespace std_msgs


#endif //GMAPPING_JAVA_CPP_HEADER_H
