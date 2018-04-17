//
// Created by zhibo on 18-4-17.
//

#ifndef GMAPPING_JAVA_CPP_LASERSCAN_H
#define GMAPPING_JAVA_CPP_LASERSCAN_H

#include <msg/common.h>

#include <msg/Header.h>

namespace sensor_msgs
{
    template <class ContainerAllocator>
    struct LaserScan_
    {
        typedef LaserScan_<ContainerAllocator> Type;

        LaserScan_()
                : header()
                , angle_min(0.0)
                , angle_max(0.0)
                , angle_increment(0.0)
                , time_increment(0.0)
                , scan_time(0.0)
                , range_min(0.0)
                , range_max(0.0)
                , ranges()
                , intensities()  {
        }
        LaserScan_(const ContainerAllocator& _alloc)
                : header(_alloc)
                , angle_min(0.0)
                , angle_max(0.0)
                , angle_increment(0.0)
                , time_increment(0.0)
                , scan_time(0.0)
                , range_min(0.0)
                , range_max(0.0)
                , ranges(_alloc)
                , intensities(_alloc)  {
            (void)_alloc;
        }



        typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
        _header_type header;

        typedef float _angle_min_type;
        _angle_min_type angle_min;

        typedef float _angle_max_type;
        _angle_max_type angle_max;

        typedef float _angle_increment_type;
        _angle_increment_type angle_increment;

        typedef float _time_increment_type;
        _time_increment_type time_increment;

        typedef float _scan_time_type;
        _scan_time_type scan_time;

        typedef float _range_min_type;
        _range_min_type range_min;

        typedef float _range_max_type;
        _range_max_type range_max;

        typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _ranges_type;
        _ranges_type ranges;

        typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _intensities_type;
        _intensities_type intensities;




        typedef boost::shared_ptr< ::sensor_msgs::LaserScan_<ContainerAllocator> > Ptr;
        typedef boost::shared_ptr< ::sensor_msgs::LaserScan_<ContainerAllocator> const> ConstPtr;

    }; // struct LaserScan_

    typedef ::sensor_msgs::LaserScan_<std::allocator<void> > LaserScan;

    typedef boost::shared_ptr< ::sensor_msgs::LaserScan > LaserScanPtr;
    typedef boost::shared_ptr< ::sensor_msgs::LaserScan const> LaserScanConstPtr;

// constants requiring out of line definition



    template<typename ContainerAllocator>
    std::ostream& operator<<(std::ostream& s, const ::sensor_msgs::LaserScan_<ContainerAllocator> & v)
    {
//        ros::message_operations::Printer< ::sensor_msgs::LaserScan_<ContainerAllocator> >::stream(s, "", v);
        return s;
    }

} // namespace sensor_msgs

#endif //GMAPPING_JAVA_CPP_LASERSCAN_H
