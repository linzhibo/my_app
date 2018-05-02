//
// Created by zhibo on 18-4-18.
//

#ifndef GMAPPING_JAVA_CPP_GSLAM_H
#define GMAPPING_JAVA_CPP_GSLAM_H
#include <iostream>
#include <string>
#include <map>
#include <fstream>
#include <sstream>
#include <msg/GetMap.h>
#include <msg/LaserScan.h>
#include <msg/Float64.h>
#include <gmapping/gridfastslam/gridslamprocessor.h>
#include <gmapping/sensor/sensor_base/sensor.h>

#include <boost/thread.hpp>

class GSlam {
public:
    GSlam();
    GSlam(unsigned long int seed, unsigned long int max_duration_buffer);

    ~GSlam();

    void init();
    bool isWhitespace(std::string s);
    bool getParam(std::map<std::string, double> param, std::string name, double &val);
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, time_t t);
    void startLiveSlam(const sensor_msgs::LaserScan::ConstPtr& scan);
//    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void publishTransform();

    void publishLoop();

private:
    //functions
    double computePoseEntropy();    //can be used later
    void updateMap(const sensor_msgs::LaserScan& scan);
    bool initMapper(const sensor_msgs::LaserScan& scan);
    bool addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose);

    //variables
    int laser_count_;
    double throttle_scans_; //int
    bool got_first_scan_;
    bool got_map_;
    nav_msgs::GetMap::Response map_;
//    bool do_reverse_range_;

    GMapping::GridSlamProcessor* gsp_;
    GMapping::RangeSensor* gsp_laser_;
    GMapping::OdometrySensor* gsp_odom_;

    boost::thread* transform_thread_;
    boost::mutex map_mutex_;
    unsigned int gsp_laser_beam_count_;

//    std::string laser_frame_;


    // Parameters used by GMapping
    std::vector<double> laser_angles_;
    std::map<std::string, double> slam_param;
    time_t map_update_interval_time_t;
    double map_update_interval_;
    double maxRange_;
    double maxUrange_;
    double minimum_score_;
    double sigma_;
    double kernelSize_;     //int
    double lstep_;
    double astep_;
    double iterations_;     //int
    double lsigma_;
    double ogain_;
    double lskip_;          //int
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    double particles_;  //int
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;

//    unsigned long int seed_;
    long int seed_;
    double transform_publish_period_;
    double tf_delay_;

};


#endif //GMAPPING_JAVA_CPP_GSLAM_H
