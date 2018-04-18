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
    void isWhitespace(std::string s);
    bool getParam();

private:
    int laser_count_;
    int throttle_scans_;

    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double minimum_score_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
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

    unsigned long int seed_;

    double transform_publish_period_;
    double tf_delay_;

};


#endif //GMAPPING_JAVA_CPP_GSLAM_H
