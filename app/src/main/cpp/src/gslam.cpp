//
// Created by zhibo on 18-4-18.
//

#include "gslam.h"


GSlam::GSlam() :
{
    seed_ = time(NULL);
    init();
}

GSlam::GSlam(unsigned long int seed, unsigned long int max_duration_buffer)
{
    init();
}

bool GSlam::isWhitespace(std::string s){
    for(int index = 0; index < s.length(); index++){
        if(!std::isspace(s[index]))
            return false;
    }
    return true;
}
bool GSlam::getParam (std::map<std::string, double> param, std::string name, double &val)
{
    if(param[name])
    {
        val = param[name];
        return true;
    }
    return false;
}
void GSlam::init()
{
    std::map<std::string, double> slam_param;
    std::ifstream file("param.txt");
    std::string line;
    std::string data;
    double      val1;
    while(std::getline(file,line))
    {
        if (!line.empty() && !isWhitespace(line))
        {
            std::stringstream linestream(line);
            std::getline(linestream, data, ' ');
            linestream >> val1;
            slam_param.insert(std::pair<std::string, double>(data, val1));
        }
    }
    // Parameters used by our GMapping wrapper
    if(!getParam(slam_param,"throttle_scans", throttle_scans_))
        throttle_scans_ = 1;
    
    double tmp;
    if(!getParam(slam_param,"map_update_interval", tmp))
        tmp = 5.0;

    // Parameters used by GMapping itself
    maxUrange_ = 0.0;  maxRange_ = 0.0; // preliminary default, will be set in initMapper()
    if(!getParam(slam_param,"minimumScore", minimum_score_))
        minimum_score_ = 0;
    if(!getParam(slam_param,"sigma", sigma_))
        sigma_ = 0.05;
    if(!getParam(slam_param,"kernelSize", kernelSize_))
        kernelSize_ = 1;
    if(!getParam(slam_param,"lstep", lstep_))
        lstep_ = 0.05;
    if(!getParam(slam_param,"astep", astep_))
        astep_ = 0.05;
    if(!getParam(slam_param,"iterations", iterations_))
        iterations_ = 5;
    if(!getParam(slam_param,"lsigma", lsigma_))
        lsigma_ = 0.075;
    if(!getParam(slam_param,"ogain", ogain_))
        ogain_ = 3.0;
    if(!getParam(slam_param,"lskip", lskip_))
        lskip_ = 0;
    if(!getParam(slam_param,"srr", srr_))
        srr_ = 0.1;
    if(!getParam(slam_param,"srt", srt_))
        srt_ = 0.2;
    if(!getParam(slam_param,"str", str_))
        str_ = 0.1;
    if(!getParam(slam_param,"stt", stt_))
        stt_ = 0.2;
    if(!getParam(slam_param,"linearUpdate", linearUpdate_))
        linearUpdate_ = 1.0;
    if(!getParam(slam_param,"angularUpdate", angularUpdate_))
        angularUpdate_ = 0.5;
    if(!getParam(slam_param,"temporalUpdate", temporalUpdate_))
        temporalUpdate_ = -1.0;
    if(!getParam(slam_param,"resampleThreshold", resampleThreshold_))
        resampleThreshold_ = 0.5;
    if(!getParam(slam_param,"particles", particles_))
        particles_ = 30;
    if(!getParam(slam_param,"xmin", xmin_))
        xmin_ = -100.0;
    if(!getParam(slam_param,"ymin", ymin_))
        ymin_ = -100.0;
    if(!getParam(slam_param,"xmax", xmax_))
        xmax_ = 100.0;
    if(!getParam(slam_param,"ymax", ymax_))
        ymax_ = 100.0;
    if(!getParam(slam_param,"delta", delta_))
        delta_ = 0.05;
    if(!getParam(slam_param,"occ_thresh", occ_thresh_))
        occ_thresh_ = 0.25;
    if(!getParam(slam_param,"llsamplerange", llsamplerange_))
        llsamplerange_ = 0.01;
    if(!getParam(slam_param,"llsamplestep", llsamplestep_))
        llsamplestep_ = 0.01;
    if(!getParam(slam_param,"lasamplerange", lasamplerange_))
        lasamplerange_ = 0.005;
    if(!getParam(slam_param,"lasamplestep", lasamplestep_))
        lasamplestep_ = 0.005;

    if(!getParam(slam_param,"tf_delay", tf_delay_))
        tf_delay_ = transform_publish_period_;
}