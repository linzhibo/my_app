//
// Created by zhibo on 18-4-18.
//

#include "gslam.h"
// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

GSlam::GSlam()
{
    seed_ = time(NULL);
    init();
}

GSlam::GSlam(unsigned long int seed, unsigned long int max_duration_buffer)
{
    init();
}

GSlam::~GSlam()
{
    if(!slam_param.empty())
        slam_param.clear();

    delete gsp_;
    if(gsp_laser_)
        delete gsp_laser_;
    if(gsp_odom_)
        delete gsp_odom_;
}

bool GSlam::isWhitespace(std::string s){
    for(int index = 0; index < s.length(); index++){
        if(!std::isspace(s[index]))
            return false;
    }
    return true;
}   //ok
bool GSlam::getParam (std::map<std::string, double> param, std::string name, double &val)   //ok
{
    if(param[name])
    {
        val = param[name];
        return true;
    }
    return false;
}
void GSlam::init() //ok
{
    gsp_ = new GMapping::GridSlamProcessor();
    gsp_laser_ = NULL;
    gsp_odom_ = NULL;
    got_first_scan_ = false;
    got_map_ = false;

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
    // Parameters used by  GMapping wrapper
    if(!getParam(slam_param,"throttle_scans", throttle_scans_))
        throttle_scans_ = 1;

    if(!getParam(slam_param,"map_update_interval", map_update_interval_))
        map_update_interval_ = 5.0;
//    map_update_interval_time_t = map_update_interval_;

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

void GSlam::startLiveSlam(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if(!got_first_scan_)
    {
        if(!initMapper(*scan))
            return;
        got_first_scan_ = true;
    }
    GMapping::OrientedPoint odom_pose;

    time_t last_map_update;
    time(&last_map_update);

    if(addScan(*scan, odom_pose))
    {
        std::cout<<"scan processed"<<std::endl;

//        GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;

        //transform laser to map, odom to laser

        if(!got_map_ || double((scan->header.stamp - last_map_update)) > map_update_interval_)
        {
            updateMap(*scan);
            last_map_update = scan->header.stamp;

        }
    }
//    transform_thread_ = new boost::thread(boost::bind(&GSlam::publishLoop, this, transform_publish_period_));
}

bool GSlam::initMapper(const sensor_msgs::LaserScan& scan) //ok with maybetodo
{
    //TODO Maybe
    //get centered_laser_pose_

//    laser_frame_ = scan.header.frame_id;
    gsp_laser_beam_count_ = unsigned int (scan.ranges.size());
    laser_angles_.resize(scan.ranges.size());
    // Make sure angles are started so that they are centered
    double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
    for(unsigned int i=0; i<scan.ranges.size(); ++i)
    {
        laser_angles_[i]=theta;
        theta += std::fabs(scan.angle_increment);
    }
    GMapping::OrientedPoint gmap_pose(0, 0, 0);
    if(!getParam(slam_param,"maxRange", maxRange_))
        maxRange_ = scan.range_max - 0.01;
    if(!getParam(slam_param,"maxUrange", maxUrange_))
        maxUrange_ = maxRange_;

    gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                           gsp_laser_beam_count_,
                                           fabs(scan.angle_increment),
                                           gmap_pose,
                                           0.0,
                                           maxRange_);
    GMapping::SensorMap smap;
    smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
    gsp_->setSensorMap(smap);
    gsp_odom_ = new GMapping::OdometrySensor("odom");
    GMapping::OrientedPoint initialPose;
    if(!getOdomPose(initialPose, scan.header.stamp))
    {
        std::cout<<"Unable to determine inital pose of laser! Starting point will be set to zero."<<std::endl;
        initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
    }

    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                                int(kernelSize_), lstep_, astep_, int(iterations_),
                                lsigma_, ogain_, unsigned int(lskip_));

    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    gsp_->setUpdatePeriod(temporalUpdate_);
    gsp_->setgenerateMap(false);
    gsp_->GridSlamProcessor::init(unsigned int(particles_), xmin_, ymin_, xmax_, ymax_,
                                  delta_, initialPose);
    gsp_->setllsamplerange(llsamplerange_);
    gsp_->setllsamplestep(llsamplestep_);
    gsp_->setlasamplerange(lasamplerange_);
    gsp_->setlasamplestep(lasamplestep_);
    gsp_->setminimumScore(minimum_score_);

    // Call the sampling function once to set the seed.
    GMapping::sampleGaussian(1,unsigned int(seed_));

    return true;
}

bool GSlam::getOdomPose(GMapping::OrientedPoint& gmap_pose, time_t t)
{
//    // Get the pose of the centered laser at the right time
//    centered_laser_pose_.stamp_ = t;
//    // Get the laser's pose that is centered
//    tf::Stamped<tf::Transform> odom_pose;
//    try
//    {
//        tf_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
//    }
//    catch(tf::TransformException e)
//    {
//        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
//        return false;
//    }
//    double yaw = tf::getYaw(odom_pose.getRotation());
//
//    gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
//                                        odom_pose.getOrigin().y(),
//                                        yaw);
    return true;
}


double GSlam::computePoseEntropy() //ok
{
    double weight_total = 0.0;
    for (std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
            it != gsp_->getParticles().end();it++)
    {
        weight_total += it->weight;
    }

    double entropy = 0.0;
    for (std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
         it != gsp_->getParticles().end();it++)
    {
        if(it->weight/weight_total>0.0)
            entropy += it->weight / weight_total * log(it->weight / weight_total);
    }
    return -entropy;
}

bool
GSlam::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose) //ok
{
    if(!getOdomPose(gmap_pose, scan.header.stamp))
        return false;
    if (scan.ranges.size() != gsp_laser_beam_count_)
        return false;

    double *ranges_double = new double[scan.ranges.size()];
    for (unsigned int i = 0; i < scan.ranges.size(); i++)
    {
        if(scan.ranges[i] < scan.range_min)
            ranges_double[i] = (double)scan.range_max;
        else
            ranges_double[i] = (double)scan.ranges[i];
    }

    GMapping::RangeReading reading(scan.ranges.size(),
                                   ranges_double,
                                   gsp_laser_,
                                   scan.header.stamp);
    delete[] ranges_double;
    reading.setPose(gmap_pose);

    return gsp_->processScan(reading);
}

void GSlam::updateMap(const sensor_msgs::LaserScan &scan)
{
    boost::mutex::scoped_lock map_lock(map_mutex_);
    GMapping::ScanMatcher matcher;

    matcher.setLaserParameters(unsigned int (scan.ranges.size()), &(laser_angles_[0]), gsp_laser_->getPose());
    matcher.setlaserMaxRange(maxRange_);
    matcher.setusableRange(maxUrange_);
    matcher.setgenerateMap(true);

    GMapping::GridSlamProcessor::Particle best = gsp_->getParticles()[gsp_->getBestParticleIndex()];
    std_msgs::Float64 entropy;
//    entropy.data = computePoseEntropy();
    if(!got_map_)
    {
        map_.map.info.resolution = float(delta_);
        map_.map.info.origin.position.x = 0.0;
        map_.map.info.origin.position.y = 0.0;
        map_.map.info.origin.position.z = 0.0;
        map_.map.info.origin.orientation.x = 0.0;
        map_.map.info.origin.orientation.y = 0.0;
        map_.map.info.origin.orientation.z = 0.0;
        map_.map.info.origin.orientation.w = 0.0;
    }
    GMapping::Point center;
    center.x = (xmin_ - xmax_) / 2.0;
    center.y = (ymin_ - ymax_) / 2.0;

    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, delta_);
    for (GMapping::GridSlamProcessor::TNode *n = best.node;n;n=n->parent)
    {
        if(!n->reading)
        {
//            std::cout << "In function updateMap, Reading is NULL" << std::endl;
            continue;
        }
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
        matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
    }

    if(map_.map.info.width != (unsigned int) smap.getMapSizeX() ||
       map_.map.info.height != (unsigned int) smap.getMapSizeY())
    {
        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(),smap.getMapSizeY()));
        xmin_ = wmin.x; ymin_ = wmin.y;
        xmax_ = wmax.x; ymax_ = wmax.y;
//        std::cout <<("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
//                xmin_, ymin_, xmax_, ymax_) <<std::endl;
        map_.map.info.width = (unsigned int)smap.getMapSizeX();
        map_.map.info.height = (unsigned int)smap.getMapSizeY();
        map_.map.info.origin.position.x = xmin_;
        map_.map.info.origin.position.y = ymin_;

//        std::cout<<("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y)<<std::endl;
    }

    for (int x = 0; x < smap.getMapSizeX(); x++)
    {
        for (int y = 0; y <smap.getMapSizeY(); y++)
        {
            GMapping::IntPoint p(x,y);
            double occ = smap.cell(p);
            assert(occ<=1.0);
            if(occ< 0 )
                map_.map.data[MAP_IDX(map_.map.info.width,x,y)] = -1;
            else if(occ > occ_thresh_)
                map_.map.data[MAP_IDX(map_.map.info.width,x,y)] = 100;
            else
                map_.map.data[MAP_IDX(map_.map.info.width,x,y)] = 0;
        }
    }

    got_map_ = true;
    time(&map_.map.header.stamp);

}

















