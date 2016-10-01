#ifndef IPL_ROS_PROBE_FACTORY_H
#define IPL_ROS_PROBE_FACTORY_H

#include "ipipeline_io/IPLRosProbe.h"


/**
 * \class IPLRosProbeFactory: example of probe factory class
 * Mostly useful for the test programs, given that the RosProbe is
 * not able to do anything
 * */

class IPLRosProbeFactory 
{
    protected:
        ros::NodeHandle & nh_;
    public:
        IPLRosProbeFactory(ros::NodeHandle & nh) : nh_(nh) {}
        ~IPLRosProbeFactory() {}

        IPLRosProbe * operator()(unsigned int i) {
            char probename[64];
            sprintf(probename,"Probe%04d",i);
            return new IPLRosProbe(probename,nh_);
        }
};



#endif // IPL_ROS_PROBE_FACTORY_H
