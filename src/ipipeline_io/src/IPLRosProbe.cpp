#include <sys/time.h>
#include <cv_bridge/cv_bridge.h>

#include "ipipeline_io/ProbeStatus.h"
#include "ipipeline_io/IPLRosProbe.h"

IPLRosProbe::IPLRosProbe(const char * n, ros::NodeHandle & nh) :
	IPLGenericProbe(n), it_(nh) 
{
    std::string basename("/");
    basename += n;
    sub_ = nh.subscribe(basename + "/command",1,&IPLRosProbe::commandCallback,this);
    pub_ = it_.advertise(basename + "/image",1);
    status_pub_ = nh.advertise<ipipeline_io::ProbeStatus>(basename + "/status",1);
    firstpub_ = true;
}

void IPLRosProbe::commandCallback(const ipipeline_io::ProbeCommandConstPtr & msg)
{
    boost::mutex::scoped_lock lock(mutex_);
    // ROS_INFO("Received command callback %d,%d",msg->action,msg->index);
    command_ = *msg;
}

void IPLRosProbe::updateProbe()
{
    boost::mutex::scoped_lock lock(mutex_);
    if (!command_.action) return;
    // ROS_INFO("Update probe %d,%d",command_.action,command_.index);
    this->processCommand((IPLProbeAction)command_.action,command_.index);
    ROS_INFO("Probe %s: source is %s index %d",this->getName(),(*opit)->getName(),inputIndex);
    command_.action = 0;
    firstpub_ = true;
}
		
		
void IPLRosProbe::probeFunction(const cv::Mat & in) 
{
    boost::mutex::scoped_lock lock(mutex_);
    if (pub_.getNumSubscribers() && (in.total()>0)) {
        // if (firstpub_) ROS_INFO("Trying the conversion");
        sensor_msgs::ImagePtr msg;
        if (in.channels()==3) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", in).toImageMsg();
        } else if (in.channels()==1) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", in).toImageMsg();
        } else {
            ROS_WARN("Cannot convert source image to a publishable format");
            return;
        }
        pub_.publish(msg);
        // if (firstpub_) ROS_INFO("Conversion worked");
        ipipeline_io::ProbeStatus status;
        if (opstore && (opit != opstore->end())) {
            status.input = (*opit)->getName();
            status.index = inputIndex;
        }
        status_pub_.publish(status);
        firstpub_ = false;
    }
}

		
