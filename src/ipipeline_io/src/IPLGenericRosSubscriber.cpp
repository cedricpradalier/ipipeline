
#include "ipipeline_io/IPLGenericRosSubscriber.h"
#include <cv_bridge/cv_bridge.h>

void IPLGenericRosSubscriber::callback(const sensor_msgs::ImageConstPtr& msg) {
    boost::mutex::scoped_lock lock(mutex_);
    imgmat_ = cv_bridge::toCvShare(msg, "rgb8")->image;
    setFrameId(msg->header.seq);
    ready = true;
    // Signal data readyness
    signalTrigger();
}

IPLGenericRosSubscriber::IPLGenericRosSubscriber(const char * n) :
	IPLImageSource(n, (IPLImageProcessorRole) (IPL_ROLE_SOURCE | IPL_ROLE_TRIGGER)), ready(false)
{
}

IPLGenericRosSubscriber::~IPLGenericRosSubscriber()
{
}

bool IPLGenericRosSubscriber::processInput()
{
    boost::mutex::scoped_lock lock(mutex_);
    if (!ready) {
        return true;
    }
    outputVector.resize(1);
    outputVector[0] = imgmat_;
    return true;
}

