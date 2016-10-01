#include "ipipeline_io/IPLGenericRosPublisher.h"
#include <cv_bridge/cv_bridge.h>


IPLGenericRosPublisher::IPLGenericRosPublisher(const char * n) 
    : IPLImageProcessor(n)
{
	deftInput = addInput("Default");
}


bool IPLGenericRosPublisher::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
    if (in.size() > 0) {
        if (in[0].channels()==3) {
            image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", in[0]).toImageMsg();
        } else if (in[0].channels()==1) {
            image_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", in[0]).toImageMsg();
        } else {
            ROS_WARN("IPLGenericRosPublisher: Cannot convert source image to a publishable format");
            return false;
        }
        this->publish();
    }
    return true;
}

bool IPLGenericRosPublisher::checkInput() const
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
	if (in.size() > 1) {
		ROS_WARN("image publisher %s receives more than one image",name);
	}
	return true;
}

