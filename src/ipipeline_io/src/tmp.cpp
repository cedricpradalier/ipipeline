#include "ipipeline_io/IPLGenericRosPublisher.h"


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
        IplImage ipl = (IplImage)(in[0]);
        image_ = img_bridge_.cvToImgMsg(&ipl,"rgb8");
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

