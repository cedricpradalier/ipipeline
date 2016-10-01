#include "ipipeline_io/IPLMultiArrayRosPublisher.h"


IPLMultiArrayRosPublisher::IPLMultiArrayRosPublisher(const char * n, 
       ros::NodeHandle & nh,  const std::string & topic) 
    : IPLImageProcessor(n)
{
	deftInput = addInput("Default");
    pub = nh.advertise<std_msgs::Float32MultiArray>(topic,1);

}


bool IPLMultiArrayRosPublisher::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
    if (in.size() > 0) {
        unsigned int w = in[0].size().width;
        unsigned int h = in[0].size().height;
        array.layout.dim.resize(2);
        array.layout.dim[0].label="row";
        array.layout.dim[0].size=h*w;
        array.layout.dim[0].stride=w;
        array.layout.dim[1].label="col";
        array.layout.dim[1].size=w;
        array.layout.dim[1].stride=1;
        array.data.resize(h*w);
        for (unsigned int j=0;j<h;j++) {
            for (unsigned int i=0;i<w;i++) {
                array.data[i+j*w] = in[0].at<float>(j,i);
            }
        }
        pub.publish(array);
    }
    return true;
}

bool IPLMultiArrayRosPublisher::checkInput() const
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
	if (in.size() > 1) {
		ROS_WARN("image publisher %s receives more than one image",name);
	}
    for (unsigned int i=0;i<in.size();i++) {
        if (in[i].type() != CV_32FC1) {
            return error(INVALID_TYPE);
        }
    }
	return true;
}

