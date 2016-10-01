#include <opencv2/highgui/highgui.hpp>

#include "ipipeline_core/IPLImageSaver.h"

IPLImageSaver::IPLImageSaver( const char * n, 
		const std::string & outputname, bool ispattern, bool autoinc) :
	IPLImageProcessor(n)
{
	frameno = 0;
	autoincr = autoinc;
	output = outputname;
	pattern_output = ispattern;
    im_name = output;
	deftInput = addInput("Default");
	first = true;
}

IPLImageSaver::~IPLImageSaver()
{
}

bool IPLImageSaver::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
	
	if (pattern_output) {
		if (autoincr) {
			_frameid = frameno++;
		}
        char tmp[output.size() + 32];
		sprintf(tmp,output.c_str(),frameid);
        im_name = tmp;
	}

    cv::imwrite(im_name,in[0]);

	if (first) {
		if (pattern_output)
			ROS_INFO("Saver %s: saved image in %s (%s %d)",
					name,im_name.c_str(),output.c_str(),frameid);
		else
			ROS_INFO("Saver %s: saved image in %s",name,im_name.c_str());
	}
	first = false;

	return true;
}

bool IPLImageSaver::checkInput() const
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
	if (in.size() > 1) {
		ROS_WARN("image saver %s receives more than one image\n"
				"\tSaving only first to %s\n",name,output.c_str());
	}
	return true;
}



