#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPLThresholder.h"

IPLThresholder::IPLThresholder(const char * n,
        double thr, double max_val, int thr_type) :
	IPLImageFilter(n), 
    threshold(thr), max_value(max_val), threshold_type(thr_type)
{
}

void IPLThresholder::readConfig(Config *config)
{
	config->getDouble("threshold",&threshold);
	config->getDouble("max_value",&max_value);
	config->getInt("threshold_type",&threshold_type);
}


bool IPLThresholder::checkInput() const 
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
    
    for (unsigned int i=0;i<in.size();i++) {
        if (in[i].channels() != 1) {
            return error(INVALID_TYPE);
        }
    }
	return true;
}

bool IPLThresholder::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	unsigned int i;
	ImageProcessorOutput in = input->getOutput();
    outputVector.resize(in.size());
	for (i=0;i<in.size();i++) {
        cv::threshold(in[i],outputVector[i],threshold,max_value,threshold_type);
	}
	return true;
}

