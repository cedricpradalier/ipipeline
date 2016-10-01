#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPLMedianFilter.h"

IPLMedianFilter::IPLMedianFilter(const char * n, unsigned int width) : 
	IPLImageFilter(n), ksize_(width)
{
}

bool IPLMedianFilter::checkInput() const 
{
	return true;
}

bool IPLMedianFilter::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        cv::medianBlur(in[i],outputVector[i],ksize_);
    }
	return true;
}

