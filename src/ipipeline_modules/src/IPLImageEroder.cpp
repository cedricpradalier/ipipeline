#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPLImageEroder.h"

IPLImageEroder::IPLImageEroder(const char * n, unsigned int size) : 
	IPLImageFilter(n), kernel_(cv::Mat::ones(size,size,CV_8UC1))
{
}

bool IPLImageEroder::checkInput() const 
{
	return true;
}

bool IPLImageEroder::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        cv::erode(in[i],outputVector[i],kernel_);
    }
	return true;
}

