
#include "ipipeline_modules/IPLImageSqrt.h"

IPLImageSqrt::IPLImageSqrt(const char * n) : 
	IPLImageFilter(n)
{
}

bool IPLImageSqrt::checkInput() const 
{
	return true;
}

bool IPLImageSqrt::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        cv::sqrt(in[i],outputVector[i]);
    }

	return true;
}

