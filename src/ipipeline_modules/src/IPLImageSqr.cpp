
#include "ipipeline_modules/IPLImageSqr.h"

IPLImageSqr::IPLImageSqr(const char * n) : 
	IPLImageFilter(n)
{
}

bool IPLImageSqr::checkInput() const 
{
	return true;
}

bool IPLImageSqr::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        cv::multiply(in[i],in[i],outputVector[i]);
    }
	return true;
}

