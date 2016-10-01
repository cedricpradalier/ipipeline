
#include "ipipeline_modules/IPLImageAbs.h"

IPLImageAbs::IPLImageAbs(const char * n) : 
	IPLImageFilter(n)
{
}

bool IPLImageAbs::checkInput() const 
{
	return true;
}

bool IPLImageAbs::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        cv::absdiff(in[i],cv::Scalar(0.),outputVector[i]);
    }

	return true;
}

