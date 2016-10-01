#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPLImageBlurrer.h"

IPLImageBlurrer::IPLImageBlurrer(const char * n, unsigned int width) : 
	IPLImageFilter(n), ksize_(width,width)
{
}

bool IPLImageBlurrer::checkInput() const 
{
	return true;
}

bool IPLImageBlurrer::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        // TODO? use Gaussian Blur?
        cv::blur(in[i],outputVector[i],ksize_);
    }
	return true;
}

