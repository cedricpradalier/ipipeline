#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPLImageGradient.h"

IPLImageGradient::IPLImageGradient(const char * n, int dx, int dy) : 
	IPLImageFilter(n), dx_(dx), dy_(dy)
{
}

bool IPLImageGradient::checkInput() const 
{
	return true;
}

bool IPLImageGradient::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        cv::Sobel(in[i],outputVector[i],-1,dx_,dy_);
    }
	return true;
}

