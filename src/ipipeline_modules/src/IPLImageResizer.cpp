#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPLImageResizer.h"

IPLImageResizer::IPLImageResizer(const char * n, double ws, double hs) : 
	IPLImageFilter(n)
{
	wscale = ws;
	hscale = hs;
	assert(ws > 0);
	assert(hs > 0);
}


bool IPLImageResizer::checkInput() const 
{
	return true;
}

bool IPLImageResizer::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        cv::resize(in[i],outputVector[i], cv::Size(), wscale, hscale);
    }

	return true;
}

