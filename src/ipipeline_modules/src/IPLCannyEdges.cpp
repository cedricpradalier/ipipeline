#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPLCannyEdges.h"

IPLCannyEdges::IPLCannyEdges(const char * n, double thresLow, double thresHigh) : 
	IPLImageFilter(n)
{
	lowThres = thresLow;
	highThres = thresHigh;
}


IPLCannyEdges::~IPLCannyEdges()
{
}


bool IPLCannyEdges::checkInput() const 
{
    IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
    for (unsigned int i=0;i<in.size();i++) {
        switch (in[i].type()) {
            case CV_8UC1:
                break;
            default : 
                return error(INVALID_TYPE);
        }
    }
    return true;
}

bool IPLCannyEdges::processInput()
{
    IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
    outputVector.resize(in.size());
    for (unsigned int i=0;i<in.size();i++) {
        cv::Canny(in[i],outputVector[i],lowThres,highThres);
    }

	return true;
}

