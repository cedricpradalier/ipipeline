#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPLImageRotater.h"

IPLImageRotater::IPLImageRotater(const char * n,  int angle_deg) : 
	IPLImageFilter(n), angle_deg_(angle_deg)
{
}

bool IPLImageRotater::checkInput() const 
{
	return true;
}

bool IPLImageRotater::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        cv::Point2f center(in[i].cols/2., in[i].rows/2.);
        cv::Mat rot = cv::getRotationMatrix2D(center,angle_deg_,1.0);
        cv::warpAffine(in[i],outputVector[i],rot,in[i].size());
    }
	return true;
}


