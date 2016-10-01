#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPLTranslateCenter.h"

IPLTranslateCenter::IPLTranslateCenter(const char * n) : 
	IPLImageFilter(n)
{
	centerPosId = addInput("Center Pos");
}



bool IPLTranslateCenter::checkInput() const 
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	IPLImageProcessor * centerIP = getInput(centerPosId);
    ImageProcessorOutput center = centerIP->getOutput();
    if (center.size() != in.size()) {
        return error("Inconsistent number of images");
    }
    for (unsigned int i=0;i<center.size();i++) {
        if (center[i].type() != CV_32FC1) {
            return error("Center must be of type CV_32FC1");
        }
        if (center[i].rows * center[i].cols != 2) {
            return error("Center must contain 2-element vectors");
        }
    }
	return true;
}

bool IPLTranslateCenter::processInput()
{
	IPLImageProcessor * centerIP = getInput(centerPosId);
    ImageProcessorOutput center = centerIP->getOutput();
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        cv::Mat C = center[i].reshape(1,2);
        cv::Mat_<float> transl(2,3,0.0);
        transl(0,0) = 1.0; transl(0,2) = C.at<float>(0,0);
        transl(1,1) = 1.0; transl(1,2) = C.at<float>(1,0);
        cv::warpAffine(in[i],outputVector[i],transl,in[i].size());
    }
	return true;
}


