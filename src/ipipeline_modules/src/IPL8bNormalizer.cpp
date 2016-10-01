#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPL8bNormalizer.h"

IPL8bNormalizer::IPL8bNormalizer(const char * n, bool maskInput) : 
	IPLImageFilter(n)
{
    if (maskInput) {
        maskInputId = addInput("Mask");
    } else {
        maskInputId = -1;
    }
}

bool IPL8bNormalizer::checkInput() const 
{
	IPLImageProcessor * inputIP = getInput(deftInput);
	ImageProcessorOutput in = inputIP->getOutput();
    for (unsigned int i=0;i<in.size();i++) {
        // Only work for single channel
        if ((in[i].type() >> CV_CN_SHIFT) != 1) {
                return error(INVALID_TYPE);
        }
    }
    if (maskInputId >= 0) {
        IPLImageProcessor * maskIP = getInput(maskInputId);
        ImageProcessorOutput mask = maskIP->getOutput();
        for (unsigned int i=0;i<mask.size();i++) {
            // Only work for single channel
            if (mask[i].type() != CV_8UC1) {
                return error(INVALID_TYPE);
            }
        }
        if (!sameSize(inputIP,maskIP)) {
            return error(INVALID_IMAGE_SIZE);
        }
    }
	return true;
}

bool IPL8bNormalizer::processInput()
{
	IPLImageProcessor * inputIP = getInput(deftInput);
	ImageProcessorOutput in = inputIP->getOutput();
    outputVector.resize(in.size());
    if (maskInputId >= 0) {
        IPLImageProcessor * maskIP = getInput(maskInputId);
        ImageProcessorOutput mask = maskIP->getOutput();
        for (unsigned int i=0;i<in.size();i++) {
            cv::normalize(in[i],outputVector[i],0,0xFF,CV_MINMAX,CV_8UC1,mask[i]);
        }
    } else {
        cv::Mat mask;
        for (unsigned int i=0;i<in.size();i++) {
            cv::normalize(in[i],outputVector[i],0,0xFF,CV_MINMAX,CV_8UC1,mask);
        }
    }
	return true;
}

