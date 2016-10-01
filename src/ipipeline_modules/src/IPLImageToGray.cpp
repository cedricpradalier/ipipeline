#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPLImageToGray.h"

IPLImageToGray::IPLImageToGray(const char * n) : 
	IPLImageFilter(n)
{
}

IPLImageToGray::~IPLImageToGray()
{
}


bool IPLImageToGray::checkInput() const 
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    for (i=0;i<in.size();i++) {
       switch (in[i].channels()) {
           case 1:
           case 3:
               break;
           default:
            return error(INVALID_TYPE);

        }
    }
	return true;
}

bool IPLImageToGray::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        if (in[i].channels() == 1) {
            outputVector[i] = in[i];
        } else {
            cv::cvtColor(in[i],outputVector[i],CV_RGB2GRAY,1);
        }
    }
	return true;
}

