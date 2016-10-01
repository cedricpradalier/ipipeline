
#include "ipipeline_modules/IPLImageAnd.h"

IPLImageAnd::IPLImageAnd(const char * n) : 
	IPLBinaryOperator(n)
{
}

bool IPLImageAnd::checkInput() const 
{
	IPLImageProcessor * ipA = getInput(inputA);
	IPLImageProcessor * ipB = getInput(inputB);
    if (!sameSizeAndType(ipA,ipB)) return false;
	ImageProcessorOutput iA = ipA->getOutput();
    for (unsigned int i=0;i<iA.size();i++) {
        // Only for binary types
        switch (iA[i].type() & CV_MAT_DEPTH_MASK) {
            case CV_8U:
            case CV_8S:
            case CV_16U:
            case CV_16S:
            case CV_32S:
                break;
            default:
                return error(INVALID_TYPE);
        }
    }

	return true;
}


bool IPLImageAnd::processInput()
{
	IPLImageProcessor * ipA = getInput(inputA);
	IPLImageProcessor * ipB = getInput(inputB);
	ImageProcessorOutput iA = ipA->getOutput();
	ImageProcessorOutput iB = ipB->getOutput();

	unsigned int i;
    outputVector.resize(iA.size());
    for (i=0;i<iA.size();i++) {
        cv::bitwise_and(iA[i],iB[i],outputVector[i]);
    }
	return true;
}

