
#include "ipipeline_modules/IPLImageDiff.h"

IPLImageDiff::IPLImageDiff(const char * n) : 
	IPLBinaryOperator(n)
{
}

bool IPLImageDiff::checkInput() const 
{
	IPLImageProcessor * ipA = getInput(inputA);
	IPLImageProcessor * ipB = getInput(inputB);
    if (!sameSizeAndType(ipA,ipB)) return false;
	return true;
}


bool IPLImageDiff::processInput()
{
	IPLImageProcessor * ipA = getInput(inputA);
	IPLImageProcessor * ipB = getInput(inputB);
	ImageProcessorOutput iA = ipA->getOutput();
	ImageProcessorOutput iB = ipB->getOutput();

	unsigned int i;
    outputVector.resize(iA.size());
    for (i=0;i<iA.size();i++) {
        cv::subtract(iA[i],iB[i],outputVector[i]);
    }
	return true;
}

