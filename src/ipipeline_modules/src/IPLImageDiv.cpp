
#include "ipipeline_modules/IPLImageDiv.h"

IPLImageDiv::IPLImageDiv(const char * n) : 
	IPLBinaryOperator(n)
{
}

bool IPLImageDiv::checkInput() const 
{
	IPLImageProcessor * ipA = getInput(inputA);
	IPLImageProcessor * ipB = getInput(inputB);
    if (!sameSizeAndType(ipA,ipB)) return false;
	return true;
}


bool IPLImageDiv::processInput()
{
	IPLImageProcessor * ipA = getInput(inputA);
	IPLImageProcessor * ipB = getInput(inputB);
	ImageProcessorOutput iA = ipA->getOutput();
	ImageProcessorOutput iB = ipB->getOutput();

	unsigned int i;
    outputVector.resize(iA.size());
    for (i=0;i<iA.size();i++) {
        cv::divide(iA[i],iB[i],outputVector[i]);
    }
	return true;
}

