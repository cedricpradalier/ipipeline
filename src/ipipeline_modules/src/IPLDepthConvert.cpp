
#include "ipipeline_modules/IPLDepthConvert.h"

IPLDepthConvert::IPLDepthConvert(const char * n, int type) :
	IPLImageFilter(n), type_(type)
{
}

bool IPLDepthConvert::checkInput() const 
{
	return true;
}

bool IPLDepthConvert::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<outputVector.size();i++) {
        in[i].convertTo(outputVector[i],type_,1,0);
    }
	return true;
}

