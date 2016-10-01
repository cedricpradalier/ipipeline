#include <math.h>

#include "ipipeline_modules/IPLGradientPhase.h"

IPLGradientPhase::IPLGradientPhase(const char * n, bool degree) : 
	IPLImageProcessor(n)
{
	inputX = addInput("X");
	inputY = addInput("Y");
    angleInDegree_ = degree;
}

IPLGradientPhase::~IPLGradientPhase() 
{
}


bool IPLGradientPhase::checkInput() const 
{
	IPLImageProcessor * inpx = getInput(inputX);
	IPLImageProcessor * inpy = getInput(inputY);
	if (!sameSizeAndType(inpx,inpy)) {
		return error(INVALID_IMAGE_SIZE);
	}
	return true;
}

bool IPLGradientPhase::processInput()
{
	IPLImageProcessor * inpx = getInput(inputX);
	ImageProcessorOutput inx = inpx->getOutput();
	IPLImageProcessor * inpy = getInput(inputY);
	ImageProcessorOutput iny = inpy->getOutput();

    outputVector.resize(inx.size()); // == iny.size()
    for (unsigned int i=0;i<outputVector.size();i++) {
        cv::phase(inx[i],iny[i],outputVector[i],angleInDegree_);
    }
	return true;
}

