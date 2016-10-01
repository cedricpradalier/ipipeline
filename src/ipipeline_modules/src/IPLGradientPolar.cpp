#include <math.h>

#include "ipipeline_modules/IPLGradientPolar.h"

IPLGradientPolar::IPLGradientPolar(const char * n, bool degree) : 
	IPLImageProcessor(n)
{
	inputX = addInput("X");
	inputY = addInput("Y");
    angleInDegree_ = degree;
    outputVector.resize(2);
}

IPLGradientPolar::~IPLGradientPolar() 
{
}


bool IPLGradientPolar::checkInput() const 
{
	IPLImageProcessor * inpx = getInput(inputX);
	IPLImageProcessor * inpy = getInput(inputY);
	if (!sameSizeAndType(inpx,inpy)) {
		return error(INVALID_IMAGE_SIZE);
	}
	ImageProcessorOutput inx = inpx->getOutput();
    if (inx.size() != 1) {
        return error("GradientPolar can only deal with single image");
    }
	return true;
}

bool IPLGradientPolar::processInput()
{
	IPLImageProcessor * inpx = getInput(inputX);
	ImageProcessorOutput inx = inpx->getOutput();
	IPLImageProcessor * inpy = getInput(inputY);
	ImageProcessorOutput iny = inpy->getOutput();

    cv::cartToPolar(inx[0],iny[0],outputVector[0],outputVector[1],angleInDegree_);
	return true;
}

