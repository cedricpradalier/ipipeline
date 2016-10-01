
#include "ipipeline_modules/IPLImageROI.h"

IPLImageROI::IPLImageROI(const char * n, unsigned int x, unsigned int y, unsigned int w, unsigned int h) : 
	IPLImageFilter(n)
{
	setROI(x,y,w,h);
}

IPLImageROI::IPLImageROI(const char * n, const cv::Rect & rect) : 
	IPLImageFilter(n)
{
	setROI(rect);
}

void IPLImageROI::setROI(unsigned int x, unsigned int y, unsigned int w, unsigned int h)
{
    ROI = cv::Rect(x,y,w,h);
}

void IPLImageROI::setROI(const cv::Rect & rect) 
{
    ROI = rect;
}

bool IPLImageROI::checkInput() const 
{
	return true;
}

bool IPLImageROI::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        outputVector[i] = cv::Mat(in[i],ROI); 
    }

	return true;
}

