
#include "ipipeline_core/IPLExternalImageSource.h"

IPLExternalImageSource::~IPLExternalImageSource()
{
}

IPLExternalImageSource::IPLExternalImageSource(const char * name, 
		const cv::Mat * I, unsigned int n) :
	IPLImageProcessor(name)
{
	setImage(I,n);
}

void IPLExternalImageSource::setImage(const cv::Mat* I, unsigned int n)
{
    outputVector.resize(n);
	if (I != NULL) {
        for (unsigned int i=0;i<n;i++) {
            outputVector[i] = *I;
        }
	} 
}


bool IPLExternalImageSource::processInput()
{
	// this one is always ready, since it only output its image from
	// memory
	return true;
}


bool IPLExternalImageSource::checkInput() const
{
	return true;
}

