
#include "ipipeline_core/IPLImageSource.h"

//#define TRACE

IPLImageSource::IPLImageSource(const char * name, IPLImageProcessorRole role) : 
    IPLImageProcessor(name,role) 
{
}

IPLImageSource::IPLImageSource(const char * name, 
        IPLImageProcessorRole role, int type, unsigned int n,
		unsigned int w, unsigned int h) :
	IPLImageProcessor(name,role)
{
    reallocate(type,w,h,n);
}

IPLImageSource::~IPLImageSource() 
{
}


void IPLImageSource::setSourceInfo(int type, unsigned int n,
		unsigned int w, unsigned int h)
{
    reallocate(type,w,h,n);
}

bool IPLImageSource::checkInput() const
{
	return true;
}


