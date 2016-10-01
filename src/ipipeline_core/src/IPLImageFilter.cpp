
#include "ipipeline_core/IPLImageFilter.h"

IPLImageFilter::IPLImageFilter(const char * n) :
	IPLImageProcessor(n)
{
	deftInput = addInput("Default");
}

IPLImageFilter::~IPLImageFilter() 
{
}


