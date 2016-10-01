
#include "ipipeline_core/IPLCachingSource.h"

IPLCachingSource::IPLCachingSource(const char * name, 
				IPLImageProcessorRole role, int type, unsigned int n,
				unsigned int w, unsigned int h) :
	IPLImageSource(name,role,type,n,w,h) 
{
	activate_cache = false;
	update_cache = true;
}

IPLCachingSource::~IPLCachingSource()
{
}

