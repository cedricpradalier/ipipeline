#include "ipipeline_core/IPLMultiImageSource.h"

IPLMultiImageSource::~IPLMultiImageSource()
{
}


IPLMultiImageSource::IPLMultiImageSource(const char * name, 
		const char * filenames, unsigned int fstart, unsigned int fend) :
	IPLCvImageSource(name)
{
	ftemplate = filenames;
	firstFrame = curFrame = fstart;
	lastFrame = fend;
}



bool IPLMultiImageSource::processInput()
{
	char tmp[ftemplate.size()+16];
	unsigned int nloop = 0;
	unsigned int cFrame = curFrame;
	while (nloop < 1) {
		if (curFrame > lastFrame) {
			printf("IPLMultiImageSource: reset to first frame\n");
			curFrame = firstFrame;
		}
		snprintf(tmp,ftemplate.size()+16,ftemplate.c_str(),curFrame);
		curFrame += 1;
		//printf("Trying to load '%s'\n",tmp);
		if (loadSource(tmp)) {
			//printf("Loaded\n");
			assert(abs(outputVector[0].rows) < 10000);
			assert(abs(outputVector[0].cols) < 10000);
			assert(abs(outputVector[0].step) < 10000);
			return true;
		}
		//printf("Failed\n");
		if (curFrame == cFrame) {
			nloop += 1;
		}
	}
	return error("Could not find a single loadable image");
}


