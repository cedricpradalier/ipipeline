#ifndef IPL_MULTI_IMAGE_SOURCE_H
#define IPL_MULTI_IMAGE_SOURCE_H

#include "IPLImageSource.h"
#include "IPLCvImageSource.h"

/**
 * \class IPLMultiImageSource : \see IPLIppiImageSource
 * Implementation of a stream of image source
 * An image source has no input
 * output is a single of IppiImage
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLMultiImageSource : public IPLCvImageSource
{
	protected:
		unsigned int firstFrame, lastFrame, curFrame;
		std::string ftemplate;
	public :
		virtual ~IPLMultiImageSource();
		/**
		 * Constructor: uses loadSource(fname) to fill internal
		 * buffer
		 * **/
		IPLMultiImageSource(const char * n,
				const char * ftemplate, unsigned int fstart, unsigned int fend);

		virtual bool processInput();
};




#endif // IPL_MULTI_IMAGE_SOURCE_H
