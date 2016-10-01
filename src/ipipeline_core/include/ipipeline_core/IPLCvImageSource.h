#ifndef IPL_CV_IMAGE_SOURCE_H
#define IPL_CV_IMAGE_SOURCE_H

#include "IPLImageSource.h"

/**
 * \class IPLIppiImageSource : \see IPLImageSource
 * Implementation of a constant image source
 * An image source has no input
 * output is a set of IppiImage
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLCvImageSource : public IPLImageSource
{
	public :
		virtual ~IPLCvImageSource();
		/**
		 * Constructor: uses loadSource(I) to fill internal
		 * buffer
		 * **/
		IPLCvImageSource(const char * n,
				const cv::Mat & I);

		/**
		 * Constructor: uses loadSource(fname) to fill internal
		 * buffer
		 * **/
		IPLCvImageSource(const char * n, const std::string & fname);

		/** Use Imlib2 to load fname into the internal output buffer **/
		bool loadSource(const std::string & fname);

		/** does nothing **/
		virtual bool processInput();
	protected:
		/**
		 * Protected Constructor: used by IPLMultiImageSource
		 * **/
		IPLCvImageSource(const char * n);
};




#endif // IPL_CV_IMAGE_SOURCE_H
