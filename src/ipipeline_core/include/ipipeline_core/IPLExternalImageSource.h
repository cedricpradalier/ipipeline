#ifndef IPL_EXTERNAL_IMAGE_SOURCE_H
#define IPL_EXTERNAL_IMAGE_SOURCE_H

#include "IPLImageProcessor.h"

/**
 * \class IPLExternalImageSource 
 * Implementation of a image source proxy. 
 * An image source has no input
 * output is a set of ImageProcessorOutput
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLExternalImageSource : public IPLImageProcessor
{
	protected :

	public :
		/** I must not be NULL */
		IPLExternalImageSource(const char * name, 
				const cv::Mat * I=NULL, unsigned int n=1);
		virtual ~IPLExternalImageSource();

		void setImage(const cv::Mat * I, unsigned int n=1);

		/** dummy function since there is no input **/
		virtual bool checkInput() const;

		/** does nothing **/
		virtual bool processInput();
};




#endif // IPL_EXTERNAL_IMAGE_SOURCE_H
