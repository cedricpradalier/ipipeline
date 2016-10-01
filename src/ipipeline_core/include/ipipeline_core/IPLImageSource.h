#ifndef IPL_IMAGE_SOURCE_H
#define IPL_IMAGE_SOURCE_H

#include "IPLImageProcessor.h"
/**
 * \class IPLImageSource : abstract class
 * Implementation of a generic image source. 
 * An image source has no input
 * output is a set of ImageProcessorOutput
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageSource : public IPLImageProcessor
{
	protected :
		IPLImageSource(const char * name, IPLImageProcessorRole role);

	public :
		/**
		 * \arg t : type of the output
		 * \arg n : number of output images, 
		 * \arg w,h : image size
		 * **/
		IPLImageSource(const char * name, IPLImageProcessorRole role, 
				int type, unsigned int n=1,
				unsigned int w=0, unsigned int h=0);
		virtual ~IPLImageSource();

		/** dummy function since there is no input **/
		virtual bool checkInput() const;

		/** Redefined source characteristic, and reallocate out **/
		void setSourceInfo(int type, unsigned int n,
				unsigned int w, unsigned int h);

};




#endif // IPL_IMAGE_SOURCE_H
