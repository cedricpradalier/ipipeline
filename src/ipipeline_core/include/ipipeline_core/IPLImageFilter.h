#ifndef IPL_IMAGE_FILTER_H
#define IPL_IMAGE_FILTER_H

#include "IPLImageProcessor.h"

/**
 * \class IPLImageFilter : abstract class
 * Used to apply am ippi operator on input image.
 * input "Default" must output a set of images
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageFilter : public IPLImageProcessor
{
	protected :
		unsigned int deftInput;

	public :
		IPLImageFilter(const char * n);

		~IPLImageFilter();

};




#endif // IPL_IMAGE_FILTER_H
