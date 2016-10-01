#ifndef IPL_IMAGE_SQRT_H
#define IPL_IMAGE_SQRT_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLImageSqrt : \see IPLImageFilter
 * Square root of input 
 * input "Default" must output a set of 1 channel images
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageSqrt : public IPLImageFilter
{
	public :
		IPLImageSqrt(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_SQRT_H
