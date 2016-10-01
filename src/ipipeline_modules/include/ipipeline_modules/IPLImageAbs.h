#ifndef IPL_IMAGE_ABS_H
#define IPL_IMAGE_ABS_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLImageAbs : \see IPLImageFilter
 * Take the absolute value of input image
 * input "Default" must output a set of Int16s or Float32 image 
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageAbs : public IPLImageFilter
{
	public :
		IPLImageAbs(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_ABS_H
