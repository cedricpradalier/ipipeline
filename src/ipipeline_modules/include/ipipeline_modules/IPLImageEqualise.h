#ifndef IPL_IMAGE_EQUALISE_H
#define IPL_IMAGE_EQUALISE_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLImageEqualise : \see IPLImageFilter
 * Equalise the histogram of an image.
 * input "Default" must output a set of Int8u or RGB8u image 
 * input "Mask" (if present) must output a set of Int8u (binary image) indicating which
 * pixel to consider in the histogram 
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageEqualise : public IPLImageFilter
{
	public :
		IPLImageEqualise(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_EQUALISE_H
