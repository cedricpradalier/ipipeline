#ifndef IPL_IMAGE_GRADIENT_H
#define IPL_IMAGE_GRADIENT_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLImageGradient : \see IPLImageFilter
 * Blur input using ippiFilterGauss and a 3x3 mask
 * input "Default" must output a set of Int8u or RGB8u image 
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageGradient : public IPLImageFilter
{
	protected:
        int dx_, dy_;
	public :
		IPLImageGradient(const char * n, int dx, int dy);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_GRADIENT_H
