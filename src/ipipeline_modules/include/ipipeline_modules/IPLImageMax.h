#ifndef IPL_IMAGE_MAX_H
#define IPL_IMAGE_MAX_H

#include "ipipeline_core/IPLBinaryOperator.h"

/**
 * \class IPLImageMax : \see IPLImageFilter
 * Take the Maximum value of 2 input image
 * output is as many image as input, with same type and size
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageMax : public IPLBinaryOperator
{
	public :
		IPLImageMax(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_MAX_H
