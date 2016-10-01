#ifndef IPL_IMAGE_MIN_H
#define IPL_IMAGE_MIN_H

#include "ipipeline_core/IPLBinaryOperator.h"

/**
 * \class IPLImageMin : \see IPLImageFilter
 * Take the Minimum value of 2 input image
 * output is as many image as input, with same type, and size 
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageMin : public IPLBinaryOperator
{
	public :
		IPLImageMin(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_MIN_H
