#ifndef IPL_IMAGE_SQR_H
#define IPL_IMAGE_SQR_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLImageSqr : \see IPLImageFilter
 * Square input 
 * input "Default" must output a set of 1 channel images
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageSqr : public IPLImageFilter
{
	public :
		IPLImageSqr(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_SQR_H
