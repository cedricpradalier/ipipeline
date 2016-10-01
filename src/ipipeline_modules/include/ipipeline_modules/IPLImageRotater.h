#ifndef IPL_IMAGE_ROTATER_H
#define IPL_IMAGE_ROTATER_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLImageRotater : \see IPLImageFilter
 * input "Default" must output a set of images
 * output is as many image as input, with same type and same size
 * TODO: add code for proper output size management
 * TODO: add code to use flip and transpose instead of warpAffine when possible
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageRotater : public IPLImageFilter
{
	protected :
		int angle_deg_; 

	public :
		IPLImageRotater(const char * n, int angle_deg);

		virtual bool processInput();

		virtual bool checkInput() const;

};




#endif // IPL_IMAGE_ROTATER_H
