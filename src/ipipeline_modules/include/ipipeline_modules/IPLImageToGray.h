#ifndef IPL_IMAGE_TOGRAY_H
#define IPL_IMAGE_TOGRAY_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLImageToGray : \see IPLImageFilter
 * Convert input to gray level only
 * input "Default" must output a set of RGB8u image 
 * output is as many image as input, with type Int8u
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageToGray : public IPLImageFilter
{
	public :
		IPLImageToGray(const char * n);
		~IPLImageToGray();

		virtual bool processInput();

		virtual bool checkInput() const;


};




#endif // IPL_IMAGE_TOGRAY_H
