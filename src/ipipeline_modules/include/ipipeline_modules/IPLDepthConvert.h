#ifndef IPL_DEPTH_CONVERT_H
#define IPL_DEPTH_CONVERT_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLDepthConvert 
 * Used to change the color depth of an input image
 * input "Default" must output a set of images
 * output is as many image as input, with the specified type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLDepthConvert : public IPLImageFilter
{
	protected:
		int type_;
	public :
		IPLDepthConvert(const char * n, int type);

		virtual bool processInput();

		virtual bool checkInput() const;
};


#endif // IPL_DEPTH_CONVERT_H
