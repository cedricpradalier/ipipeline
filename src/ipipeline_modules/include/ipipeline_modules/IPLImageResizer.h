#ifndef IPL_IMAGE_RESIZER_H
#define IPL_IMAGE_RESIZER_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLImageResizer : \see IPLImageFilter
 * Resize input using cv::Resize function 
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageResizer : public IPLImageFilter
{
	protected :
		double wscale;
		double hscale;

	public :
		/** 
		 * \arg ws: horizontal scaling for image 0 & 1
		 * \arg hs: vertical scaling for image 0 & 1
		 ***/
		IPLImageResizer(const char * n, double ws, double hs);

        // TODO: add constructor with dest size argument
        // TODO: add interpolation method in constructor

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_RESIZER_H
