#ifndef IPL_LOCAL_MAX_H
#define IPL_LOCAL_MAX_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLLocalMax : \see IPLImageFilter
 * Find local maximum, using a input image and a max of thresholded pixels
 * input "Image" is a set of 1-Channel images.
 * input "Mask" is a set of 1-Channel boolean image
 * output is as many float matrices than original image, each of them with n
 * lines and 2 columns, where n is the number of local maxima found 
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLLocalMax : public IPLImageProcessor
{
	protected :
        int imageInput;
        int maskInput;
	public :
		/**
         * See OpenCV threshold function
		 * **/
		IPLLocalMax(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;

};




#endif // IPL_LOCAL_MAX_H
