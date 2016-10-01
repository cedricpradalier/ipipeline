#ifndef IPL_GLCM_MASK_H
#define IPL_GLCM_MASK_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLGlcmMask : \see IPLImageFilter 
 * Select part of input image which contains texture
 * input "Default" must output be a set of Int8u image
 * output is set of Int8u images (0x00 or 0xFF) that can be used as AND mask to
 * remove textureless region.
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLGlcmMask : public IPLImageFilter
{
	protected :
		unsigned int divider;
		int dx,dy,threshold;
        cv::Size winsize;

		int glcm(const cv::Mat1b & img, int u, int v);
		
	public :
		/**
		 * \arg div : number of division of original image
		 * \arg _dx, \arg _dy: direction of the texture to look for
		 * \arg _threshold: min texture measure to accept
		 * **/
		IPLGlcmMask(const char * n, 
				unsigned int div, int _dx, int _dy, int _threshold);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_GLCM_MASK_H
