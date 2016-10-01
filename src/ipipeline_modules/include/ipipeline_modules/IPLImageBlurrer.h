#ifndef IPL_IMAGE_BLURRER_H
#define IPL_IMAGE_BLURRER_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLImageBlurrer : \see IPLImageFilter
 * Blur input using ippiFilterGauss and a 3x3 mask
 * input "Default" must output a set of Int8u or RGB8u image 
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageBlurrer : public IPLImageFilter
{
    protected:
        cv::Size ksize_;
	public :
		IPLImageBlurrer(const char * n, unsigned int width=3);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_BLURRER_H
