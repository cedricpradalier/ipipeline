#ifndef IPL_IMAGE_DILATER_H
#define IPL_IMAGE_DILATER_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLImageDilater : \see IPLImageFilter
 * Apply a dilatation operator on input using ippiDilate function 
 * input "Default" must output a set of Int8u images
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageDilater : public IPLImageFilter
{
    protected:
        cv::Mat kernel_;
	public :
        // TODO: add a constructor with specific kernel
		IPLImageDilater(const char * n, unsigned int size);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_DILATER_H
