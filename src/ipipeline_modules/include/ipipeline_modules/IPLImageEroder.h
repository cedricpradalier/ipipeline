#ifndef IPL_IMAGE_ERODER_H
#define IPL_IMAGE_ERODER_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLImageEroder : \see IPLImageFilter
 * Apply a dilatation operator on input using ippiErode function 
 * input "Default" must output a set of Int8u images
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageEroder : public IPLImageFilter
{
    protected:
        cv::Mat kernel_;
	public :
		IPLImageEroder(const char * n, unsigned int size);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_ERODER_H
