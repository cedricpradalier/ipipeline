#ifndef IPL_MEDIAN_FILTER_H
#define IPL_MEDIAN_FILTER_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLMedianFilter : \see IPLImageFilter
 * Apply a median filter operator on input using ippiMedianFilter function 
 * input "Default" must output a set of Int8u images
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLMedianFilter : public IPLImageFilter
{
    protected:
        int ksize_;
	public :
		IPLMedianFilter(const char * n, unsigned int width);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_MEDIAN_FILTER_H
