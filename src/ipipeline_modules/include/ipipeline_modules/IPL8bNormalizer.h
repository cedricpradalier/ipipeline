#ifndef IPL_8B_NORMALIZER_H
#define IPL_8B_NORMALIZER_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPL8bNormalizer : \see IPLImageFilter
 * Convert its 1 channel input to Int8u, with full range
 * input "Default" must output any number of 1 channel image.
 * output is as many Int8u image as in input.
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPL8bNormalizer : public IPLImageFilter
{
    protected:
        int maskInputId;
	public :
		IPL8bNormalizer(const char * n, bool maskInput = false);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_8B_NORMALIZER_H
