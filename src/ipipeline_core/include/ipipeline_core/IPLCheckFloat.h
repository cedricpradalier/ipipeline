#ifndef IPL_CHECK_FLOAT_H
#define IPL_CHECK_FLOAT_H

#include "IPLImageProcessor.h"

/**
 * \class IPLCheckFloat : \see IPLImageProcessor
 * Assert that all of the pixels of a Float 32 image are
 * valid number
 * Input "Default" is a set of Float32 images
 * There is no output
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLCheckFloat : public IPLImageProcessor
{
	protected:
		unsigned int deftInput;
	public :
		IPLCheckFloat(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_CHECK_FLOAT_H
