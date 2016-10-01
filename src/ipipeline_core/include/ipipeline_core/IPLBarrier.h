#ifndef IPL_BARRIER_H
#define IPL_BARRIER_H

#include "IPLImageProcessor.h"

/**
 * \class IPLBarrier 
 * Use to garantee that all input image have been processed before continuing
 * input "Default" must output any number of image. It it transferred as the
 * output of this module
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLBarrier : public IPLImageProcessor
{
	protected : 
		unsigned int deftInput;
		IPLImageProcessor * input;

	public :
		/**
		 * \arg _which: reference to the index of the input image 
		 * to select. Can vary dynamically.
		 * **/
		IPLBarrier(const char * n,IPLImageProcessor * in,...);

		virtual bool processInput();
		virtual ImageProcessorOutput getOutput() const;

		virtual bool checkInput() const;
};



#endif // IPL_BARRIER_H
