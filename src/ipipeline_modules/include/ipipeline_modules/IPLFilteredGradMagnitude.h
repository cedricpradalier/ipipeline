#ifndef IPL_FILTERED_GRADIENT_MAGNITUDE_H
#define IPL_FILTERED_GRADIENT_MAGNITUDE_H

#include "ipipeline_core/IPLImageProcessor.h"

/**
 * \class IPLFilteredGradMagnitude : \see IPLImageProcessor
 * Compute the gradient phase from image src X and Y.
 * input "X" must output a single Float32 image
 * input "Y" must output a single Float32 image
 * input "Mask" must output a single Int8u image
 * output is a single Float32 image
 * \see IPLImageProcessor for virtual function 
 * TODO: Find a way to implement this function using IPP primitives.
 * **/
class IPLFilteredGradMagnitude : public IPLImageProcessor
{
	protected :
		IppiImage * out;
		unsigned int inputX,inputY,inputM;

		void freeOutput();
		/**
		 * Reallocate out with type t, and nimg w x h images 
		 * **/
		void reallocate(unsigned int w, unsigned int h);

	public :
		IPLFilteredGradMagnitude(const char * n);
		~IPLFilteredGradMagnitude();

		virtual bool processInput();

		virtual bool checkInput() const;

		virtual const ImageProcessorOutput * getOutput()const;
};




#endif // IPL_FILTERED_GRADIENT_MAGNITUDE_H
