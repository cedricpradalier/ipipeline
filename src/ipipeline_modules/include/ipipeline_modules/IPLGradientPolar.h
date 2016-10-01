#ifndef IPL_GRADIENT_POLAR_H
#define IPL_GRADIENT_POLAR_H

#include "ipipeline_core/IPLImageProcessor.h"

/**
 * \class IPLGradientPolar : \see IPLImageProcessor
 * Compute the gradient range and magnitude from image src X and Y.
 * input "X" must output a single Float32 image
 * input "Y" must output a single Float32 image
 * output is a pair of Float32 image, the magnitude and the phase of the 
 * gradient, in radians
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLGradientPolar : public IPLImageProcessor
{
	protected :
		unsigned int inputX,inputY;
        bool angleInDegree_;

	public :
		IPLGradientPolar(const char * n, bool degree);
		~IPLGradientPolar();

		virtual bool processInput();

		virtual bool checkInput() const;

};




#endif // IPL_GRADIENT_POLAR_H
