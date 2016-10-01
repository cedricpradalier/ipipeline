#ifndef IPL_GRADIENT_PHASE_H
#define IPL_GRADIENT_PHASE_H

#include "ipipeline_core/IPLImageProcessor.h"

/**
 * \class IPLGradientPhase : \see IPLImageProcessor
 * Compute the gradient phase from image src X and Y.
 * input "X" must output a single Float32 image
 * input "Y" must output a single Float32 image
 * output is a single Float32 image
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLGradientPhase : public IPLImageProcessor
{
	protected :
		bool angleInDegree_;
		unsigned int inputX,inputY;

	public :
		IPLGradientPhase(const char * n, bool degree);
		~IPLGradientPhase();

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_GRADIENT_PHASE_H
