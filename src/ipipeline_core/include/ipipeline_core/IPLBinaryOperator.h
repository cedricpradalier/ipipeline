#ifndef IPL_BINARY_OPERATOR_H
#define IPL_BINARY_OPERATOR_H

#include "IPLImageProcessor.h"

/**
 * \class IPLBinaryOperator : abstract class
 * Apply a binary operator on its two input. 
 * input "InputA" must be a set of n images
 * input "InputB" must be a set of n images
 * output is a set of n images, possibly of the same type.
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLBinaryOperator : public IPLImageProcessor
{
	protected :
		unsigned int inputA,inputB;
	public :
		IPLBinaryOperator(const char * n);

		~IPLBinaryOperator();

};




#endif // IPL_BINARY_OPERATOR_H
