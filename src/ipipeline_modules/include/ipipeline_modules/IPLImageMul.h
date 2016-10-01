#ifndef IPL_IMAGE_Mul_H
#define IPL_IMAGE_Mul_H

#include "ipipeline_core/IPLBinaryOperator.h"

/**
 * \class IPLImageMul : \see IPLBinaryOperator
 * Apply a Mul operator on its two input. 
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLImageMul : public IPLBinaryOperator
{
	public :
		IPLImageMul(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_Mul_H
