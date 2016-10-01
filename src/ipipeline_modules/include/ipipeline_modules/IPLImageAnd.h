#ifndef IPL_IMAGE_AND_H
#define IPL_IMAGE_AND_H

#include "ipipeline_core/IPLBinaryOperator.h"

/**
 * \class IPLImageAnd : \see IPLBinaryOperator
 * Apply a AND operator on its two input. 
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLImageAnd : public IPLBinaryOperator
{
	public :
		IPLImageAnd(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_AND_H
