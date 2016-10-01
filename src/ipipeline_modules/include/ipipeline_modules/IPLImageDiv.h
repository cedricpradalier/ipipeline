#ifndef IPL_IMAGE_Div_H
#define IPL_IMAGE_Div_H

#include "ipipeline_core/IPLBinaryOperator.h"

/**
 * \class IPLImageDiv : \see IPLBinaryOperator
 * Apply a Div operator on its two input. 
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLImageDiv : public IPLBinaryOperator
{
	public :
		IPLImageDiv(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_Div_H
