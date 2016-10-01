#ifndef IPL_IMAGE_Add_H
#define IPL_IMAGE_Add_H

#include "ipipeline_core/IPLBinaryOperator.h"

/**
 * \class IPLImageAdd : \see IPLBinaryOperator
 * Apply a Add operator on its two input. 
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLImageAdd : public IPLBinaryOperator
{
	public :
		IPLImageAdd(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_Add_H
