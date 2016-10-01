#ifndef IPL_IMAGE_Diff_H
#define IPL_IMAGE_Diff_H

#include "ipipeline_core/IPLBinaryOperator.h"

/**
 * \class IPLImageDiff : \see IPLBinaryOperator
 * Apply a Diff operator on its two input. 
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLImageDiff : public IPLBinaryOperator
{
	public :
		IPLImageDiff(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_Diff_H
