#ifndef IPL_Vdisparity_H
#define IPL_Vdisparity_H

#include "ipipeline_core/IPLImageFilter.h"
/**
 * \class IPLVdisparity : \see IPLImageFilter
 * Compute Vdisparity
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLVdisparity : public IPLImageFilter
{
	protected:
		unsigned int disparityRange;
		
	public :
		IPLVdisparity(const char * n, unsigned int dispRange);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_Vdisparity_H
