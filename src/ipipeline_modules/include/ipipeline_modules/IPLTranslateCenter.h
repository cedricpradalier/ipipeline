#ifndef IPL_TRANSLATE_CENTER_H
#define IPL_TRANSLATE_CENTER_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLTranslateCenter : \see IPLImageFilter
 * input "Default" must output a set of Int8u or RGB8u image 
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLTranslateCenter : public IPLImageFilter
{
	protected:
		unsigned int centerPosId;

	public :
		IPLTranslateCenter(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_TRANSLATE_CENTER_H
