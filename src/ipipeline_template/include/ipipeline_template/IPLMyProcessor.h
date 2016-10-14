#ifndef IPL_MY_PROCESSOR_H
#define IPL_MY_PROCESSOR_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLMyProcessor : \see IPLImageFilter
 * Template class for a customized image processor, inheriting from ImageFilter
 * i.e. it receives an image and output one.
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLMyProcessor : public IPLImageFilter
{
	protected:
        // Add local variables here
        double local_variable;
	public :
		IPLMyProcessor(const char * name);

		virtual bool processInput();

		virtual bool checkInput() const;

        virtual void readConfig(Config *config);
};




#endif // IPL_MY_PROCESSOR_H
