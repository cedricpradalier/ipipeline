#ifndef IPL_DEMUX_H
#define IPL_DEMUX_H

#include "IPLImageProcessor.h"

/**
 * \class IPLDemux 
 * Select one image in its input set
 * input "Default" must output any number of image.
 * output is one image with same type as Default.
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLDemux : public IPLImageProcessor
{
	protected : 
		unsigned int deftInput;
		unsigned int which;
		unsigned int & rwhich;	

	public :
		/**
		 * \arg _which: index of the input image to select
		 * **/
		IPLDemux(const char * n, unsigned int _which);
		/**
		 * \arg _which: reference to the index of the input image 
		 * to select. Can vary dynamically.
		 * **/
		IPLDemux(const char * n, unsigned int * _which);

		virtual bool processInput();

		virtual bool checkInput() const;

        void selectInput(unsigned int _which) {
            rwhich = _which;
        }
};



#endif // IPL_DEMUX_H
