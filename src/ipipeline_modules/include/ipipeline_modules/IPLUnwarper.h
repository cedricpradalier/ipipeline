#ifndef IPL_IMAGE_UNWARPER_H
#define IPL_IMAGE_UNWARPER_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLUnwarper : \see IPLImageFilter
 * Blur input using ippiFilterGauss and a 3x3 mask
 * input "Default" must output a set of Int8u or RGB8u image 
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLUnwarper : public IPLImageFilter
{
	protected:
		// enum {Cylindrical, ...} model
		IppiImage32f lx,ly;
		int interpMode;

		void updateTables(unsigned int cx, unsigned int cy, 
				unsigned int rmin, unsigned int rmax,
				unsigned int owidth, unsigned int oheight);
	public :
		IPLUnwarper(const char * n, unsigned int cx, unsigned int cy, 
				unsigned int rmin, unsigned int rmax, int intMode, 
				unsigned int owidth, unsigned int oheight);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_UNWARPER_H
