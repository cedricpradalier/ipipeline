#ifndef IPL_CIRCULAR_GAUSSIAN_H
#define IPL_CIRCULAR_GAUSSIAN_H

#include "ipipeline_core/IPLImageSource.h"
#include "ipipeline_modules/IPLCenterDependantModule.h"

/**
 * \class IPLCircularGaussian : \see IPLImageSource
 * Image source representing a gaussian centered on the image
 * An image source has no input
 * output is a set of IppiImage
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLCircularGaussian : public IPLImageSource, public IPLCenterDependantModule
{
	protected:
		bool zeroOut;
		double negHalfInvSigmaSqr;
		virtual bool updateImageCenter();
	public :
		virtual ~IPLCircularGaussian();
		/**
		 * Constructor: with or without fixed cx,cy buffer
		 * **/
		IPLCircularGaussian(const char * n, int type,
				unsigned int width, unsigned int height, 
				double sigma, double cx, double cy, bool zeroOutside=true);

		IPLCircularGaussian(const char * n, int type, 
				unsigned int width, unsigned int height, double sigma, 
				bool zeroOutside=true);

		/** does nothing **/
		virtual bool processInput();

		virtual void readConfig(Config * config);
};




#endif // IPL_CIRCULAR_GAUSSIAN_H
