#ifndef IPL_CIRCULAR_GRADIENT_REF_H
#define IPL_CIRCULAR_GRADIENT_REF_H

#include "ipipeline_core/IPLImageSourceWithView.h"
#include "ipipeline_modules/IPLCenterDependantModule.h"

/**
 * \class IPLCircularGradientRef : \see IPLImageSource
 * Implementation of an image source representing the gradient of 
 * an image
 * An image source has no input
 * output is a set of IppiImage
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLCircularGradientRef : public IPLImageSourceWithView, public IPLCenterDependantModule
{
	protected:
		virtual bool buildSourceImage() ;
		virtual bool updateImageCenter() ;
	public :
		virtual ~IPLCircularGradientRef();

		IPLCircularGradientRef(const char * n, 
				unsigned int width, unsigned int height,
				unsigned int deltax, unsigned int deltay);

		/** just calls IPLImageSourceWithView::processInput **/
		virtual bool processInput();
};




#endif // IPL_CIRCULAR_GRADIENT_REF_H
