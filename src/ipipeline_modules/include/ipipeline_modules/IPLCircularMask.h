#ifndef IPL_CIRCULAR_MASK_H
#define IPL_CIRCULAR_MASK_H

#include "ipipeline_core/IPLImageSourceWithView.h"
#include "ipipeline_modules/IPLCenterDependantModule.h"

/**
 * \class IPLCircularMask : \see IPLImageSource
 * Implementation of an image mask shaped as a torus
 * An image source has no input
 * output is a set of IppiImage
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLCircularMask : public IPLImageSourceWithView, public IPLCenterDependantModule
{
	protected:
		double outRad,inRad;
		double outcx,outcy;
		virtual bool buildSourceImage() ;
		virtual bool updateImageCenter();

		struct DeadZone{
			double angleCenter;
			double angleWidth;
		} ;
		std::vector<DeadZone> deadzones;
			
		bool testPixel(unsigned int i, unsigned int j) const;
	public :
		virtual ~IPLCircularMask();

		IPLCircularMask(const char * n, 
				unsigned int width, unsigned int height, 
				unsigned int deltax, unsigned int deltay, 
				double outterRad, double innerRad);

		IPLCircularMask(const char * n, 
				unsigned int width, unsigned int height, 
				unsigned int deltax, unsigned int deltay, 
				double outterRad, double innerRad,
				const std::vector<DeadZone> & dzones);

		/** just calls IPLImageSourceWithView::processInput **/
		virtual bool processInput();

		void readConfig(Config * cfg) ;
};




#endif // IPL_CIRCULAR_MASK_H
