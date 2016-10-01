#ifndef IPL_CIRCULAR_PHASE_FILTER_H
#define IPL_CIRCULAR_PHASE_FILTER_H

#include "ipipeline_core/IPLImageFilter.h"
#include "ipipeline_modules/IPLCenterDependantModule.h"

/**
 * \class IPLCircularPhaseFilter : \see IPLImageFilter
 * Use the gradient phase received from a IPLGradientPhase module
 * and returns a binary image with 1 where the gradient phase is aligned 
 * toward the image center
 * input "default" must output a set of Float32 images
 * output is as many single Int8u image
 * \see IPLImageProcessor for virtual function 
 * TODO: check if there is a way to implement this function using IPP
 * primitives. We are missing remainder(x,m) for now.
 * **/
class IPLCircularPhaseFilter : public IPLImageFilter, IPLCenterDependantModule
{
	protected:
		double angleThreshold;
		bool fixedCenter, needsRefUpdate;
		IppiImage32f R;
		bool updateReferenceOrientation();
		virtual bool updateImageCenter();
	public :
		IPLCircularPhaseFilter(const char * n, double angleThresh);
		IPLCircularPhaseFilter(const char * n, double angleThresh, 
				double cx, double cy);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_CIRCULAR_PHASE_FILTER_H
