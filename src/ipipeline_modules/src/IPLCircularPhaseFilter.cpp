#include <math.h>

#include "ipipeline_modules/IPLCircularPhaseFilter.h"

IPLCircularPhaseFilter::IPLCircularPhaseFilter(const char * n, double angleThresh) : 
	IPLImageFilter(n)
{
	angleThreshold = angleThresh;
	fixedCenter = false;
	needsRefUpdate = true;
}

IPLCircularPhaseFilter::IPLCircularPhaseFilter(const char * n, double angleThresh, double x, double y) : 
	IPLImageFilter(n)
{
	angleThreshold = angleThresh;
	setImageCenter(x,y);
}

bool IPLCircularPhaseFilter::checkInput() const 
{
	IPLImageProcessor * input = getInput(deftInput);
	const ImageProcessorOutput * in = input->getOutput();
	if (!in->checkType()) return error(INCONSISTENT_TYPE);
	switch (in->type()) {
		case Float32:
			break;
		default : 
			return error(INVALID_TYPE);
	}

	return true;
}

bool IPLCircularPhaseFilter::updateImageCenter()
{
	fixedCenter = true;
	needsRefUpdate = true;
	return true;
}

bool IPLCircularPhaseFilter::updateReferenceOrientation()
{
	// Assumption: R has been resized already;
	unsigned int i,j;
	for (j=0;j<R.height;j++) {
		for (i=0;i<R.width;i++) {
			R(i,j) = atan2(j-cy,i-cx);
		}
	}
	return true;
}

bool IPLCircularPhaseFilter::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	const IppiImage * in = input->getOutput()->asImage();
	unsigned int i,j,k;

	if (needsRefUpdate || (in->width != width) || (in->height != height)) {
		if (!fixedCenter) {
			cx = in->width/2.0;
			cy = in->height/2.0;
		}
		R.allocate(in->width,in->height);
		updateReferenceOrientation();
		needsRefUpdate = false;
	}
	reallocate(Int8u, in->width,in->height,input->nimages);

	switch (in->type()) {
		case Float32 :
			{
				const IppiImage32f * I = in->as32f();
				IppiImage8u * O = out->as8u();
				for (k=0;k<nimages;k++) {
					for (j=0;j<I[k].height;j++) {
						for (i=0;i<I[k].width;i++) {
							double d = fabs(remainder(I[k](i,j)-R(i,j),M_PI));
							O[k](i,j) = (d < angleThreshold)?255:0;
						}
					}
				}
				break;
			}
		default :
			// not possible after checkinput
			break;
	}
	return true;
}

