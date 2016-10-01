#include <math.h>
#include <ippcv.h>

#include "ipipeline_modules/IPLFilteredGradMagnitude.h"

// Adds 2.7 ms in -O0, but remove 0.2 in -O3
#define PHASEFILT_OUTPUT_FLOAT

IPLFilteredGradMagnitude::IPLFilteredGradMagnitude(const char * n) : 
	IPLImageProcessor(n)
{
	inputX = addInput("X");
	inputY = addInput("Y");
	inputM = addInput("Mask");
	out = NULL;
	_nimages = 1;
}

IPLFilteredGradMagnitude::~IPLFilteredGradMagnitude() 
{
	freeOutput();
}

void IPLFilteredGradMagnitude::reallocate(unsigned int w, unsigned int h)
{
	if (out == NULL) {
		out = new IppiImage32f; 
	}
	if ((width!=w) || (height!=h)) {
		_width = w;
		_height = h;
		out->as32f()->allocate(width,height); 
	}
}

void IPLFilteredGradMagnitude::freeOutput()
{
	if (out != NULL) {
		delete out->as32f(); 
		out = NULL;
	}
	_width = 0;
	_height = 0;
	_nimages = 0;
}


const ImageProcessorOutput * IPLFilteredGradMagnitude::getOutput()const
{
	return out;
}


bool IPLFilteredGradMagnitude::checkInput() const 
{
	IPLImageProcessor * inpx = getInput(inputX);
	const ImageProcessorOutput * inx = inpx->getOutput();
	if (!inx->checkType()) return error(INCONSISTENT_TYPE);
	switch (inx->type()) {
		case Float32:
			break;
		default : 
			return error(INVALID_TYPE);
	}

	IPLImageProcessor * inpy = getInput(inputY);
	const ImageProcessorOutput * iny = inpy->getOutput();
	if (!iny->checkType()) return error(INCONSISTENT_TYPE);
	switch (iny->type()) {
		case Float32:
			break;
		default : 
			return error(INVALID_TYPE);
	}

	if (inx->type() != iny->type()) {
		return error(INVALID_TYPE);
	}
	const IppiImage *Ix = inx->asImage();
	const IppiImage *Iy = iny->asImage();
	if ((Ix->width != Iy->width)||(Ix->height!=Iy->height)) {
		return error(INVALID_IMAGE_SIZE);
	}

	IPLImageProcessor * inpm = getInput(inputM);
	const ImageProcessorOutput * inm = inpm->getOutput();
	if (!inm->checkType()) return error(INCONSISTENT_TYPE);
	switch (inm->type()) {
#ifdef PHASEFILT_OUTPUT_FLOAT
		case Float32:
#else
		case Int8u:
#endif
			break;
		default : 
			return error(INVALID_TYPE);
	}

	const IppiImage *Im = inm->asImage();
	if ((Im->width != Ix->width)||(Im->height!=Ix->height)) {
		return error(INVALID_IMAGE_SIZE);
	}

	return true;
}

bool IPLFilteredGradMagnitude::processInput()
{
	IPLImageProcessor * inpx = getInput(inputX);
	const IppiImage * inx = inpx->getOutput()->asImage();
	IPLImageProcessor * inpy = getInput(inputY);
	const IppiImage * iny = inpy->getOutput()->asImage();
	IPLImageProcessor * inpm = getInput(inputM);
	const IppiImage * inm = inpm->getOutput()->asImage();
	unsigned int i,j;

	reallocate(inx->width,inx->height); // same as iny, by checkInput

	switch (inx->type()) { // same as iny, by checkInput
		case Float32 :
			{
				const IppiImage32f * Ix = inx->as32f();
				const IppiImage32f * Iy = iny->as32f();
#ifdef PHASEFILT_OUTPUT_FLOAT
				// Adds 2.7 ms
				const IppiImage32f * Im = inm->as32f();
#else
				const IppiImage8u * Im = inm->as8u();
#endif
				IppiImage32f * O = out->as32f();
				ippiSet_32f_C1R(0,O[0].pixels,O[0].bstep,O[0].size);
				for (j=0;j<Ix->height;j++) {
					for (i=0;i<Ix->width;i++) {
						if (Im[0](i,j)>1.0) {
							float x = Ix[0](i,j);
							float y = Iy[0](i,j);
							O[0](i,j) = sqrt(x*x+y*y);
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

