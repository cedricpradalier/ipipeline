
#include "ipipeline_modules/IPLSobelEdges.h"

IPLSobelEdges::IPLSobelEdges(const char * n,float threshold,  bool withMask) : 
	IPLImageProcessor(n)
{
	_nimages = 1;
	optimalThreshold = false;
	sobThreshold = threshold;
	inputX = addInput("X");
	inputY = addInput("Y");
	inputMag = addInput("Mag");
	useMask = withMask;
	if (withMask) {
		inputMask = addInput("Mask");
	}
}

IPLSobelEdges::IPLSobelEdges(const char * n,bool withMask) : 
	IPLImageProcessor(n)
{
	_nimages = 1;
	optimalThreshold = true;
	sobThreshold = 0;
	inputX = addInput("X");
	inputY = addInput("Y");
	inputMag = addInput("Mag");
	useMask = withMask;
	if (withMask) {
		inputMask = addInput("Mask");
	}
}

bool IPLSobelEdges::computeOptimalThreshold(const IppiImage32f * mag) {
#if 1
	Ipp64f mean;
	ippiMean_32f_C1R(mag->pixels,mag->bstep,mag->size,&mean,ippAlgHintNone);
	sobThreshold = 4*mean;
#else
	unsigned int i;
	Ipp32s hist[64];
	Ipp32f levels[65];
	Ipp32f gmax,gmin;
	ippiMinMax_32f_C1R(mag->pixels,mag->bstep,mag->size,&gmin,&gmax);
	double dg = ((gmax-gmin)/64);
	for (i=0;i<64;i++) {
		levels[i] = gmin + i*dg;
	}
	levels[64] = gmax;
	ippiHistogramRange_32f_C1R(mag->pixels,mag->bstep,mag->size,
			hist,levels,65);

	i=0;
	unsigned int sum = 0;
	unsigned int npixels = (unsigned int)round(0.9*mag->width*mag->height);
	while ((sum < npixels) && (i<64)) {
		sum += hist[i++];
	}
	sobThreshold = gmin + i*dg;
#endif
	// printf("Sobel: optimal threshold : %f\n",sobThreshold);
	return true;
}

bool IPLSobelEdges::checkInput() const 
{
	unsigned int inputs[3] = {inputX,inputY,inputMag};
	unsigned int i;
	for (i=0;i<3;i++) {
		IPLImageProcessor * input = getInput(inputs[i]);
		const ImageProcessorOutput * in = input->getOutput();
		if (!in->checkType()) return error(INCONSISTENT_TYPE);
		switch (in->type()) {
			case Float32:
				break;
			default : 
				return error(INVALID_TYPE);
		}
	}
	if (!sameSize(getInput(inputX), getInput(inputY))) {
		return error("Invalid image size for input Y/X");
	}
	if (!sameSize(getInput(inputX), getInput(inputMag))) {
		return error("Invalid image size for input Mag");
	}
	if (useMask) {
		IPLImageProcessor * input = getInput(inputMask);
		const ImageProcessorOutput * in = input->getOutput();
		if (!in->checkType()) return error(INCONSISTENT_TYPE);
		switch (in->type()) {
			case Int8u:
				break;
			default : 
				return error(INVALID_TYPE);
		}
		if (!sameSize(getInput(inputX), getInput(inputMask))) {
			return error("Invalid image size for input Mask");
		}
	}
	return true;
}

bool IPLSobelEdges::processInput()
{
	IPLImageProcessor * inpX = getInput(inputX);
	const IppiImage32f * inX = inpX->getOutput()->as32f();
	IPLImageProcessor * inpY = getInput(inputY);
	const IppiImage32f * inY = inpY->getOutput()->as32f();
	IPLImageProcessor * inpN = getInput(inputMag);
	const IppiImage32f * inN = inpN->getOutput()->as32f();
	unsigned int i,j;
	
	if (optimalThreshold) {
		computeOptimalThreshold(inN);
	}

	_width = inX->width;
	_height = inX->height;
	out.allocate(width,height);
	// First, take care of borders
	for (i=0;i<width;i++) {
		out(i,0) = out(i,height-1) = 0;
	} 
	for (i=0;i<height;i++) {
		out(0,i) = out(width-1,i) = 0;
	} 

	// Then run the show
	if (useMask) {
		IPLImageProcessor * inpM = getInput(inputMask);
		const IppiImage8u * inM = inpM->getOutput()->as8u();
		for (j=1;j<height-1;j++) {
			for (i=1;i<width-1;i++) {
				if (!(*inM)(i,j)) {
					out(i,j)=0;
				} else {
					out(i,j)=isSobelEdge(*inX,*inY,*inN,i,j)?255:0;
				}
			}
		}
	} else {
		for (j=1;j<height-1;j++) {
			for (i=1;i<width-1;i++) {
				out(i,j)=isSobelEdge(*inX,*inY,*inN,i,j)?255:0;
			}
		}
	}

	return true;
}

