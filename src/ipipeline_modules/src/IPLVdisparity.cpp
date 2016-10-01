#include "ipipeline_modules/IPLVdisparity.h"

IPLVdisparity::IPLVdisparity(const char * n, unsigned int dispRange) : 
	IPLImageFilter(n)
{
	disparityRange = dispRange;
}

bool IPLVdisparity::checkInput() const 
{
	IPLImageProcessor * input = getInput(deftInput);
	const ImageProcessorOutput * in = input->getOutput();
	if (!in->checkType()) return error(INCONSISTENT_TYPE);
	switch (in->type()) {
		case Int8u:
			break;
		case Int16s:
			break;
		default : 
			return error(INVALID_TYPE);
	}
	return true;
}

bool IPLVdisparity::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	const IppiImage * in = input->getOutput()->asImage();
	unsigned int i,j,k;
	
	reallocate(Int16s,disparityRange,input->height,input->nimages);

	// **** Compute V-disparity **** //
	switch (in->type()) {
		case Int8u :
			{
				const IppiImage8u * I = in->as8u();
				IppiImage16s * O = out->as16s();
				O->setto(0);
				for (k=0;k<nimages;k++) {
					for (i=0;i<I[k].width;i++) {
						for (j=0;j<I[k].height;j++) {
							if (I[k](i,j) > 0) {
								O[k](I[k](i,j),j) ++;
							}
						}
					}
				}
				break;
			}
		case Int16s :
			{
				const IppiImage16s * I = in->as16s();
				IppiImage16s * O = out->as16s();
				O->setto(0);
				for (k=0;k<nimages;k++) {
					for (i=0;i<I[k].width;i++) {
						for (j=0;j<I[k].height;j++) {
							if (I[k](i,j) > 0) {
								O[k](I[k](i,j),j) ++;
							}
						}
					}
				}
				break;
			}
		default :
			// not possible after checkinput
			break;
	}
	// out->saveAsMatrix("vdisp.mat");
	return true;
}

