#include <algorithm>

#include "ipipeline_modules/IPLRoiFuser.h"

#define MIN(a,b) (((int)(a)<(int)(b))?(a):(b))
#define MAX(a,b) (((int)(a)>(int)(b))?(a):(b))


IPLRoiFuser::IPLRoiFuser(const char * n, unsigned int ninp) : 
	IPLImageProcessor(n)
{
	_nimages = 1;
	ninput = ninp;
	unsigned int i;
	for (i=0;i<ninput;i++) {
		char tmp[128];
		sprintf(tmp,"image%03d",i);
		imageInputId.push_back(addInput(tmp));
	}	
	output = NULL;
}

IPLRoiFuser::~IPLRoiFuser()
{
	delete output;
}

bool IPLRoiFuser::checkInput() const 
{
	unsigned int i;
	OutputType t = Unknown;
	if (ninput < 1) return error("Need at least one input");
	for (i=0;i<ninput;i++) {
		IPLImageProcessor * input = getInput(imageInputId[i]);
		const ImageProcessorOutput * in = input->getOutput();
		// must be RGBA 8u
		if (!in->checkType()) return error(INCONSISTENT_TYPE);
		if (!in->isImage()) return error(INVALID_TYPE);
		if (t == Unknown) t = in->type();
		// all the image must be the same size
		if (t != in->type()) return error("All input types must be identical");
	}
		
	return true;
}

bool IPLRoiFuser::processInput()
{
	unsigned int i;
	const IppiImage * image[ninput];
	IppiRect roi[ninput];
	_width = _height = 0;
	for (i=0;i<ninput;i++) {
		// no check here, checkInput should have been called before
		IPLImageProcessor * in = getInput(imageInputId[i]);
		roi[i] = in->getRoi();
		image[i] = in->getOutput()->asImage();
		if (image[i]->width > _width) _width = image[i]->width;
		if (image[i]->height > _height) _height = image[i]->height;
	}

	// checkInput checked that we have at least 2 inputs

	// all image of the same type, checked in checkInput()
	switch (image[0]->type()) {
		case Int8u:
			{
				IppiImage8u * O;
				O = (output==NULL)?(new IppiImage8u):(output->as8u());
				O->allocate(width,height);
				output = O;

				for (i=0;i<ninput;i++) {
					const IppiImage8u * I = image[i]->as8u();
					IppiSize size = {roi[i].width,roi[i].height};
					ippiCopy_8u_C1R(I->roistart(roi[i].x,roi[i].y),I->bstep,
							O->pixels,O->bstep,size);
				}
			}
			break;
		case Int16s:
			{
				IppiImage16s * O;
				O = (output==NULL)?(new IppiImage16s):(output->as16s());
				O->allocate(width,height);
				output = O;

				for (i=0;i<ninput;i++) {
					const IppiImage16s * I = image[i]->as16s();
					IppiSize size = {roi[i].width,roi[i].height};
					ippiCopy_16s_C1R(I->roistart(roi[i].x,roi[i].y),I->bstep,
							O->pixels,O->bstep,size);
				}
			}
			break;
		case Int32s:
			{
				IppiImage32s * O;
				O = (output==NULL)?(new IppiImage32s):(output->as32s());
				O->allocate(width,height);
				output = O;

				for (i=0;i<ninput;i++) {
					const IppiImage32s * I = image[i]->as32s();
					int u,v;
					for (v=roi[i].y;v<roi[i].height;v++) {
						for (u=roi[i].x;u<roi[i].width;u++) {
							(*O)(u,v) = (*I)(u,v);
						}
					}
				}
			}
			break;
		case Float32:
			{
				IppiImage32f * O;
				O = (output==NULL)?(new IppiImage32f):(output->as32f());
				O->allocate(width,height);
				output = O;

				for (i=0;i<ninput;i++) {
					const IppiImage32f * I = image[i]->as32f();
					IppiSize size = {roi[i].width,roi[i].height};
					ippiCopy_32f_C1R(I->roistart(roi[i].x,roi[i].y),I->bstep,
							O->pixels,O->bstep,size);
				}
			}
			break;
		case RGB8u:
			{
				IppiImageRGB8u * O;
				O = (output==NULL)?(new IppiImageRGB8u):(output->asRGB8u());
				O->allocate(width,height);
				output = O;

				for (i=0;i<ninput;i++) {
					const IppiImageRGB8u * I = image[i]->asRGB8u();
					IppiSize size = {roi[i].width,roi[i].height};
					ippiCopy_8u_C3R(I->roistart(roi[i].x,roi[i].y),I->bstep,
							O->pixels,O->bstep,size);
				}
			}
			break;
		case RGBA8u:
			{
				IppiImageRGBA8u * O;
				O = (output==NULL)?(new IppiImageRGBA8u):(output->asRGBA8u());
				O->allocate(width,height);
				output = O;

				for (i=0;i<ninput;i++) {
					const IppiImageRGBA8u * I = image[i]->asRGBA8u();
					IppiSize size = {roi[i].width,roi[i].height};
					ippiCopy_8u_C4R(I->roistart(roi[i].x,roi[i].y),I->bstep,
							O->pixels,O->bstep,size);
				}
			}
			break;
		default :
			return false;
	}
	
	return true;
}
