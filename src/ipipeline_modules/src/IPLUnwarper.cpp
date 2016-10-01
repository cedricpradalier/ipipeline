#include <math.h>


#include "ipipeline_modules/IPLUnwarper.h"

IPLUnwarper::IPLUnwarper(const char * n, unsigned int cx, unsigned int cy, 
				unsigned int rmin, unsigned int rmax, int intMode,
				unsigned int owidth, unsigned int oheight) : 
	IPLImageFilter(n)
{
	interpMode = intMode;
	_width = owidth;
	_height = oheight;
	updateTables(cx,cy,rmin,rmax,owidth,oheight);
}


void IPLUnwarper::updateTables(unsigned int cx, unsigned int cy, 
				unsigned int rmin, unsigned int rmax,
				unsigned int owidth, unsigned int oheight)
{
	unsigned int i, j;
	lx.allocate(owidth,oheight);
	ly.allocate(owidth,oheight);
	for (j=0;j<oheight;j++) {
		for (i=0;i<owidth;i++) {
			double r = rmin + (double)(j*(rmax-rmin))/oheight;
			double theta = i * 2*M_PI/owidth;
			lx(i,j) = cx + r*cos(theta);
			ly(i,j) = cy + r*sin(theta);
		}
	}
	lx.saveAsMatrix("lx.mat");
	ly.saveAsMatrix("ly.mat");
}

bool IPLUnwarper::checkInput() const 
{
	IPLImageProcessor * input = getInput(deftInput);
	const ImageProcessorOutput * in = input->getOutput();
	if (!in->checkType()) return error(INCONSISTENT_TYPE);
	switch (in->type()) {
		case RGBA8u:
		case RGB8u:
		case Int8u:
			break;
		default : 
			return error(INVALID_TYPE);
	}
	return true;
}

bool IPLUnwarper::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	const IppiImage * in = input->getOutput()->asImage();
	unsigned int i;

	reallocate(in->type(),lx.width, lx.height,input->nimages);

	switch (in->type()) {
		case Int8u :
			{
				const IppiImage8u * I = in->as8u();
				const IppiImage8u * O = out->as8u();
				for (i=0;i<nimages;i++) {
					ippiRemap_8u_C1R(I[i].pixels,I[i].size, I[i].bstep, I[i].rect,
							lx.pixels,lx.bstep, ly.pixels, ly.bstep, 
							O[i].pixels, O[i].bstep, O[i].size,	interpMode);
				}
				break;
			}
		case RGB8u :
			{
				const IppiImageRGB8u * I = in->asRGB8u();
				const IppiImageRGB8u * O = out->asRGB8u();
				for (i=0;i<nimages;i++) {
					ippiRemap_8u_C3R(I[i].pixels,I[i].size, I[i].bstep, I[i].rect,
							lx.pixels,lx.bstep, ly.pixels, ly.bstep, 
							O[i].pixels, O[i].bstep, O[i].size,	interpMode);
				}
				break;
			}
		case RGBA8u :
			{
				const IppiImageRGBA8u * I = in->asRGBA8u();
				const IppiImageRGBA8u * O = out->asRGBA8u();
				for (i=0;i<nimages;i++) {
					ippiRemap_8u_C4R(I[i].pixels,I[i].size, I[i].bstep, I[i].rect,
							lx.pixels,lx.bstep, ly.pixels, ly.bstep, 
							O[i].pixels, O[i].bstep, O[i].size,	interpMode);
				}
				break;
			}
		default :
			// not possible after checkinput
			break;
	}
	return true;
}

