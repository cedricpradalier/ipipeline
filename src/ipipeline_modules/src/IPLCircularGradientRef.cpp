#include <math.h>
#include "ipipeline_modules/IPLCircularGradientRef.h"

IPLCircularGradientRef::~IPLCircularGradientRef()
{
}


IPLCircularGradientRef::IPLCircularGradientRef(const char * n, 
		unsigned int width, unsigned int height,
		unsigned int deltax, unsigned int deltay) :
	IPLImageSourceWithView(n,Float32,3,width+2*deltax,height+2*deltay,width,height)
{
	buildSourceImage();
	setImageCenter(this->width/2.0,this->height/2.0);
}

bool IPLCircularGradientRef::updateImageCenter()
{
	double outcx,outcy;
	IppiImage32f *O = out->as32f();
	outcx = O->width/2;
	outcy = O->height/2;
	double x,y;
	x = outcx - cx;
	y = outcy - cy;
	this->setView((int)x,(int)y);
	return true;
}

bool IPLCircularGradientRef::buildSourceImage()
{
	unsigned int i,j;
	double cx,cy;
	IppiImage32f *O = out->as32f();
	// Here we don't use the variable center, because the variable center will
	// be used when changing the view
	cx = O->width/2;
	cy = O->height/2;
	printf("Cx %f Cy %f\n",cx,cy);
	for (j=0;j<O->height;j++) {
		for (i=0;i<O->width;i++) {
#if 1
			double n = hypot(j-cy,i-cx);
			double dx = (double)i - cx;
			double dy = (double)j - cy;
			O[0](i,j) = dx/n;
			O[1](i,j) = dy/n;
			dx = fabs(dx); dy = fabs(dy);
			double t = (dx>dy)?(dy/dx):(dx/dy);
			O[2](i,j) = isnan(t)?0:t;
#else
			double x = i, y = j;
			double n = atan2(y-cy,x-cx);
			double dx = cos(n);
			double dy = sin(n);
			O[0](i,j) = dx;
			O[1](i,j) = dy;
			O[2](i,j) = fabs((dx>dy)?(dy/dx):(dx/dy));
			if (abs(j-O->height/2)<2) {
				printf("P %.1f %.1f A %.2f CS %.2f %.2f\n",x,y,n*180/M_PI,O[0](i,j),O[1](i,j));
			}
#endif
		}
	}
	return true;
}

bool IPLCircularGradientRef::processInput()
{
	return IPLImageSourceWithView::processInput();
}



