#include <math.h>
#include "ipipeline_modules/IPLCircularMask.h"

IPLCircularMask::~IPLCircularMask()
{
}


IPLCircularMask::IPLCircularMask(const char * n,
		unsigned int width, unsigned int height, 
		unsigned int deltax, unsigned int deltay, 
		double outterRad, double innerRad) :
	IPLImageSourceWithView(n,IPL_ROLE_SOURCE,CV_8UC1,1,width+2*deltax,height+2*deltay,width,height)
{
	outcx = width/2+deltax;
	outcy = height/2+deltay;
	// printf("CMask center: %f %f\n",outcx,outcy);
	outRad = outterRad;
	inRad = innerRad;
	deadzones.clear();
	buildSourceImage();
	setImageCenter(width/2.0,height/2.0);
	// printf("IPLCircularMask: %s: %dx%d view %dx%d\n",name,width+2*deltax,height+2*deltay,width,height);
	// out->asImage()->printState(name);
}

IPLCircularMask::IPLCircularMask(const char * n,
		unsigned int width, unsigned int height, 
		unsigned int deltax, unsigned int deltay, 
		double outterRad, double innerRad,
		const std::vector<DeadZone> & dzones) :
	IPLImageSourceWithView(n,IPL_ROLE_SOURCE,CV_8UC1,1,width+2*deltax,height+2*deltay,width,height)
{
	outcx = width/2+deltax;
	outcy = height/2+deltay;
	// printf("CMask center: %f %f\n",outcx,outcy);
	outRad = outterRad;
	inRad = innerRad;
	deadzones = dzones;
	buildSourceImage();
	setImageCenter(width/2.0,height/2.0);
	// printf("IPLCircularMask: %s: %dx%d view %dx%d\n",name,width+2*deltax,height+2*deltay,width,height);
	// out->asImage()->printState(name);
}

bool IPLCircularMask::updateImageCenter()
{
	outcx = outputVector[0].cols/2;
	outcy = outputVector[0].rows/2;
	double x,y;
	assert(cx < outputVector[0].cols);
	assert(cy < outputVector[0].rows);
	x = outcx - cx;
	y = outcy - cy;
    cv::Rect lview = this->getView();
    lview.x = x; lview.y = y;
	this->setView(lview);
	return true;
}

bool IPLCircularMask::testPixel(unsigned int i, unsigned int j) const
{
	unsigned int k;
	double r = hypot(i-outcx,j-outcy);
	double alpha = atan2(j-outcy,i-outcx);
	if ((r < inRad) || (r > outRad)) return false;
	for (k=0;k<deadzones.size();k++) {
		if (fabs(remainder(alpha-deadzones[k].angleCenter,2*M_PI)) < 
				deadzones[k].angleWidth) {
			return false;
		}
	}
	return true;
}

bool IPLCircularMask::buildSourceImage()
{
	int i,j;
	// Here we don't use the variable center, because the variable center will
	// be used when changing the view
    // Allocation has been done in IPLImageSource
    cv::Mat_<uchar> & I = (cv::Mat_<uchar> &)(outputVector[0]);

    for (j=0;j<I.rows;j++) {
        for (i=0;i<I.cols;i++) {
            I(i,j) = testPixel(i,j)?255:0;
        }
    }
	return true;
}

bool IPLCircularMask::processInput()
{
	if(!IPLImageSourceWithView::processInput()) {
		return false;
	}
	// out->asImage()->saveAsMatrix(std::string(name)+"_out.mat");
	// view->saveAsMatrix(std::string(name)+"_view.mat");
	return true;
}

void IPLCircularMask::readConfig(Config * cfg) 
{
	unsigned int i;
	deadzones.clear();
	cfg->getDouble("innerRad",&inRad);
	cfg->getDouble("outterRad",&outRad);
	i = 0;
	while (1) {
		DeadZone dz;
		char varname[128];
		sprintf(varname,"deadZoneAngle%d",i);
		if (!cfg->getDouble(varname,&dz.angleCenter)) {
			break;
		}
		dz.angleCenter *= M_PI/180;

		sprintf(varname,"deadZoneWidth%d",i);
		if (!cfg->getDouble(varname,&dz.angleWidth)) {
			break;
		}
		dz.angleWidth *= M_PI/180;

		deadzones.push_back(dz);
		i++;
	}
	buildSourceImage();
}


