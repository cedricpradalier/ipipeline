#include <math.h>
#include "ipipeline_modules/IPLCircularGaussian.h"

IPLCircularGaussian::~IPLCircularGaussian()
{
}

IPLCircularGaussian::IPLCircularGaussian(const char * n, int type, unsigned int width, unsigned int height, 
				double sig, double cx, double cy, bool zeroOutside) :
	IPLImageSource(n,IPL_ROLE_SOURCE,type,1,width,height)
{
	zeroOut = zeroOutside;
	negHalfInvSigmaSqr = -0.5/(sig*sig);
	setImageCenter(cx,cy);
}

IPLCircularGaussian::IPLCircularGaussian(const char * n, int type, unsigned int width, unsigned int height, 
		double sig, bool zeroOutside) :
	IPLImageSource(n,IPL_ROLE_SOURCE,type,1,width,height)
{
	zeroOut = zeroOutside;
	negHalfInvSigmaSqr = -0.5/(sig*sig);
	setImageCenter(width/2.0,height/2.0);
}

void IPLCircularGaussian::readConfig(Config *config)
{
	double sigma = -sqrt(2*negHalfInvSigmaSqr);
	config->getDouble("sigma",&sigma);
	config->getBool("zeroOut",&zeroOut);
	negHalfInvSigmaSqr = -0.5/(sigma*sigma);
}

bool IPLCircularGaussian::updateImageCenter()
{
	int i,j;
    cv::Mat_<float> O(outputVector[0].size(),CV_32FC1);
    for (j=0;j<O.rows;j++) {
        for (i=0;i<O.cols;i++) {
            double r = hypot(i-cx,j-cy);
            if (zeroOut && (r > (O.cols/2))) {
                O(i,j) = 0;
            } else {
                O(i,j) = exp(negHalfInvSigmaSqr*(r*r));
            }
        }
    }
    int ch = outputVector[0].channels();
    if (ch == 1) {
        O.convertTo(outputVector[0],outputVector[0].type());
    } else {
        cv::Mat channels[ch];
        O.convertTo(channels[0],outputVector[0].type());
        for (int i=1;i<ch;i++) {
            channels[i] = channels[0];
        }
        cv::merge(channels,ch,outputVector[0]);
    }
	return true;
}

bool IPLCircularGaussian::processInput()
{
	// this one is always ready, since it only output its image from
	// memory
	return true;
}



