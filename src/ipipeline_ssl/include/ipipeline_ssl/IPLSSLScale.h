#ifndef IPL_SSL_SCALE_H
#define IPL_SSL_SCALE_H

#include <math.h>
#include "ipipeline_core/IPLImageFilter.h"
#include "ipipeline_ssl/IPLSSLImage.h"

/**
 * \class IPLSSLScale : \see IPLImageFilter
 * Find pixels of high circular self similarity
 * input "In" must output a Int8u image, the X component of the gradient
 * If a mask input is required, it must be a Int8u image, and will be used to 
 * select where the edges will be looked for.
 * output is a single Int16s image, with 255 in edge pixels.
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLSSLScale : public IPLImageProcessor
{
	protected:
        static const unsigned int SSL_MAX_SCALE = 300;
        double factor;
        unsigned int win_size;
        IPLSSLImage::SSL_DIRECTION direction;

        std::vector<int> idxsqrtp;
        std::vector<int> idxp;

        unsigned int imageInputId;
        unsigned int sslInputId;
        double computeScale(const cv::Mat & image, double row, double col);
	public :
		IPLSSLScale(const char * n, IPLSSLImage::SSL_DIRECTION dir, unsigned int win=30, double p=2./3.);

		virtual bool processInput();

		virtual bool checkInput() const;

        virtual void readConfig(Config *config);
};




#endif // IPL_SSL_SCALE_H
