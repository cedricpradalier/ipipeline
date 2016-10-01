#ifndef IPL_SSL_IMAGE_H
#define IPL_SSL_IMAGE_H

#include <math.h>
#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLSSLImage : \see IPLImageFilter
 * Find pixels of high circular self similarity
 * input "In" must output a Int8u image, the X component of the gradient
 * If a mask input is required, it must be a Int8u image, and will be used to 
 * select where the edges will be looked for.
 * output is a single Int16s image, with 255 in edge pixels.
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLSSLImage : public IPLImageFilter
{
    public:
        typedef enum {
            SSL_RIGHT=1,
            SSL_LEFT=2,
            SSL_UP=4,
            SSL_DOWN=8,
            SSL_HORIZONTAL=3,
            SSL_VERTICAL=12,
            SSL_ALL=15
        } SSL_DIRECTION;
        static const unsigned int SSL_MAX_SCALE;
	protected:
        double factor;
        unsigned int win_size;
        SSL_DIRECTION direction;

        std::vector<int> idxsqrtp;
        std::vector<int> idxp;
	public :
		IPLSSLImage(const char * n, SSL_DIRECTION dir, unsigned int win=30, double p=2./3.);

		virtual bool processInput();

		virtual bool checkInput() const;

        virtual void readConfig(Config *config);
};




#endif // IPL_SSL_IMAGE_H
