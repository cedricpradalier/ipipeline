
#include "ipipeline_ssl/IPLSSLImage.h"

const unsigned int IPLSSLImage::SSL_MAX_SCALE = 300;

IPLSSLImage::IPLSSLImage(const char * n, SSL_DIRECTION dir, unsigned int win, double p) : 
	IPLImageFilter(n), factor(p), win_size(win), direction(dir)
{
    idxsqrtp.resize(SSL_MAX_SCALE);
    idxp.resize(SSL_MAX_SCALE);
	double sqrtp = sqrt(p);
	
	for(unsigned int i=1; i<SSL_MAX_SCALE; i++){
		idxp[i] = (int) round(p*i);
		idxsqrtp[i] = (int) round(sqrtp*i);
	}	
}

void IPLSSLImage::readConfig(Config *config)
{
	config->getDouble("factor",&factor);
	config->getUInt("win_size",&win_size);
}


bool IPLSSLImage::checkInput() const 
{
    IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    for (i=0;i<in.size();i++) {
        if (in[i].type() != CV_16SC1) {
            return error(INVALID_TYPE);
        }
    }
	return true;
}




bool IPLSSLImage::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    outputVector.resize(in.size());
    for (i=0;i<in.size();i++) {
        outputVector[i].create(in[i].size(),CV_16SC1);
        // outputVector[i].create(in[i].size(),CV_32FC1);
        outputVector[i] = cv::Scalar(0);

        unsigned int width = in[i].size().width;
        unsigned int height = in[i].size().height;

#if 0
        for(unsigned int y=win_size; (int)y<height-win_size-1; y++) {
            for(unsigned int x=win_size; (int)x<width-win_size-1;x++) {
                unsigned short s0=0;
                unsigned short s1=0;

                short res = 0;

                s1 = s0 = 0;
                for(unsigned k=1; k<win_size; k++)
                {
                    int idx1 = idxp[k];
                    int idx2 = idxsqrtp[k];
                    
#define AT(x,y) ((short)(in[i].at<unsigned char>(x,y)))
                    s0 += abs( AT(y,x+idx1) - AT(y,x+k) );
                    s0 += abs( AT(y,x-idx1) - AT(y,x-k) );
                    s0 += abs( AT(y+idx1,x) - AT(y+k,x) );
                    s0 += abs( AT(y-idx1,x) - AT(y-k,x) );
                                                          
                    s1 += abs( AT(y,x+idx2) - AT(y,x+k) );
                    s1 += abs( AT(y,x-idx2) - AT(y,x-k) );
                    s1 += abs( AT(y+idx2,x) - AT(y+k,x) );
                    s1 += abs( AT(y-idx2,x) - AT(y-k,x) );
#undef AT
                }
                res += s1-s0;

                outputVector[i].at<short>(y,x) = res; // (res<<8)/(4*win_size);
            }
        }
#else
        cv::Mat Ic = in[i](cv::Range(win_size,height-win_size-1),
                    cv::Range(win_size,width-win_size-1));
        cv::Mat Oc = outputVector[i](cv::Range(win_size,height-win_size-1),
                    cv::Range(win_size,width-win_size-1));
        for(unsigned k=1; k<win_size; k++)
        {
            int idx1 = idxp[k];
            int idx2 = idxsqrtp[k];
#define AT(x,y) (in[i](cv::Range(win_size+y,height-win_size-1+y), cv::Range(win_size+x,width-win_size-1+x)))
            if (direction & SSL_RIGHT) {
                Oc += abs(AT(0,k) - AT(0,idx2));
                Oc -= abs(AT(0,k) - AT(0,idx1));
            }

            if (direction & SSL_LEFT) {
                Oc += abs(AT(0,-k) - AT(0,-idx2));
                Oc -= abs(AT(0,-k) - AT(0,-idx1));
            }

            if (direction & SSL_DOWN) {
                Oc += abs(AT(k,0) - AT(idx2,0));
                Oc -= abs(AT(k,0) - AT(idx1,0));
            }

            if (direction & SSL_UP) {
                Oc += abs(AT(-k,0) - AT(-idx2,0));
                Oc -= abs(AT(-k,0) - AT(-idx1,0));
            }
#undef AT
        }

#endif
    }

	return true;
}

