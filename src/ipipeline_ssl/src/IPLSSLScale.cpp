
#include "ipipeline_ssl/IPLSSLScale.h"

IPLSSLScale::IPLSSLScale(const char * n, IPLSSLImage::SSL_DIRECTION dir, unsigned int win, double p) : 
	IPLImageProcessor(n), factor(p), win_size(win), direction(dir)
{
    idxsqrtp.resize(SSL_MAX_SCALE);
    idxp.resize(SSL_MAX_SCALE);
	double sqrtp = sqrt(p);
	
	for(unsigned int i=1; i<SSL_MAX_SCALE; i++){
		idxp[i] = (int) round(p*i);
		idxsqrtp[i] = (int) round(sqrtp*i);
	}	

    imageInputId = addInput("Image");
    sslInputId = addInput("SSL");
}

void IPLSSLScale::readConfig(Config *config)
{
	config->getDouble("factor",&factor);
	config->getUInt("win_size",&win_size);
}


bool IPLSSLScale::checkInput() const 
{
    IPLImageProcessor * input = getInput(imageInputId);
    ImageProcessorOutput image = input->getOutput();
	unsigned int i;
    for (i=0;i<image.size();i++) {
        if (image[i].type() != CV_16SC1) {
            return error(INVALID_TYPE);
        }
    }

    input = getInput(sslInputId);
    ImageProcessorOutput ssl = input->getOutput();
    for (i=0;i<ssl.size();i++) {
        if (ssl[i].type() != CV_32FC1) {
            return error(INVALID_TYPE);
        }
    }

    if (image.size() != ssl.size()) {
        return error(INCONSISTENT_INPUT_SIZE);
    }
	return true;
}


double IPLSSLScale::computeScale(const cv::Mat & image, double row, double col) {
    double smax = 0;
    unsigned int v = row;
    unsigned int u = col;
    unsigned int width = image.size().width;
    unsigned int height = image.size().height;
    float max = 0;
#define AT(x,y) image.at<int16_t>(x,y)
    if (direction & IPLSSLImage::SSL_RIGHT) {
        unsigned int bound = std::min(IPLSSLImage::SSL_MAX_SCALE, width-u);
        signed short acc=0;
        for(unsigned int k=1; k<bound; k++) {
            unsigned int idx1 = idxp[k];
            unsigned int idx2 = idxsqrtp[k];
            acc += abs( AT(v,u+idx2) - AT(v,u+k) ) - abs( AT(v,u+idx1) - AT(v,u+k) );
            float normalized_acc = acc / (k*k*sqrt(k));
            if (normalized_acc >= max) {
                max = normalized_acc;
                smax = std::max((double)k,smax);
            }
        }
    }
    if (direction & IPLSSLImage::SSL_LEFT) {
        unsigned int bound = std::min(IPLSSLImage::SSL_MAX_SCALE, u);
        signed short acc=0;
        for(unsigned int k=1; k<bound; k++) {
            unsigned int idx1 = idxp[k];
            unsigned int idx2 = idxsqrtp[k];
            acc += abs( AT(v,u-idx2) - AT(v,u-k) ) - abs( AT(v,u-idx1) - AT(v,u-k) );
            float normalized_acc = acc / (k*k*sqrt(k));
            if (normalized_acc >= max) {
                max = normalized_acc;
                smax = std::max((double)k,smax);
            }
        }
    }
    if (direction & IPLSSLImage::SSL_DOWN) {
        unsigned int bound = std::min(IPLSSLImage::SSL_MAX_SCALE, height-v);
        signed short acc=0;
        for(unsigned int k=1; k<bound; k++) {
            unsigned int idx1 = idxp[k];
            unsigned int idx2 = idxsqrtp[k];
            acc += abs( AT(v+idx2,u) - AT(v+k,u) ) - abs( AT(v+idx1,u) - AT(v+k,u) );
            float normalized_acc = acc / (k*k*sqrt(k));
            if (normalized_acc >= max) {
                max = normalized_acc;
                smax = std::max((double)k,smax);
            }
        }
    }
    if (direction & IPLSSLImage::SSL_UP) {
        unsigned int bound = std::min(IPLSSLImage::SSL_MAX_SCALE, v);
        signed short acc=0;
        for(unsigned int k=1; k<bound; k++) {
            unsigned int idx1 = idxp[k];
            unsigned int idx2 = idxsqrtp[k];
            acc += abs( AT(v-idx2,u) - AT(v-k,u) ) - abs( AT(v-idx1,u) - AT(v-k,u) );
            float normalized_acc = acc / (k*k*sqrt(k));
            if (normalized_acc >= max) {
                max = normalized_acc;
                smax = std::max((double)k,smax);
            }
        }
    }
#undef AT
    return smax;
}


bool IPLSSLScale::processInput()
{
    ImageProcessorOutput image = getInput(imageInputId)->getOutput();
    ImageProcessorOutput ssl = getInput(sslInputId)->getOutput();
	unsigned int i,k;
    outputVector.resize(ssl.size());
    for (i=0;i<image.size();i++) {
        cv::Size sz = ssl[i].size();
        sz.width = 3;
        outputVector[i].create(sz,CV_32FC1);

        for (k=0;(signed)k<sz.height;k++) {
            outputVector[i].at<float>(k,0) = ssl[i].at<float>(k,0);
            outputVector[i].at<float>(k,1) = ssl[i].at<float>(k,1);
            // printf("Considering %.2f %.2f: ",(double)ssl[i].at<float>(k,0), (double)ssl[i].at<float>(k,1));
            outputVector[i].at<float>(k,2) = computeScale(image[i], ssl[i].at<float>(k,1), ssl[i].at<float>(k,0));
            // printf("\tScale %.2f\n",(double)outputVector[i].at<float>(k,2));
        }
    }

	return true;
}

