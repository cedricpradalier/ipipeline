
#include "ipipeline_modules/IPLGlcmMask.h"

IPLGlcmMask::IPLGlcmMask(const char * n,
		unsigned int div, int _dx, int _dy, int _threshold) : 
	IPLImageFilter(n), divider(div), 
	dx(_dx), dy(_dy), threshold(_threshold)
{
	winsize.width = 0;
	winsize.height = 0;
}

#define BIN_POWER 5
#define NBIN (1 << (8-BIN_POWER))

int IPLGlcmMask::glcm(const cv::Mat1b & img, int u0, int v0)
{
	int i,j;
	unsigned int P[NBIN][NBIN];
	memset(P,0,sizeof(P));
	
	/** Range precomputation **/
	int umin,umax,vmin,vmax;
	int imin,imax,jmin,jmax;
	if (dx < 0) {
		imin = -dx;
		imax = MIN(u0 + winsize.width,img.cols-10)-u0;
	} else { 
		imin = 0;
		imax = MIN(u0 + winsize.width-dx,img.cols-10-dx)-u0;
	}
	umin = u0 + imin;
	umax = u0 + imax;

	if (dy < 0) {
		jmin = - dy;
		jmax = MIN(v0 + winsize.height,img.rows-10)-v0;
	} else { 
		jmin = 0;
		jmax = MIN(v0 + winsize.height-dy,img.rows-10-dy)-v0;
	}
	vmin = v0 + jmin;
	vmax = v0 + jmax;

	/** Building of co-occurence matrix **/
	unsigned int idx1,idx2;
	for (i=umin;i<umax;i++) {
		for (j=vmin;j<vmax;j++) {
			idx1 = img(i,j) >> BIN_POWER;
			idx2 = img(i+dx,j+dy) >> BIN_POWER;
			P[idx1][idx2] += 1;
		}
	}
	
	/** Entropy computation **/
	unsigned int ent = 0;
	for (i=0;i<NBIN;i++) {
		for (j=0;j<NBIN;j++) {
			ent += (i-j)*(i-j)*P[i][j];
		}
	}
	return ent;
}

bool IPLGlcmMask::checkInput() const 
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
    for (unsigned int i=0;i<in.size();i++) {
        switch (in[i].type()) {
            case CV_8UC1:
                break;
            default : 
                return error(INVALID_TYPE);
        }
    }
	return true;
}


bool IPLGlcmMask::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    int u,v;

#warning This has not been retested
    outputVector.resize(in.size());

	for (i=0;i<in.size();i++) {
        outputVector[i].create(in[i].size(),CV_8UC1);
        winsize.width = in[i].cols/divider;
        winsize.height = in[i].rows/divider;
		for (u=10;u<in[i].cols-10-winsize.width;u+=winsize.width) {
			for (v=10;v<in[i].rows-10-winsize.height;v+=winsize.height) {
				int contrast = glcm((const cv::Mat1b &)(in[i]),u,v);
				//printf("constrast %dx%d : %d\n",u,v,contrast);
                cv::Mat1b m(outputVector[i],cv::Rect(u,v,winsize.width,winsize.height));
				if (contrast > threshold) {
                    m.setTo(0xFF);
				} else {
                    m.setTo(0x00);
				}
			}
		}
	}
	return true;
}

