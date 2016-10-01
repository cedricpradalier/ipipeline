#ifndef IPL_SOBEL_EDGES_H
#define IPL_SOBEL_EDGES_H

#include <math.h>
#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLSobelEdges : \see IPLImageFilter
 * Find edges pixels from the image gradient
 * input "X" must output a Float32 image, the X component of the gradient
 * input "Y" must output a Float32 image, the Y component of the gradient
 * input "Mag" must output a Float32 image, a monotonic function (sqr) of the
 * gradient norm, can be x*x+y*y, as long as comparison are preserved. If
 * optimal threshold is required, then Mag must be the square of the gradient
 *
 * If a mask input is required, it must be a Int8u image, and will be used to 
 * select where the edges will be looked for.
 * output is a single Int8u image, with 255 in edge pixels.
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLSobelEdges : public IPLImageProcessor
{
	protected:
		bool useMask;

		bool optimalThreshold;
		Ipp32f sobThreshold;
		unsigned int inputX, inputY, inputMag, inputMask;
		IppiImage8u out;

		inline bool isSobelEdge(const IppiImage32f & gx, const IppiImage32f & gy,
				const IppiImage32f & gn, unsigned int i, unsigned int j) {
			float x,y,n,d;
			// Warning no limit checking
			x = gx(i,j); y = gy(i,j); n = gn(i,j);
			if ((x==0)&&(y==0)) return false;
			if (n < sobThreshold) return false;
			if (x==0) { // Tested OK
				float no, so;
				so = gn(i,j+1);
				no = gn(i,j-1);
				return (so<=n) && (no<=n);
			} else if (y == 0) { // Tested OK
				float we, ea;
				we = gn(i-1,j);
				ea = gn(i+1,j);
				return (we<=n) && (ea<=n);
			} else if ((y<=0 && x>-y)  || (y>=0 && x<-y)) { // Tested OK
				float we, sw, ea, ne;
				d = fabs(y/x);
				we = gn(i-1,j); sw = gn(i-1,j+1); 
				ea = gn(i+1,j); ne = gn(i+1,j-1); 
				return ((we*(1-d)+sw*d)<=n) && ((ea*(1-d)+ne*d)<=n);
			} else if ((x>0 && -y>=x)  || (x<0 && -y<=x)) { // Tested OK
				float no, se, so, nw;
				d = fabs(x/y);
				so = gn(i,j+1); se = gn(i+1,j+1); 
				no = gn(i,j-1); nw = gn(i-1,j-1); 
				return ((so*(1-d)+se*d)<=n) && ((no*(1-d)+nw*d)<=n);
			} else if ((x<=0 && x>y) || (x>=0 && x<y)) { // Tested OK
				float no, ne, so, sw;
				d = fabs(x/y);
				so = gn(i,j+1); sw = gn(i-1,j+1); 
				no = gn(i,j-1); ne = gn(i+1,j-1); 
				return ((no*(1-d)+ne*d)<=n) && ((so*(1-d)+sw*d)<=n);
			} else if ((y<0 && x<=y) || (y>0 && x>=y)) { // Tested OK
				float we, se, ea, nw;
				d = fabs(y/x);
				we = gn(i-1,j); nw = gn(i-1,j-1); 
				ea = gn(i+1,j); se = gn(i+1,j+1); 
				return ((we*(1-d)+nw*d)<=n) && ((ea*(1-d)+se*d)<=n);
			} else {
#ifndef NDEBUG
				bool validIfCase=false;
#endif
				assert(validIfCase);
				return false;
			}
		}

		bool computeOptimalThreshold(const IppiImage32f *mag);
	public :
		// Use an optimal threshold, but Mag, must be the square of the
		// gradient
		IPLSobelEdges(const char * n, bool withMask);
		// Specify threshold
		IPLSobelEdges(const char * n, float threshold, bool withMask);

		virtual bool processInput();

		virtual bool checkInput() const;

		virtual const ImageProcessorOutput * getOutput() const {
			return &out;
		}
};




#endif // IPL_SOBEL_EDGES_H
