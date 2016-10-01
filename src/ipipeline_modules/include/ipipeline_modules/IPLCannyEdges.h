#ifndef IPL_CANNY_EDGES_H
#define IPL_CANNY_EDGES_H

#include <math.h>
#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLCannyEdges : \see IPLImageFilter
 * Find edges pixels from an image (Only CV_8uC1)
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLCannyEdges : public IPLImageFilter
{
	protected:

		double lowThres, highThres;
	public :
		IPLCannyEdges(const char * n, double thresLow, double thresHigh);
		virtual ~IPLCannyEdges();

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_CANNY_EDGES_H
