#ifndef IPL_DRAW_CROSS_H
#define IPL_DRAW_CROSS_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLLocalMax : \see IPLImageProcessor
 * Draw crosses in the image
 * input "Image" is a set of RGB.
 * input "Coord" is a set of 1-Channel nx2 floating point image representing
 * the cross size
 * output is as many images, with crosses drawn at the coordinate
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLDrawCross : public IPLImageProcessor
{
	protected :
        int imageInput;
        int coordInput;
        cv::Scalar color;
        unsigned int scale;
        unsigned int thickness;
	public :
		/**
         * See OpenCV threshold function
		 * **/
		IPLDrawCross(const char * n, cv::Scalar col, unsigned int sz, unsigned int thick=1);

		virtual bool processInput();

		virtual bool checkInput() const;

};




#endif // IPL_DRAW_CROSS_H
