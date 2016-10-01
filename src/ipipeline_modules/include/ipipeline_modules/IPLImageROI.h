#ifndef IPL_IMAGE_ROI_H
#define IPL_IMAGE_ROI_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLImageROI : \see IPLImageFilter
 * Returns a selected region of an image.
 * input "Default" must output a set of Int8u or RGB8u image 
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageROI : public IPLImageFilter
{
	protected:
        cv::Rect ROI;
	public :
		IPLImageROI(const char * n, unsigned int x, unsigned int y, unsigned int w, unsigned int h);
		IPLImageROI(const char * n, const cv::Rect & rect);

		void setROI(unsigned int x, unsigned int y, unsigned int w, unsigned int h);
		void setROI(const cv::Rect & rect);

		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IMAGE_ROI_H
