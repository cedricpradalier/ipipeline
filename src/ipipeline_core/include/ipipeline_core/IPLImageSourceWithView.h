#ifndef IPL_IMAGE_SOURCE_WITH_VIEW_H
#define IPL_IMAGE_SOURCE_WITH_VIEW_H

#include "IPLImageSource.h"

/**
 * \class IPLImageSource : abstract class
 * Implementation of a generic image source. 
 * An image source has no input
 * output is a set of ImageProcessorOutput
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageSourceWithView : public IPLImageSource
{
	protected :
		bool firstOutput;
		unsigned int x, y;
        std::vector<cv::Mat> view;
        std::vector<cv::Rect> roi;

		void allocateView(unsigned int w, unsigned int h);

	public :
		/**
		 * \arg t : type of the output
		 * \arg n : number of output images, 
		 * \arg w,h : image size
		 * **/
		IPLImageSourceWithView(const char * name, 
				IPLImageProcessorRole role, int type, unsigned int n=1,
				unsigned int w=0, unsigned int h=0,
				unsigned int vw=0, unsigned int vh=0);
		virtual ~IPLImageSourceWithView();

		void setView(cv::Rect rect, unsigned int index = 0); 
        cv::Rect getView(unsigned int index = 0) const {
            assert(index < roi.size());
            return roi[index];
        }

		virtual bool processInput();
		virtual ImageProcessorOutput getOutput() const {
			return view;
		}
};




#endif // IPL_IMAGE_SOURCE_WITH_VIEW_H
