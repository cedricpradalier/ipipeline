#ifndef IPL_DELAYED_IMAGE_H
#define IPL_DELAYED_IMAGE_H

#include "IPLImageProcessor.h"

/**
 * \class IPLDelayedImage : 
 * Used to introduce a memory in the processing loop.
 * The longer the memory, the bigger the required memory allocation.
 * input "Default" must output a set of images
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLDelayedImage : public IPLImageProcessor
{
	protected :
		struct MemItem {
            std::vector<cv::Mat> image;

			MemItem();
			~MemItem() {}
			bool gotfirst;
			void reallocate(int type, 
					unsigned int w, unsigned int h, unsigned int nimg=1);
		};
		std::vector<MemItem> memory;
		unsigned int deftInput;
		unsigned int top;

	public :
		IPLDelayedImage(const char * n, unsigned int delay);

		~IPLDelayedImage();

		virtual bool checkInput() const ;

		virtual bool processInput();

    virtual ImageProcessorOutput getOutput() const;
};




#endif // IPL_DELAYED_IMAGE_H
