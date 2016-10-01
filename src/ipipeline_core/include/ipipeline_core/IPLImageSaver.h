#ifndef IPL_IMAGE_SAVER_H
#define IPL_IMAGE_SAVER_H

#include "IPLImageProcessor.h"

/**
 * \class IPLImageSaver : \see IPLImageSaver
 * Implementation of a input saver 
 * An image saver has no output, but saves its input (the first image if 
 * multiples) into a PGM (for Int8u) or TGA (for RGB8u or RGBA8u) file
 * input is a set of IppiImage
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLImageSaver : public IPLImageProcessor
{
	protected:
		unsigned int deftInput;
		bool first, autoincr;
		unsigned int frameno;

		bool pattern_output;
        std::string output;
        std::string im_name;
	public :
		/**
		 * Constructor: outputname is the name of the file that
		 * the ImageProcessor will create. If ispattern is true,
		 * this name, will be expected to have (at most) one %d
		 * inside, to insert the frame number. sprintf will be used
		 * **/
		IPLImageSaver(const char * n, 
				const std::string & outputname,
				bool ispattern=false,
				bool autoincr=false);
		virtual ~IPLImageSaver();

		/** Save its input **/
		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_IPPI_IMAGE_SAVER_H
