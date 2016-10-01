#ifndef IPL_MATRIX_SAVER_H
#define IPL_MATRIX_SAVER_H

#include "ipipeline_core/IPLImageProcessor.h"

/**
 * \class IPLMatrixSaver : \see IPLImageSaver
 * Implementation of a input saver 
 * An image saver has no output, but saves its input as matrix
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLMatrixSaver : public IPLImageProcessor
{
	protected:
		unsigned int deftInput;
		bool first;

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
		IPLMatrixSaver(const char * n, 
				const std::string & outputname, 
				bool ispattern=false);
		virtual ~IPLMatrixSaver();

		/** Save its input **/
		virtual bool processInput();

		virtual bool checkInput() const;
};




#endif // IPL_MATRIX_SAVER_H
