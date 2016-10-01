#ifndef IPL_ROI_FUSER_H
#define IPL_ROI_FUSER_H

#include <vector>

#include "ipipeline_core/IPLImageProcessor.h"

/**
 * \class IPLRoiFuser
 * Copy all inputs to the output image, using the input provided ROI
 * input "image%03d" must be an IppiImage, all of the same type
 * output is the same type of image as input
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLRoiFuser : public IPLImageProcessor
{
	protected :
		unsigned int ninput;
		std::vector<unsigned int> imageInputId;

		IppiImage* output;
	public :
		/**
		 * \arg ninput: number of expected input. Inputs will be created
		 * from this value
		 * **/
		IPLRoiFuser(const char * n, unsigned int ninput);
		~IPLRoiFuser();


		virtual bool processInput();

		virtual bool checkInput() const;

		const ImageProcessorOutput * getOutput()const { return output; }
};




#endif // IPL_ROI_FUSER_H
