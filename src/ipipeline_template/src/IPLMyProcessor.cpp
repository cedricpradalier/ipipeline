
#include "ipipeline_template/IPLMyProcessor.h"

IPLMyProcessor::IPLMyProcessor(const char * name) : 
	IPLImageFilter(name)
{
    // Nothing to initialize here, everything is inherited from the image
    // filter.
}

void IPLMyProcessor::readConfig(Config *config)
{
	config->getDouble("local_variable",&local_variable);
}


bool IPLMyProcessor::checkInput() const 
{
    // First get the image processor that produced our input (deftInput is the
    // default input defined by ImageFilter).
    IPLImageProcessor * input = getInput(deftInput);
    // Request the output of our input processor, which will be our input data.
    // ImageProcessorOutput is an array of cv::Mat
    ImageProcessorOutput in = input->getOutput();
    // Now check that all our inputs have the correct type (8bit gray here).
    for (unsigned int i=0;i<in.size();i++) {
        if (in[i].type() != CV_8UC1) {
            return error(INVALID_TYPE);
        }
    }
    // OK, everything is consistent
	return true;
}




bool IPLMyProcessor::processInput()
{
    // First get the image processor that produced our input (deftInput is the
    // default input defined by ImageFilter).
	IPLImageProcessor * input = getInput(deftInput);
    // Request the output of our input processor, which will be our input data.
    // ImageProcessorOutput is an array of cv::Mat
    ImageProcessorOutput in = input->getOutput();
    // Make sure our output has the same number of cv::Mat as our input
    outputVector.resize(in.size());
    // Now iterate on all the cv::Mat in our input
    for (unsigned int i=0;i<in.size();i++) {
        // Enforce output image sizes
        outputVector[i].create(in[i].size(),CV_16SC1);
        // Initialize to the correct value
        outputVector[i] = cv::Scalar(0);
        // And that's all for now... Add your own code below
    }

    // And we're done. Return false if the processing tree needs to be aborted.
	return true;
}

