#include "ipipeline_core/IPLDemux.h"

IPLDemux::IPLDemux(const char * n, unsigned int _which) : 
	IPLImageProcessor(n), which(_which), rwhich(which) 
{
	deftInput = addInput("Default");
    outputVector.resize(1);
}

IPLDemux::IPLDemux(const char * n, unsigned int * _which) : 
	IPLImageProcessor(n), which(0), rwhich(*_which) 
{
	deftInput = addInput("Default");
    outputVector.resize(1);
}

bool IPLDemux::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in= input->getOutput();
    outputVector[0] = in[rwhich];
	return true;
}

bool IPLDemux::checkInput() const 
{
	IPLImageProcessor * input = getInput(deftInput);
    ImageProcessorOutput in = input->getOutput();
	if (rwhich >= in.size()) {
		return error("index %d > available inputs %d\n",
				rwhich,in.size()-1);
	}
	return true;
}

