
#include "ipipeline_core/IPLBufferWriter.h"

IPLMonoBufferWriter::IPLMonoBufferWriter(const char * n, 
		IPLImageProcessor * in, 
		void *  buffer, unsigned int bufSize,
		char * iname, unsigned int inamesize,
		unsigned int * widthp, unsigned int * heightp, 
		unsigned int index, 
		SignalFunction fdone,void * arg) :
	IPLImageProcessor(n,in), 
	bufferDest(buffer), bufferSize(bufSize),
	ipname(iname), ipnameSize(inamesize),
	widthDest(widthp), heightDest(heightp),
	inputIndex(index), signalF(fdone), signalArg(arg)
{
	inputid = addInput("Default");
	assert((bufferDest != NULL) && (bufferSize > 0));
}


bool IPLMonoBufferWriter::checkInput() const 
{
	IPLImageProcessor * input = getInput(inputid);
	ImageProcessorOutput in = input->getOutput();
    if (in.size() == 0) return false;
	return true;
}

bool IPLMonoBufferWriter::processInput()
{
	IPLImageProcessor * input = getInput(inputid);
	ImageProcessorOutput in = input->getOutput();
    assert(inputIndex < in.size());

	if (ipname != NULL) {
		strncpy(ipname,input->getName(),ipnameSize);
		ipname[ipnameSize-1] = 0;
	}

    const cv::Mat & m = in[inputIndex];
    if (m.rows * m.step > bufferSize) {
        fprintf(stderr,"MonoWriter: buffer size too small\n");
        return false;
    }

    memcpy(bufferDest,m.data,m.rows*m.step);
    if (widthDest!=NULL) *widthDest=m.cols;
    if (heightDest!=NULL) *heightDest=m.rows;
	
	if (signalF != NULL) {
		signalF(signalArg);
	}
	return true;
}


