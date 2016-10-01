#include <cstdarg>
#include "ipipeline_core/IPLBarrier.h"

IPLBarrier::IPLBarrier(const char * n,IPLImageProcessor * in, ...) : 
	IPLImageProcessor(n,in)
{
	va_list vl;
	deftInput = addInput("Default");
	in->addReceiver(this,"Default");
	input = in;
	va_start(vl,in);
	unsigned int i=0;
	while (vl != NULL) {
		char tmp[10];
		sprintf(tmp,"aux%03d",i++);
		addInput(tmp);
		IPLImageProcessor* aux=va_arg(vl,IPLImageProcessor*);
		aux->addReceiver(this,tmp);
	}
	va_end(vl);
}


bool IPLBarrier::processInput()
{
	IPLImageProcessor * in = getInput(deftInput);
	input = in;
	return true;
}

bool IPLBarrier::checkInput() const 
{
	return true;
}

ImageProcessorOutput IPLBarrier::getOutput() const
{
	return input->getOutput();
}

