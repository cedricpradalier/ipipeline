
#include "ipipeline_modules/IPLMatrixSaver.h"

IPLMatrixSaver::IPLMatrixSaver( const char * n, 
		const std::string & outputname, bool ispattern) :
	IPLImageProcessor(n)
{
	output = outputname;
	pattern_output = ispattern;
    im_name = output;
	deftInput = addInput("Default");
	first = true;
}

IPLMatrixSaver::~IPLMatrixSaver()
{
}

bool IPLMatrixSaver::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
	
    if (in.size() > 0) {
        for (unsigned int i = 0; i<in.size();i++) {
            if (pattern_output) {
                char tmp[output.size() + 32];
                sprintf(tmp,output.c_str(),frameid,i);
                im_name = tmp;
            }
            saveMatToFile(in[i],im_name);
            if (first) {
                if (pattern_output)
                    printf("Saver %s: saved image in %s (%s %d)\n",
                            name,im_name.c_str(),output.c_str(),frameid);
                else
                    printf("Saver %s: saved image in %s\n",name,im_name.c_str());
            }
            first = false;
        }
    }

	return true;
}

bool IPLMatrixSaver::checkInput() const
{
	return true;
}



