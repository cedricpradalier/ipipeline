
#include <assert.h>

#include "ipipeline_core/IPLCvImageSource.h"
#include "ipipeline_core/IPLImageSaver.h"
#include "IPLTestDriver.h"

IPLTestDriver::IPLTestDriver() 
    : IPLDriver("IPLTest")
{
}

IPLTestDriver::~IPLTestDriver()
{
}


bool IPLTestDriver::initialise() 
{
    ROS_INFO("In '%s::initialise",this->getName().c_str());
    if (config.selectSection(this->getName())) {
        config.getString("infile",infile);
        config.getString("outfile",outfile);
        ROS_INFO("Test driver: reading from '%s' writing to '%s'",infile.c_str(),outfile.c_str());
    } else {
        ROS_WARN("Could not select section '%s'",this->getName().c_str());
    }
    return true;
}

bool IPLTestDriver::buildProcessingTree()
{
	IPLImageProcessor * image = new IPLCvImageSource("image source", infile);
	store(image);

	IPLImageSaver * saver = new IPLImageSaver("image sink",outfile);
	image->addReceiver(saver);
	store(saver);

	return true;
}




