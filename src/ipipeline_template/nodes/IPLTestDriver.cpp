
#include <assert.h>

#include "ipipeline_io/IPLRosImageTransportPublisher.h"
#include "ipipeline_io/IPLRosImageTransportSubscriber.h"
#include "ipipeline_core/IPLCvImageSource.h"
#include "ipipeline_modules/IPLImageToGray.h"
#include "ipipeline_template/IPLMyProcessor.h"
#include "IPLTestDriver.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Constructor has to specify the prefix that will be used for the profiling
// and graph files.
IPLTestDriver::IPLTestDriver() : IPLDriver("IPLTest")
{
    use_source_image = true;
    debug = false;
}

IPLTestDriver::~IPLTestDriver()
{
}


bool IPLTestDriver::initialise() 
{
    ROS_INFO("In '%s::initialise",this->getName().c_str());
    if (config.selectSection("main")) {
        config.getBool("debug",&debug);
    }
    if (config.selectSection(this->getName())) {
        config.getBool("use_source_image",&use_source_image);
        config.getString("infile",infile);
        config.getString("intopic",intopic);
        config.getString("outtopic",outtopic);
        ROS_INFO("Test driver: reading from '%s' writing to '%s'",use_source_image?infile.c_str():intopic.c_str(),outtopic.c_str());
    } else {
        ROS_WARN("Could not select section '%s'",this->getName().c_str());
    }
    return true;
}

bool IPLTestDriver::buildProcessingTree()
{
    // For each image processor in the tree, we do two or three things.
    // - We create an instance of the image processor object (new)
    // - If needed we add the instance as a receiver for the object creating its input
    // - We store the pointer in the IPLDriver. After this point, the pointer
    // is owned by the IPLDriver and will be deleted automatically.
    
    // First create an image source, either from ROS or from a file
    IPLImageProcessor * image = NULL;
    if (use_source_image) {
        image = new IPLCvImageSource("image source", infile);
    } else {
        image = new IPLRosImageTransportSubscriber("image source", nh, intopic, "raw");
    }
    store(image);

    // Use a standard module to convert it to 8b gray (path through if it is
    // already gray)
	IPLImageToGray * gray = new IPLImageToGray("Gray");
	image->addReceiver(gray);
	store(gray);

    // Instantiate our customize image processor
	IPLMyProcessor * my_proc = new IPLMyProcessor("my processor");
	gray->addReceiver(my_proc);
	store(my_proc);

    // Instantiate an image sink that will export the result to ROS
	IPLRosImageTransportPublisher * publisher = new IPLRosImageTransportPublisher("image sink",nh,outtopic);
	my_proc->addReceiver(publisher);
	store(publisher);

	return true;
}




