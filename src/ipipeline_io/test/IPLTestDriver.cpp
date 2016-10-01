 
#include <assert.h>

#include "ipipeline_core/IPLCvImageSource.h"
#include "ipipeline_io/IPLRosImageTransportPublisher.h"
#include "ipipeline_io/IPLRosImageTransportSubscriber.h"
#include "ipipeline_modules/IPLDepthConvert.h"
#include "ipipeline_modules/IPLImageToGray.h"
#include "ipipeline_modules/IPLImageBlurrer.h"
#include "ipipeline_modules/IPLImageResizer.h"
#include "ipipeline_modules/IPLImageDiff.h"
#include "IPLTestDriver.h"

IPLTestDriver::IPLTestDriver() 
    : IPLDriver("IPLTest")
{
    use_source_image = true;
}

IPLTestDriver::~IPLTestDriver()
{
}


bool IPLTestDriver::initialise() 
{
    ROS_INFO("In '%s::initialise",this->getName().c_str());
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
    IPLImageProcessor * image = NULL;
    if (use_source_image) {
        image = new IPLCvImageSource("image source", infile);
    } else {
        image = new IPLRosImageTransportSubscriber("image source", nh, intopic, "raw");
    }
    store(image);

	IPLImageToGray * gray = new IPLImageToGray("Gray");
	image->addReceiver(gray);
	store(gray);

	IPLDepthConvert * dconvert16s = new IPLDepthConvert("dconvert 16s",CV_16SC1);
	gray->addReceiver(dconvert16s);
	store(dconvert16s);

	IPLDepthConvert * dconvert16u = new IPLDepthConvert("dconvert 16u",CV_16UC1);
	gray->addReceiver(dconvert16u);
	store(dconvert16u);

	IPLImageBlurrer * blur = new IPLImageBlurrer("Blur");
	// blur->setCondition("blurAbsError",activateBlur);
	gray->addReceiver(blur);
	store(blur);

	IPLImageResizer * resize1 = new IPLImageResizer("Resize1",0.5,0.5);
	gray->addReceiver(resize1);
	store(resize1);


	IPLImageDiff * diff = new IPLImageDiff("Diff");
	gray->addReceiver(diff,"InputB");
	blur->addReceiver(diff,"InputA");
	store(diff);

	IPLImageResizer * resize2 = new IPLImageResizer("Resize2",1.5,1.5);
	diff->addReceiver(resize2);
	store(resize2);


	IPLRosImageTransportPublisher * publisher = new IPLRosImageTransportPublisher("image sink",nh,outtopic);
	diff->addReceiver(publisher);
	store(publisher);

	return true;
}




