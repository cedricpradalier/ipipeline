
#include <assert.h>

#include "ipipeline_core/IPLCvImageSource.h"
#include "ipipeline_core/IPLImageSaver.h"
#include "ipipeline_modules/IPLDepthConvert.h"
#include "ipipeline_modules/IPLImageToGray.h"
#include "ipipeline_modules/IPLImageBlurrer.h"
#include "ipipeline_modules/IPLImageDiff.h"
#include "IPLTestDriver.h"

IPLTestDriver::IPLTestDriver() 
    : IPLDriver("IPLTest")
{
    activateBlur = true;
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
        config.getBool("blur",&activateBlur);
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
	blur->setCondition("blurAbsError",activateBlur);
	gray->addReceiver(blur);
	store(blur);

	IPLImageDiff * diff = new IPLImageDiff("Diff");
	gray->addReceiver(diff,"InputB");
	blur->addReceiver(diff,"InputA");
	store(diff);

	IPLImageSaver * saver = new IPLImageSaver("image sink",outfile);
	diff->addReceiver(saver);
	store(saver);

	return true;
}




