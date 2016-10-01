
#include <assert.h>

#include "ipipeline_io/IPLRosImageTransportPublisher.h"
#include "ipipeline_io/IPLRosImageTransportSubscriber.h"
#include "ipipeline_io/IPLMultiArrayRosPublisher.h"
#include "ipipeline_core/IPLCvImageSource.h"
#include "ipipeline_modules/IPLImageToGray.h"
#include "ipipeline_modules/IPLImageAdd.h"
#include "ipipeline_modules/IPLThresholder.h"
#include "ipipeline_modules/IPLMatrixSaver.h"
#include "ipipeline_modules/IPLDepthConvert.h"
#include "ipipeline_modules/IPLLocalMax.h"
#include "ipipeline_modules/IPLDrawCross.h"
#include "ipipeline_ssl/IPLSSLImage.h"
#include "ipipeline_ssl/IPLSSLScale.h"
#include "IPLTestDriver.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

IPLTestDriver::IPLTestDriver() 
    : IPLDriver("IPLTest")
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

	IPLSSLImage * ssl = new IPLSSLImage("ssl",IPLSSLImage::SSL_ALL);
	dconvert16s->addReceiver(ssl);
	store(ssl);

    IPLThresholder *thres = new IPLThresholder("ssl_thresh",5000,255,cv::THRESH_BINARY);
    ssl->addReceiver(thres);
    store(thres);

	IPLDepthConvert * thres8u = new IPLDepthConvert("dconvert 8u",CV_8UC1);
	thres->addReceiver(thres8u);
	store(thres8u);

	IPLLocalMax * lmax = new IPLLocalMax("local max");
	ssl->addReceiver(lmax,"Image");
	thres8u->addReceiver(lmax,"Mask");
	store(lmax);

	IPLDepthConvert * dconvertRGB = new IPLDepthConvert("dconvert rgb",CV_8UC3);
	image->addReceiver(dconvertRGB);
    dconvertRGB->setCondition("only debug",debug);
	store(dconvertRGB);

    IPLDrawCross * drawCross = new IPLDrawCross("draw cross",cv::Scalar(0,0,255),20,1);
    drawCross->setCondition("only debug",debug);
    dconvertRGB->addReceiver(drawCross,"Image");
    lmax->addReceiver(drawCross,"Coord");
    store(drawCross);

	IPLSSLScale * scale = new IPLSSLScale("scale",IPLSSLImage::SSL_ALL);
	dconvert16s->addReceiver(scale,"Image");
	lmax->addReceiver(scale,"SSL");
	store(scale);

    IPLDrawCross * drawCrossS = new IPLDrawCross("draw scale",CV_RGB(0,0,255),20,1);
    drawCrossS->setCondition("only debug",debug);
    dconvertRGB->addReceiver(drawCrossS,"Image");
    scale->addReceiver(drawCrossS,"Coord");
    store(drawCrossS);

	IPLMultiArrayRosPublisher * publisher = new IPLMultiArrayRosPublisher("ssl sink",nh,outtopic);
	scale->addReceiver(publisher);
	store(publisher);

#if 0
	IPLMatrixSaver * matrix = new IPLMatrixSaver("sslmat","sslmat");
	ssl->addReceiver(matrix);
	store(matrix);
#endif

#if 0
	IPLRosImageTransportPublisher * publisher = new IPLRosImageTransportPublisher("image sink",nh,outtopic);
	ssl->addReceiver(publisher);
	store(publisher);
#endif

	return true;
}




