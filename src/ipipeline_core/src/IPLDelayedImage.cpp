#include "ipipeline_core/IPLDelayedImage.h"
#include "ros/ros.h"


IPLDelayedImage::IPLDelayedImage(const char * n, unsigned int delay) :
	IPLImageProcessor(n), memory(delay+1)
{
	deftInput = addInput("Default");
	top = 0;
}

IPLDelayedImage::~IPLDelayedImage() 
{
}

IPLDelayedImage::MemItem::MemItem() : gotfirst(false)
{
  image = std::vector<cv::Mat>(1);
}

void IPLDelayedImage::MemItem::reallocate( int type, 
		unsigned int w, unsigned int h, unsigned int nimg)
{
    image.resize(nimg);
    for (unsigned int i=0;i<nimg;i++) {
        image[i].create(cv::Size(w,h),type);
	gotfirst = false;
    }
}


ImageProcessorOutput IPLDelayedImage::getOutput() const
{
	if (!memory[top].gotfirst) return memory[0].image;
    	return memory[top].image;
}

bool IPLDelayedImage::checkInput() const 
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
    if (in.size() == 0) return false;
	return true;
}

bool IPLDelayedImage::processInput()
{
  IPLImageProcessor * input = getInput(deftInput);
  ImageProcessorOutput in = input->getOutput();
  //~ ROS_INFO("in IPLDelayImage::processInput()");
  for (unsigned int i = 0; i < in.size(); i++) {
    in[i].copyTo(memory[top].image[i]);
    //~ memory[top].image[i] = in[i].clone();
  } 
  
  //~ memory[top].image = in;
  memory[top].gotfirst = true;
  top = (top + 1) % memory.size();
  return true;
}
