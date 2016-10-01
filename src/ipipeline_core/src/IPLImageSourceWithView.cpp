
#include "ipipeline_core/IPLImageSourceWithView.h"

//#define TRACE

IPLImageSourceWithView::IPLImageSourceWithView(const char * name, 
	    IPLImageProcessorRole role, int type, unsigned int n,
		unsigned int w, unsigned int h,
		unsigned int vw, unsigned int vh) :
	IPLImageSource(name,role,type,n,w,h)
{
	firstOutput = true;
	x = y = 0;
    view.resize(outputVector.size());
    roi.resize(outputVector.size());
    for (unsigned int i=0;i<outputVector.size();i++) {
        roi[i] = cv::Rect(0,0,w,h);
        view[i] = cv::Mat(outputVector[i],roi[i]);
    }
}

IPLImageSourceWithView::~IPLImageSourceWithView() 
{
}

void IPLImageSourceWithView::setView(cv::Rect rect, unsigned int index)
{
    assert(index < roi.size());
    cv::Rect all(0,0,outputVector[index].cols,outputVector[index].rows);
	assert(all.contains(rect.tl()));
	assert(all.contains(rect.br()));
    roi[index] = rect;
    view[index] = cv::Mat(outputVector[index],roi[index]);
}

bool IPLImageSourceWithView::processInput()
{
	return true;
}


