#ifndef IPL_OPT_FLOW_LK_H
#define IPL_OPT_FLOW_LK_H

#include "ipipeline_core/IPLBinaryOperator.h"
#include <opencv2/video/tracking.hpp>

/**
 * \class IPLImageDiff : \see IPLBinaryOperator
 * Apply a Diff operator on its two input. 
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLOptFlowLK : public IPLBinaryOperator
{
	public :
		IPLOptFlowLK(const char * n);

		virtual bool processInput();

		virtual bool checkInput() const;
};


#endif // IPL_OPT_FLOW_LK_H
  
