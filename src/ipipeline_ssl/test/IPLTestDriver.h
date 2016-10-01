#ifndef _IPL_TEST_DRIVER_H_
#define _IPL_TEST_DRIVER_H_

#include "ipipeline_core/IPLDriver.h"

/**
 * \class IPLTestDriver
 * Class implementing a basic dataflow test
 * \sa IPLImageProcessor \sa IPLIPScheduler 
 * \sa IPLParams \sa IPLSharedBuffers
 * **/
class IPLTestDriver : public IPLDriver
{
	public :
		/** 
		 * Constructor
		 * **/
		IPLTestDriver();
		~IPLTestDriver();

	protected :

		/**
		 * Build the complete processing tree
		 * **/
		virtual bool buildProcessingTree();

		virtual bool initialise();

        bool use_source_image;
		std::string infile;
		std::string intopic;
		std::string outtopic;
        ros::NodeHandle nh;
        bool debug;
};


#endif // _IPL_TEST_DRIVER_H_
