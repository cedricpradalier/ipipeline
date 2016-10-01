#ifndef IPL_IP_SCHEDULER_1T_H
#define IPL_IP_SCHEDULER_1T_H

#include <map>
#include <list>

#include "IPLIPScheduler.h"

/**
 * \class IPLIPScheduler1T
 * This class deals with the storage of ImageProcessor object and their
 * execution, in a consistent order, possibly on several processors.
 * **/
class IPLIPScheduler1T : public IPLIPScheduler
{
	public :

		/**
		 * Constructor
		 * \arg num_threads: how many threads we want to use
		 * **/
		IPLIPScheduler1T();
		~IPLIPScheduler1T();

		/** 
		 * Run the processing tree, starting by image processors registered as
		 * sources 
		 * **/
		virtual bool runProcTree();
		/**
		 * Schedule the execution of \arg torun with \arg input as input on
		 * entry \arg inputid. If torun is not ready to run, because it still
		 * miss some inputs, then it will return and be rescheduled for later
		 * **/
		virtual void push(IPLImageProcessor * torun, IPLImageProcessor * input,
				unsigned int inputid);

	protected:
		double lastProbeTime;
		/** 
		 * process next image processor (lock mutexes, popqueue, ...) 
		 * called by ProcThread
		 * **/
		virtual void processNextIP();

};

#endif // IPL_IP_SCHEDULER_1T_H
