#ifndef IPL_IP_SCHEDULER_H
#define IPL_IP_SCHEDULER_H

#include <map>
#include <list>

#include "IPLImageProcessor.h"
#include "IPLGenericProbe.h"

/**
 * \class IPLIPScheduler
 * This class deals with the storage of ImageProcessor object and their
 * execution, in a consistent order, possibly on several processors.
 * **/
class IPLIPScheduler
{
	public :

		/**
		 * Constructor
		 * \arg num_threads: how many threads we want to use
		 * **/
		IPLIPScheduler();
		virtual ~IPLIPScheduler();

		/**
		 * Clear the list of processing object
		 * **/
		void clear();

		/**
		 * Store an ImageProcessor, and if source is true, 
		 * register it as a source one, that one that is executed initially
		 * without having  to wait for input
		 * **/
		void store(IPLImageProcessor * ip, bool source = false);

		/**
		 * Store a IPLProbe 
		 * **/
		void storeProbe(IPLGenericProbe * ip);

		/**
		 * Function to start a thread doing only the probe updating
		 * Useful in step by step mode
		 **/
		bool startProbeThread();
		bool stopProbeThread(bool force=false);

		/** 
		 * Run the processing tree, starting by image processors registered as
		 * sources 
		 * **/
		virtual bool runProcTree() = 0;
		/**
		 * Schedule the execution of \arg torun with \arg input as input on
		 * entry \arg inputid. If torun is not ready to run, because it still
		 * miss some inputs, then it will return and be rescheduled for later
		 * **/
		virtual void push(IPLImageProcessor * torun, IPLImageProcessor * input,
				unsigned int inputid) = 0;

		/** 
		 * Check that the processing tree has been correctly connected
		 * **/
		bool checkProcessingTree();
		/**
		 * Print a VCG representation of the processing tree
		 * **/
		void printVCG(const char * fname);
		/**
		 * Print a DOT representation of the processing tree
		 * **/
		void printDOT(const char * fname);

		/** Activate profiling (or disactivate it) **/
		void setProfiling(bool newprof=true) {profiling=newprof;}
		/** Save profiling results in fname **/
		void saveProfiling(const char * fname);

		/** Is the scheduler terminating **/
		bool isTerminating() {return terminating;}


		void readConfig(Config * config);


	protected :
		typedef std::set<IPLGenericProbe*, OpStore_LtPtr> PopStore;
		/** The set of all registered probes **/
		PopStore probes;

	protected :
		/** Schedule object to put in the Scheduling Queue **/
		struct SchedItem
		{
			IPLImageProcessor * ip;
			IPLImageProcessor * input;
			unsigned int id;
			SchedItem(IPLImageProcessor * _ip, IPLImageProcessor * _in, unsigned int i) :
				ip(_ip), input(_in), id(i) {}
			SchedItem(IPLImageProcessor * source) :
				ip(source), input(NULL), id(0) {}
		};
		typedef std::list<SchedItem> SchedQueue;

		/** The scheduling queue **/
		SchedQueue queue;


	protected :
		/** The set of all registered image processor **/
		OpStore opstore;
		/** The set of image processor registered as source (object in this set
		 * are also in opstore) **/
		OpStore sources;
		
		/** are we trying to terminate **/
		bool terminating;

		/** 
		 * process next image processor (lock mutexes, popqueue, ...) 
		 * called by ProcThread
		 * **/
		virtual void processNextIP() = 0;

		/** proxy function for inherited classed **/
		bool registerInput(IPLImageProcessor* module, IPLImageProcessor* input, unsigned int id) {
			return module->registerInput(input,id);
		}
		void testAndProcessInput(IPLImageProcessor* module, IPLIPScheduler *sched) {
			module->testAndProcessInput(sched);
		}


		bool probeThreadFlag;
		pthread_t probeThreadId;
		friend void* updateProbeThreadFunction(void*arg);
		/** 
		 * manage all probes (change image processor, reset...)
		 * called by ProbeThread
		 * **/
		void updateProbes();

		/** are we profiling **/
		bool profiling;
		/** time spent in the processNextIP function **/
		double profiling_total;
		/** number of time the processNextIP function has been called **/
		unsigned int profiling_nloop;

};

#endif // IPL_IP_SCHEDULER_H
