#ifndef IPL_IP_SCHEDULER_MT_H
#define IPL_IP_SCHEDULER_MT_H

#include <map>
#include <list>
#include <pthread.h>
#include <semaphore.h>

#include "IPLIPScheduler.h"

/**
 * \class IPLIPSchedulerMT
 * This class deals with the storage of ImageProcessor object and their
 * execution, in a consistent order, possibly on several processors.
 * **/
class IPLIPSchedulerMT : public IPLIPScheduler
{
	public :

		/**
		 * Constructor
		 * \arg num_threads: how many threads we want to use
		 * **/
		IPLIPSchedulerMT(unsigned int num_threads);
		~IPLIPSchedulerMT();

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

	protected :
		/** The scheduling queue **/
		SchedQueue queue;

		/** 
		 * Some mutexes to deal with concurrent access to the object resources
		 * **/
		pthread_mutex_t queue_mutex;
		pthread_mutex_t thread_term_mutex;
		std::list<pthread_mutex_t*> availableThread;

		/** list of thread_id to be sure to kill all threads on exit **/
		std::vector<pthread_t> threads;
		/** number of currently running threads **/
		unsigned int running;
		/** number of sources to push in the queue **/
		unsigned int sourcesToPush;

		/** The processing thread function **/
		friend void * ProcThread(void *);
		/** The probe management thread function **/
		friend void * ProbeThread(void *);
		/** 
		 * process next image processor (lock mutexes, popqueue, ...) 
		 * called by ProcThread
		 * **/
		virtual void processNextIP();

};

#endif // IPL_IP_SCHEDULER_MT_H
