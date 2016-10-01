#include "ipipeline_core/IPLIPSchedulerMT.h"
//#define VERBOSE
//#define TICKS
//#define VDEBUG


using namespace std;

void * ProcThread(void * ipsched)
{
	IPLIPSchedulerMT * scheduler = (IPLIPSchedulerMT*)ipsched;
	while (!scheduler->isTerminating()) {
		scheduler->processNextIP();
	}
	return NULL;
}

void * ProbeThread(void * ipsched)
{
	IPLIPSchedulerMT * scheduler = (IPLIPSchedulerMT*)ipsched;
	while (!scheduler->isTerminating()) {
		scheduler->updateProbes();
		usleep(200000);
	}
	return NULL;
}

IPLIPSchedulerMT::IPLIPSchedulerMT(unsigned int num_threads) :
	IPLIPScheduler()
{
	pthread_t id;
	unsigned int i;
	running = 0;
	sourcesToPush = 0;
	pthread_mutex_init(&queue_mutex,NULL);
	pthread_mutex_init(&thread_term_mutex,NULL);
	
	pthread_mutex_lock(&thread_term_mutex);
	pthread_create(&id,NULL,ProbeThread,this);
	threads.push_back(id);

	for (i=0;i<num_threads;i++) {
		pthread_create(&id,NULL,ProcThread,this);
		threads.push_back(id);
	}
}

IPLIPSchedulerMT::~IPLIPSchedulerMT()
{
	unsigned int j;
	for (j=0;j<threads.size();j++) {
		pthread_cancel(threads[j]);
	}
}
	

void IPLIPSchedulerMT::processNextIP()
{
	pthread_mutex_t lmutex = PTHREAD_MUTEX_INITIALIZER;
#ifdef TICKS
	printf("!");fflush(stdout);
#endif
	pthread_mutex_lock(&queue_mutex);
	if (terminating) return;
	if (queue.empty()) {
#ifdef VDEBUG
		printf("Th %d queue empty, Waiting\n",(int)pthread_self());
#endif
#ifdef TICKS
		printf("+");fflush(stdout);
#endif
		pthread_mutex_lock(&lmutex);
		availableThread.push_back(&lmutex);
#ifdef timeofday
		printf("_");fflush(stdout);
#endif
		pthread_mutex_unlock(&queue_mutex);
	}
	// we can pass that only when there is data in the queue
	pthread_mutex_lock(&lmutex);
	if (terminating) return;
	pthread_mutex_unlock(&lmutex);
	assert (!queue.empty());
	
	SchedItem it = queue.front();
	queue.pop_front();
#ifdef TICKS
	printf("_");fflush(stdout);
#endif
	bool itemReady = registerInput(it.ip,it.input,it.id);
	
	running += 1;
	pthread_mutex_unlock(&queue_mutex);

	if (itemReady) {
#ifdef VDEBUG
		printf("Th %d processing %s\n",(int)pthread_self(), it.ip->getName());
#endif
		testAndProcessInput(it.ip,this);
	}
		
	
	
#ifdef TICKS
	printf("!");fflush(stdout);
#endif
	pthread_mutex_lock(&queue_mutex);
	if (terminating) return;
	running -= 1;
#ifdef VERBOSE
	printf("Th %d locked queue_mutex, %d IP in the queue, %d running, %d to be pushed\n",(int)pthread_self(),queue.size(),running,sourcesToPush);
#endif
	if ((sourcesToPush==0) && (running == 0) && (queue.empty())) {
#ifdef VERBOSE
		printf("Th %d unlocking term\n",(int)pthread_self());
#endif
		pthread_mutex_unlock(&thread_term_mutex);
	}
#ifdef TICKS
	printf("_");fflush(stdout);
#endif
	pthread_mutex_unlock(&queue_mutex);
}
	

void IPLIPSchedulerMT::push(IPLImageProcessor * torun, 
		IPLImageProcessor * input, unsigned int inputid)
{
#ifdef TICKS
	printf("!");fflush(stdout);
#endif
	pthread_mutex_lock(&queue_mutex);
	if (input == NULL)
		sourcesToPush -= 1;
	queue.push_front(SchedItem(torun,input,inputid));
#ifdef VDEBUG
	printf("Pushed : %s -> %s (%d src waiting)\n",
			(input==NULL)?"Source":input->getName(),
			torun->getName(),sourcesToPush);
#endif
	if (availableThread.empty()) {
#ifdef TICKS
		printf("_");fflush(stdout);
#endif
		pthread_mutex_unlock(&queue_mutex);
	} else {
		pthread_mutex_t * mtx = availableThread.front();
		availableThread.pop_front();
#ifdef TICKS
		printf("o");fflush(stdout);
#endif
		pthread_mutex_unlock(mtx);
		// we don't unlock the queue since we want 
		// to be sure that only the unlocked thread
		// can text this SchedItem
	}
}


bool IPLIPSchedulerMT::runProcTree()
{
	double t1,t2;
	t1 = IPLImageProcessor::timeofday();
#ifdef VDEBUG
	printf("\n\n----\n\n");fflush(stdout);
#endif
	OpStore::iterator i;
	for (i=opstore.begin();i!=opstore.end();i++) {
		(*i)->checkState();
	}
	
	queue.clear();
	sourcesToPush = sources.size();
	i = sources.begin();
	while (i != sources.end()) {
		IPLImageProcessor * ip = *i;
		push(ip,NULL,0);
		i ++;
	}
#ifdef VDEBUG
	printf("All %d sources in the queue (%d)\n",
			sources.size(),sourcesToPush);fflush(stdout);
#endif
	// Now the thread are unleashed ...
	
	// This should only be unlocked after the last process terminated
	pthread_mutex_lock(&thread_term_mutex);
	//printf("thread_term_mutex has been unlocked!\n");

	/** TODO : return an informative value **/
	t2 = IPLImageProcessor::timeofday();
	profiling_total += (t2-t1);
	profiling_nloop += 1;
	return true;
}

