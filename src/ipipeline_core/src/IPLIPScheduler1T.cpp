#include "ipipeline_core/IPLIPScheduler1T.h"
//#define VERBOSE
//#define TICKS
//#define VDEBUG


using namespace std;

IPLIPScheduler1T::IPLIPScheduler1T() : IPLIPScheduler()
{
	lastProbeTime = -1;
}

IPLIPScheduler1T::~IPLIPScheduler1T()
{
}
	

void IPLIPScheduler1T::processNextIP()
{
	if (terminating) return;
	if (queue.empty()) return;
	
	SchedItem it = queue.front();
	queue.pop_front();
	bool itemReady = registerInput(it.ip,it.input,it.id);
	
	if (itemReady) {
		testAndProcessInput(it.ip,this);
	}
}
	

void IPLIPScheduler1T::push(IPLImageProcessor * torun, 
		IPLImageProcessor * input, unsigned int inputid)
{
	queue.push_front(SchedItem(torun,input,inputid));
}

bool IPLIPScheduler1T::runProcTree()
{
	double t1,t2;
	t1 = IPLImageProcessor::timeofday();
	OpStore::iterator i;
	for (i=opstore.begin();i!=opstore.end();i++) {
		(*i)->checkState();
	}

	queue.clear();
	i = sources.begin();
	while (i != sources.end()) {
		IPLImageProcessor * ip = *i;
		push(ip,NULL,0);
		i ++;
	}

	while (!queue.empty()) {
		processNextIP();
	}


	/** TODO : return an informative value **/
	t2 = IPLImageProcessor::timeofday();
	profiling_total += (t2-t1);
	profiling_nloop += 1;

#ifndef NDEBUG
	if ((t2 - lastProbeTime) > 0.2) {
		updateProbes();
		lastProbeTime = t2;
	}
#endif

	return true;
}

