#include "ipipeline_core/IPLIPScheduler1Tstatic.h"
//#define VERBOSE
//#define TICKS
//#define VDEBUG


using namespace std;

IPLIPScheduler1Tstatic::IPLIPScheduler1Tstatic() : IPLIPScheduler()
{
	lastProbeTime = -1;
	firstRun = true;
}

IPLIPScheduler1Tstatic::~IPLIPScheduler1Tstatic()
{
}
	

void IPLIPScheduler1Tstatic::processNextIP()
{
	if (terminating) return;
	if (queue.empty()) return;
	
	SchedItem it = queue.front();
	queue.pop_front();
	bool itemReady = registerInput(it.ip,it.input,it.id);
	
	if (itemReady) {
		orderedIP.push_back(it.ip);
		testAndProcessInput(it.ip,this);
	}
}
	

void IPLIPScheduler1Tstatic::push(IPLImageProcessor * torun, 
		IPLImageProcessor * input, unsigned int inputid)
{
	if (firstRun) {
		queue.push_front(SchedItem(torun,input,inputid));
	} else {
		registerInput(torun, input, inputid);
	}
}

bool IPLIPScheduler1Tstatic::runProcTree()
{
	double t1,t2;
	t1 = IPLImageProcessor::timeofday();
	OpStore::iterator i;
	for (i=opstore.begin();i!=opstore.end();i++) {
		(*i)->checkState();
	}
	
	if (firstRun) {
		orderedIP.clear();
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

		firstRun = false;
#if 0
		printf("Run order:\n");
		for (unsigned i=0;i<orderedIP.size();i++) {
			printf("%d %s\n",i,orderedIP[i]->getName());
		}
#endif
	} else {
		std::vector<IPLImageProcessor *>::iterator it;
		for (it=orderedIP.begin();it!=orderedIP.end();it++) {
			testAndProcessInput(*it,this);
		}
	}

	/** TODO : return an informative value **/
	t2 = IPLImageProcessor::timeofday();
	profiling_total += (t2-t1);
	profiling_nloop += 1;

	if ((t2 - lastProbeTime) > 0.2) {
		updateProbes();
		lastProbeTime = t2;
	}

	return true;
}

