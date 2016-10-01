#include <algorithm>
#include "ipipeline_core/IPLIPScheduler.h"
//#define VERBOSE
//#define TICKS
//#define VDEBUG


using namespace std;

IPLIPScheduler::IPLIPScheduler()
{
	terminating = false;
	probeThreadId = 0;
	probeThreadFlag = false;

	profiling = false;
	profiling_total = 0.0;
	profiling_nloop = 0;
}

IPLIPScheduler::~IPLIPScheduler()
{
	terminating = true;
	if (probeThreadFlag) {
		stopProbeThread(true);
	}
	clear();
}
	
void IPLIPScheduler::clear()
{
	OpStore::reverse_iterator i = opstore.rbegin();
	while (i != opstore.rend()) {
		IPLImageProcessor * ip = *i;
		assert (ip != NULL);
#ifdef VDEBUG
		printf("Deleting %s\n",ip->getName());
#endif
		delete ip;
		i ++;
	}
	opstore.clear();

	PopStore::reverse_iterator j = probes.rbegin();
	while (j != probes.rend()) {
		IPLGenericProbe * ip = *j;
		if (ip != NULL)
			delete ip;
		j ++;
	}
	probes.clear();

}




void IPLIPScheduler::store(IPLImageProcessor * ip, bool source)
{
	pair<OpStore::iterator,bool> res = opstore.insert(ip);
	if (!res.second) {
		fprintf(stderr,"Image processor already in the Op. Store!\n");
		assert(res.second);
	}
    if (source) {
        sources.insert(ip);
	}
	ip->registerProfilingFlag(&profiling);
}

void IPLIPScheduler::storeProbe(IPLGenericProbe * ip)
{
	pair<PopStore::iterator,bool> res = probes.insert(ip);
	if (!res.second) {
		fprintf(stderr,"Probe already in the Probe Store!\n");
		assert(res.second);
	}
	ip->setStore(&opstore);
}

void IPLIPScheduler::updateProbes()
{
	PopStore::iterator i = probes.begin();
	while (i != probes.end()) {
		IPLGenericProbe * p = *i;
		p->updateProbe();
		i ++;
	}
}


void* updateProbeThreadFunction(void *arg)
{
	IPLIPScheduler* that = (IPLIPScheduler*)arg;
	while (that->probeThreadFlag) {
		that->updateProbes();
		usleep(100000);
	}
	return NULL;
}



bool IPLIPScheduler::startProbeThread()
{
	if (probeThreadFlag) {
		return false;
	}
	probeThreadFlag = true;
	pthread_create(&probeThreadId,NULL,updateProbeThreadFunction,this);
	return true;
}

bool IPLIPScheduler::stopProbeThread(bool force)
{
	if (!probeThreadFlag) { 
		return true;
	}
	probeThreadFlag = false;
	pthread_cancel(probeThreadId);
	pthread_join(probeThreadId,NULL);
	probeThreadId=0;
	return true;
}



bool IPLIPScheduler::checkProcessingTree()
{
	//printf("%d module in the processing tree, %d root\n",
	//		opstore.size(),sources.size());
	OpStore::iterator i = sources.begin();
	while (i != sources.end()) {
		IPLImageProcessor * ip = *i;
		//printf("Checking Processing Tree from '%s'\n",ip->getName());
		if (!ip->checkProcessingTree()) 
			return false;
		//printf("Success\n");
		i ++;
	}
	return true;
}

void IPLIPScheduler::printVCG(const char * fname)
{
	// We don't want to print the probes
	OpStore opprobes;
	PopStore::iterator it;
	for (it=probes.begin();it!=probes.end();it++) {
		opprobes.insert(*it);
	}

	IPLImageProcessor::printVCG(fname,sources,opprobes);
}

void IPLIPScheduler::printDOT(const char * fname)
{
	// We don't want to print the probes
	OpStore opprobes;
	PopStore::iterator it;
	for (it=probes.begin();it!=probes.end();it++) {
		opprobes.insert(*it);
	}

	IPLImageProcessor::printDOT(fname,sources,opprobes);
}

struct ProfilingOrder 
{
	bool operator()(const IPLImageProcessor * ip1, 
			const IPLImageProcessor * ip2)
	{
		return ip1->getProfilingTime() > ip2->getProfilingTime();
	}
};

void IPLIPScheduler::saveProfiling(const char * fname) 
{
	/*** First sort the image processor modules ***/
	OpStore::iterator i = opstore.begin();
	vector<IPLImageProcessor*> profout;
	double used = 0.0;
	while (i != opstore.end()) {
		
		// we don't profile probes
		if (probes.find(dynamic_cast<IPLGenericProbe*>(*i))!=probes.end())
			continue;

		profout.push_back(*i);
		used += (*i)->getProfilingTime();
		++ i;
	}
	ProfilingOrder profiling_order;
    std::sort(profout.begin(),profout.end(),profiling_order);


	/*** Then write the result ***/
	FILE * fp = fopen(fname,"w");
	if (fp == NULL) {
		fprintf(stderr,"Cannot save profiling to %s\n",fname);
		return;
	}
	fprintf(fp,"%20s =\t%d\n","Iterations",profiling_nloop);
	fprintf(fp,"%20s =\t%f s\n","Total time",profiling_total);
	fprintf(fp,"%20s =\t%f%%\t%f s\n","Overhead",
			100.0*(profiling_total-used)/profiling_total,
			profiling_total-used);
	fprintf(fp,"%20s =\t%f s\t%.6f s/run\t%.1f Hz\n","Used time",used,used/profiling_nloop,profiling_nloop/used);
	unsigned int j;
	for (j=0;j<profout.size();j++) {
		IPLImageProcessor * ip = profout[j];
        if (ip->getProfilingRuns()) {
            fprintf(fp,"%20s =\t%f%%\t%f s\t%f s/run\n",ip->getName(),
                    100.0*ip->getProfilingTime()/profiling_total,
                    ip->getProfilingTime(),ip->getProfilingTime()/ip->getProfilingRuns());
        } else {
            fprintf(fp,"%20s :\tdisabled\n",ip->getName());
        }
	}
	fclose(fp);
	ROS_INFO("Profiling information saved to %s",fname);
}

void IPLIPScheduler::readConfig(Config * config)
{
#if 0
	if (config->selectSection("Scheduler")) {
	}
#endif
	OpStore::iterator i = opstore.begin();
	while (i != opstore.end()) {
		if (config->selectSection((*i)->getName())) {
			(*i)->readConfig(config);
		}
		i++;
	}
}




