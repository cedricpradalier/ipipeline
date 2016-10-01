
#include <assert.h>
#include <errno.h>

#include "ipipeline_core/IPLDriver.h"
#include "ipipeline_core/IPLIPScheduler1T.h"
#include "ipipeline_core/IPLIPScheduler1Tstatic.h"
#include "ipipeline_core/IPLIPSchedulerMT.h"

IPLDriver::IPLDriver(const std::string & name) : triggerCallback(this)
{
	dname = name;
	waitingCoreInit = true;
	progressWheel = false;
	profiling = false;
	stepByStep = false;
	scheduler = NULL;
	counter = 0;
	pwheelPrefix = NULL;
    trigger_request_all = true;

    pthread_mutex_init(&triggerMutex,NULL);
    pthread_cond_init(&triggerCond,NULL);
    pthread_mutex_lock(&triggerMutex);
}

IPLDriver::~IPLDriver()
{
    pthread_mutex_unlock(&triggerMutex);
	if (scheduler != NULL) {
		char tmp[256];
		sprintf(tmp,"%s.prof",dname.c_str());
		scheduler->saveProfiling(tmp);
	}
	delete scheduler;
}

void IPLDriver::store(IPLImageProcessor * ip) 
{
    IPLImageProcessorRole role = ip->getRole();
	scheduler->store(ip,role & IPL_ROLE_SOURCE);
	if (role & IPL_ROLE_CACHING) {
		IPLCachingSource * src = dynamic_cast<IPLCachingSource*>(ip);
		if (src != NULL) cached.push_back(src);
	}
    if (role & IPL_ROLE_TRIGGER) {
        triggers.insert(std::pair<const IPLImageProcessor*,bool>(ip,false));
        ip->registerTriggerCallback(&triggerCallback);
    }
}

void IPLDriver::updateCachedSources()
{
	unsigned int i;
	for (i=0;i<cached.size();i++) {
		//printf("Activating caching for %s\n",cached[i]->getName());
		cached[i]->activateCaching();
		cached[i]->updateCache();
	}
}

bool IPLDriver::createScheduler(unsigned int numThreads,bool dynamicScheduler)
{
#ifndef NDEBUG
	ROS_DEBUG("%s: Launching scheduler\n",dname.c_str());
#endif
	if (scheduler != NULL) delete scheduler;
	if (numThreads == 1) {
		if (dynamicScheduler) {
			scheduler = new IPLIPScheduler1T();
		} else {
			scheduler = new IPLIPScheduler1Tstatic();
		}
	} else {
		scheduler = new IPLIPSchedulerMT(numThreads);
	}

	if (!buildProcessingTree())
		return false;

	if (!scheduler->checkProcessingTree()) {
		return false;
	}

#ifndef NDEBUG
	ROS_DEBUG("%s: Processing Tree Successfully built\n",dname.c_str());

#endif

	waitingCoreInit = false;
	scheduler->readConfig(&config);
	return true;
}




const char * IPLDriver::getProgressWheel(char *buffer, unsigned int maxsize) const {
	static const char * rot[20] = {
		"~~~~~",
		"-~~~~",
		"--~~~",
		"---~~",
		"----~",
		"-----",
		"----=",
		"---==",
		"--===",
		"-====",
		"=====",
		"+====",
		"++===",
		"+++==",
		"++++=",
		"+++++",
		"++++~",
		"+++~~",
		"++~~~",
		"+~~~~",
	};
	if (!buffer) {
		return rot[counter % 20];
	}
	if (pwheelPrefix) {
		snprintf(buffer,maxsize,"%s %s:    %s    ",pwheelPrefix, dname.c_str(),rot[counter%20]);
	} else {
		snprintf(buffer,maxsize,"%s:    %s    ",dname.c_str(),rot[counter%20]);
	}
	return buffer;
}
	



IPLDriver::DriverStatus IPLDriver::nextFrame(bool printtree)
{
    assert(scheduler);
#ifndef NDEBUG
	if (progressWheel) {
		printf("\r%s:    %s    \r",dname.c_str(),getProgressWheel());
		fflush(stdout);
	}
#endif
	scheduler->setProfiling(profiling);
	counter += 1;

	bool res = scheduler->runProcTree();
#ifndef NDEBUG
	if (printtree) {
		// we print the tree here because some of the module only knows the
		// type of their output after the first pass
		char tmp[256];
		sprintf(tmp,"%s.vcg",dname.c_str());
		scheduler->printVCG(tmp);
		ROS_INFO("%s: Stored processing tree in '%s'",dname.c_str(),tmp);
		sprintf(tmp,"%s.dot",dname.c_str());
		scheduler->printDOT(tmp);
		ROS_INFO("%s: Stored processing tree in '%s'",dname.c_str(),tmp);
	}
	
	if (stepByStep) {
		ROS_INFO("%s: Press <enter> to start processing next frame",dname.c_str());
		fflush(stdout);
		scheduler->startProbeThread();
		while (1) {
			int c = getchar();
			if (c == '\n') break;
			if (c == EOF) break;
		};
		scheduler->stopProbeThread();
	}
#endif
	
	if (!res) return ERROR;
	return READY_FOR_NEXT_FRAME;
}

bool IPLDriver::loadParamsFromFile(const char * fname)
{
	config.clear();
	return config.read(fname);
}

void IPLDriver::applyConfig() {
	scheduler->readConfig(&config);
}

/** Wait for all trigger to be ready */
bool IPLDriver::waitTrigger(double timeout_s) 
{
    if (triggers.empty()) return true;

    double twait = IPLImageProcessor::timeofday() + timeout_s;
    if (timeout_s > 0) {
        struct timespec ts;
        ts.tv_sec = (time_t)floor(twait);
        ts.tv_nsec = (long)((twait - ts.tv_sec)*1e9);
        int res = pthread_cond_timedwait(&triggerCond, &triggerMutex, &ts);
        switch (res) {
            case 0: 
                break;
            case ETIMEDOUT:
                return false;
            default:
                perror("IPLDriver::waitTrigger");
                return false;
        }
    } else {
        int res = pthread_cond_wait(&triggerCond, &triggerMutex);
        switch (res) {
            case 0: 
                break;
            case ETIMEDOUT:
                return false;
            default:
                perror("IPLDriver::waitTrigger");
                return false;
        }
    }
    return true;
}

/** Register that a module is ready */
void IPLDriver::signalTrigger(const IPLImageProcessor * ip)
{
    std::map< const IPLImageProcessor*,bool,std::less<const IPLImageProcessor*> >::iterator it
        = triggers.find(ip);
    // This should not happen.
    assert (it != triggers.end());

    it->second = true;

    bool all_true = true;
    for (it = triggers.begin();it != triggers.end(); it ++) {
        all_true = all_true && it->second;
    }

    if (all_true || !trigger_request_all) {
        pthread_cond_broadcast(&triggerCond);
        resetTriggers();
    }
}

/** Reset the trigger readiness flag */
void IPLDriver::resetTriggers()
{
    std::map< const IPLImageProcessor*,bool,std::less<const IPLImageProcessor*> >::iterator it;
    for (it = triggers.begin();it != triggers.end(); it ++) {
        it->second = false;
    }
}

