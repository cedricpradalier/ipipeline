#ifndef _IPL_DRIVER_H_
#define _IPL_DRIVER_H_

#include "IPLIPScheduler.h"
#include "IPLImageProcessor.h"
#include "IPLCachingSource.h"

/**
 * \class IPLDriver
 * Class implementing the dataflow management of the video processing algorithm
 * \sa IPLImageProcessor \sa IPLIPScheduler 
 * \sa IPLSharedBuffers
 * **/
class IPLDriver 
{
	public :
		/** 
		 * Constructor
		 * **/
		IPLDriver(const std::string & name);
		virtual ~IPLDriver();


		/**
		 * Read parameters from file fname
		 * **/
		bool loadParamsFromFile(const char * fname); 

		/** 
		 * Returns the config object loaded with
		 * loadParamsFromFile
		 * */
		Config * getConfig() {
			return &config;
		}
		/**
		 * Apply the config object to the scheduler
		 * */
		void applyConfig();

		/**
		 * Copy a config object already parsed
		 * **/
		void setConfig(const Config & cfg) {config = cfg;}
		
		typedef enum {ERROR, COMPLETED,
			READY_FOR_RESTART, READY_FOR_NEXT_FRAME} DriverStatus;
		/** 
		 * Process next frame, that is ask the scheduler to run the processing
		 * tree. If printtree, the processing tree is output as DOT/VCG format
		 * browsable with dotty/xvcg
		 * Returns a value from incrZIndex
		 * **/
		virtual DriverStatus nextFrame(bool printtree = false);

		/**
		 * Initialize the class for computing a new voxel space
		 * **/
		void updateCachedSources();

		/**
		 * Initialise the scheduler and internal variables
		 * Assumes the config has been read.  A factory object F is 
         * assumed to be a functor returning a new IPLGenericProbe object
         * each time it is called, with an index number as argument: F(i)
         * should create the ith probe. The pointer ownership is taken
         * by the IPLDriver object **/
        template <class ProbeFactory>
            bool prepareScheduler(ProbeFactory * f) {
                int numProbes = 0;
                int numThreads = 1;

                if (config.selectSection("IPLDriver")) {
                    config.getInt("numProbes",&numProbes);
                    config.getInt("numThreads",&numThreads);
                    config.getBool("progressWheel",&progressWheel);
                    config.getBool("profiling",&profiling);
                    config.getBool("stepByStep",&stepByStep);
                } else {
                    ROS_WARN("No IPLDriver section in config file");
                }
                
                // Call any user specific functions from there
                this->initialise();

                ROS_INFO("Launching scheduler");
                // This calls the buildProcessingTree function
                // and pass the configuration to the modules
                if (!createScheduler(numThreads,numProbes>0)) {
                    ROS_ERROR("Scheduler creation failed");
                    return false;
                }
                if (f && numProbes) {
                    addProbeModules(numProbes,*f);
                }

                return true;
            }

	protected :
		/**
		 * Called at the end of the main initialisation to prepare driver
         * specific variable
		 * */
		virtual bool initialise() {ROS_INFO("Using default initialiser");return true;}


		/**
		 * Add probes into the processing tree. A factory object F is 
         * assumed to be a functor returning a new IPLGenericProbe object
         * each time it is called, with an index number as argument: F(i)
         * should create the ith probe. The pointer ownership is taken
         * by the IPLDriver object
		 * \see IPLProbe
		 * **/
        template <class ProbeFactory>
            bool addProbeModules(unsigned int numProbes, ProbeFactory & f) {
                for (unsigned int i=0;i<numProbes;i++) {
                    scheduler->storeProbe(f(i));
                }
                ROS_INFO("%s: Added %d probes",dname.c_str(),numProbes);
                return true;
            }
		
		/** 
		 * Store an Image Processor into the processing tree,
		 * flag it as a source module if needed. Source module 
		 * are used to start the processing
		 * **/
		void store(IPLImageProcessor * ip);

		/**
		 * After reading the parameters, this function call the other
		 * initialisation functions
		 * **/
		bool createScheduler(unsigned int numThreads, bool dynamicScheduler);
		
		/**
		 * Build the complete processing tree, To be defined 
		 * **/
		virtual bool buildProcessingTree() = 0;

	public:
		void setProgressWheel(bool val) {progressWheel = val;}
		void setProfiling(bool val) {profiling = val;}
		void setStepByStep(bool val) {stepByStep = val;}
		const std::string & getName() const {return dname;}
	protected :
		std::string dname;
		bool waitingCoreInit;
		bool progressWheel;
		bool profiling;
		bool stepByStep;
		const char* pwheelPrefix;
		unsigned int counter;
		const char* getProgressWheel(char *buffer=NULL,unsigned int buffersize=0) const;
		void setProgressWheelPrefix(const char * prefix) {
			pwheelPrefix = prefix;
		}


		IPLIPScheduler * scheduler;
		std::vector<cv::Mat1b> masks;
		std::vector<IPLCachingSource *> cached;

		Config config;
    public:
        /** Define the trigger mode: if wait_for_all, then 
         * the condition is released when all registered trigger have been
         * signaled, otherwise, only one is necessary */
        void setTriggerMode(bool wait_for_all) {
            trigger_request_all = wait_for_all;
        }

        /** Wait for all trigger to be ready. Return true if there is 
         * something to do. timeout in second, relative, not yet implemented */
        bool waitTrigger(double timeout_s);

        /** Register that a module is ready */
        void signalTrigger(const IPLImageProcessor * ip);

        /** Reset the trigger readiness flag */
        void resetTriggers();

        /** Used to check if this driver has triggers */
        bool hasTriggers() {return !triggers.empty();}

    protected:
        /** All tools for trigger management **/
        std::map< const IPLImageProcessor*,bool,std::less<const IPLImageProcessor*> > triggers;
        pthread_mutex_t triggerMutex;
        pthread_cond_t triggerCond;
        bool trigger_request_all;

        class DriverTriggerCallback : public IPLTriggerCallback {
            protected:
                IPLDriver *driver_;
            public:
                DriverTriggerCallback(IPLDriver * driver) : driver_(driver) {}
                virtual void call(const IPLImageProcessor * ip) {
                    driver_->signalTrigger(ip);
                }
        };

        DriverTriggerCallback triggerCallback;
};


#endif // _IPL_DRIVER_H_
