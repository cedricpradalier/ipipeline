#ifndef IPL_GENERIC_PROBE_H
#define IPL_GENERIC_PROBE_H

#include "IPLImageProcessor.h"
#include "IPLProbeCommon.h"

/**
 * \class IPLGenericProbe 
 * Abstract definition of a probe to see output of other IPLImageProcessor
 * input "Default" can produce any output (even no output)
 * there is no output but input data are written to buffer when possible
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLGenericProbe : public IPLImageProcessor
{
	protected:
		unsigned int inputid;
		unsigned int inputIndex;

		/**
		 * List of available ImageProcessor
		 * **/
		OpStore * opstore;
		/**
		 * Currently monitored ImageProcessor
		 * **/
		OpStore::iterator opit;

        void processCommand(IPLProbeAction action, unsigned int index=0);
        virtual void probeFunction(const cv::Mat & in) {}
	public :
		IPLGenericProbe(const char * n) : IPLImageProcessor(n,IPL_ROLE_PROBE) {
            inputid = addInput("Default");
            inputIndex = 0;
        }

		~IPLGenericProbe() {}

		virtual bool checkInput() const {
			// Nothing to do, everything is good for a probe
			return true;
		}

        virtual bool processInput() {
            // ROS_INFO("Probe process");
            IPLImageProcessor * input = getInput(inputid);
            // A probe might run outside of the tree, so if its input is 
            // not ready it should be able to ask the last available.
            if (input == NULL) input = *opit;

            // If it is still NULL, nevermind
            if (input == NULL) return true;


            ImageProcessorOutput in = input->getOutput();
            if (in.size() == 0) return true;
            if (in.size() <= inputIndex) {
                inputIndex = 0;
            }

            this->probeFunction(in[inputIndex]);

            // ROS_INFO("Probe on %s",input->getName());
            return true;
        }

		/** 
		 * Set the list of available ImageProcessor. 
		 * Called by IPLIPScheduler
		 * **/
		void setStore(OpStore * op) {
			opstore = op;
			opit = opstore->begin();
            if (opit != opstore->end()) {
                (*opit)->addReceiver(this);
            }
		}

		/**
		 * Read probeAct, and react accordingly
		 * Set probeAct to Probe_Still afterward
		 * Called by IPLIPScheduler::manageProbes
         * Must be overloaded by real implementation, otherwise the probe
         * cannot move in the processing tree
		 * **/
		virtual void updateProbe() {};
};


#endif // IPL_BUFFER_PROBE_H
