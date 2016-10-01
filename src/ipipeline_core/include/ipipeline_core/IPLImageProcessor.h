#ifndef IPL_GENERAL_IMAGE_PROCESSOR
#define IPL_GENERAL_IMAGE_PROCESSOR

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <vector>
#include <set>
#include <string>

#include <ros/ros.h>

#include "ipipeline_core/Config.h"
#include "opencv2/core/core.hpp"
#include "ipipeline_core/IPLConditions.h"

class IPLIPScheduler;
class IPLImageProcessor;

struct OpStore_LtPtr {
	bool operator()(IPLImageProcessor * p1, IPLImageProcessor * p2) const
	{
		return (long unsigned int)p1 < (long unsigned int)p2;
	}
};
typedef std::set<IPLImageProcessor*, OpStore_LtPtr> OpStore;
typedef const std::vector<cv::Mat> & ImageProcessorOutput;

template <class T>
const cv::Mat_<T> & castto(const cv::Mat & m) {
    return (const cv::Mat_<T> &)(m);
}

struct IPLTriggerCallback { 
    virtual ~IPLTriggerCallback() {}
    virtual void call(const IPLImageProcessor * ip) = 0;
};

typedef enum {
    IPL_ROLE_DEFAULT=0x00,
    IPL_ROLE_SOURCE=0x01,
    IPL_ROLE_TRIGGER=0x02,
    IPL_ROLE_CACHING=0x04,
    IPL_ROLE_PROBE=0x80
} IPLImageProcessorRole;

/** 
 * \class IPLImageProcessor
 * Generic mother class for any image processor object.
 * Contains all the generic code in order to ensure that an
 * image processor object only have to implement a processing 
 * function
 * **/
class IPLImageProcessor 
{
	public :
		friend class IPLIPScheduler;
	public :
		/** public readonly accessor. Should contains the frameid, that is the
		 * number of the processed frame **/
		const unsigned int &frameid;
	public :
		/**
		 * Constructor.
		 * \arg n is the name of the Image Processor Object 
		 * **/
		IPLImageProcessor(const char * n, IPLImageProcessorRole role = IPL_ROLE_DEFAULT);
		
		/**
		 * Constructor.
		 * \arg n is the name of the Image Processor Object 
		 * nimages from \arg in.
		 * **/
		IPLImageProcessor(const char * n, IPLImageProcessor * in, 
                IPLImageProcessorRole role = IPL_ROLE_DEFAULT);

		virtual ~IPLImageProcessor();

		/** retrieve object name **/
		const char * getName() {return name;}
		/** retrieve object id **/
		unsigned int getID() {return id;}

        IPLImageProcessorRole getRole() {return role;}

		/** 
		 * Set the object condition
		 * \arg text is the condition name
		 * \arg cond is the condition object
		 * \arg destroyIt indicates if the condition object must be 
		 * destroyed in the object destructor
		 * **/
		void setCondition(const char * text, 
				Condition * cond, bool destroyIt=true);
		/**
		 * Set the object condition by creating an internal condition
		 * linked to variable v. \see BoolCondition
		 * **/
		void setCondition(const char * text, const bool & v);
		void setCondition(const char * text, const signed int & v);
		void setCondition(const char * text, const unsigned int & v);
		/** 
		 * Set the object condition using master's condition
		 * **/
		void setCondition(const IPLImageProcessor * master);

		/** Get generic output, NULL is not redefined in the inherited class **/
		virtual ImageProcessorOutput getOutput() const;

        template <class T>
        const cv::Mat_<T> & getCastOutput(unsigned int index=0)  const {
            const std::vector<cv::Mat> & gen = this->getOutput();
            assert(index < gen.size());
            return (const cv::Mat_<T> &)(gen[index]);
        }

        void reallocate(int type, unsigned int w, unsigned int h, unsigned int nimg=1);
        void reallocate(const std::vector<cv::Mat> & vin);
        void reallocate(const cv::Mat & min);

		/** Connect this object to \arg rec on input \arg inputname **/
		void addReceiver(IPLImageProcessor * rec, const char * inputname);
		/** Connect this object to \arg rec on input inputid **/
		void addReceiver(IPLImageProcessor * rec, unsigned int inputid=0);
		/** Remove rec from the list of this object's receivers **/
		void removeReceiver(IPLImageProcessor * rec);

		/** 
		 * Make profiling_flag points to a specific boolean flag. 
		 * Profiling will be activated when this flag will be true
		 * **/
		void registerProfilingFlag(bool * flag) {profiling_flag = flag;}
		/** Retrieve time spent in the processInput function of this object **/
		float getProfilingTime() const {return profiling_total;}
		float getProfilingRuns() const {return profiling_numruns;}

		/** Print the tree spanning from this module into fp **/
		void print(FILE * fp);
		/** PrettyPrint the tree spanning from this module into fp **/
		void printTree(FILE * fp);
		
		/** 
		 * Print the VCG description of the processing tree spanning from
		 * source. ImageProcessor in blacklist are skipped.
		 * Output is stored in fname
		 * **/
		static void printVCG(const char * fname, 
				OpStore & sources, OpStore & blacklist);
		/** 
		 * Print the DOT description of the processing tree spanning from
		 * source. ImageProcessor in blacklist are skipped.
		 * Output is stored in fname
		 * **/
		static void printDOT(const char * fname, 
				OpStore & sources, OpStore & blacklist);
		
		/** 
		 * Print the VCG description of the processing tree spanning from
		 * this object. Output is stored in fname
		 * **/
		void printVCG(const char * fname);
		/** 
		 * Print the DOT description of the processing tree spanning from
		 * this object. Output is stored in fname
		 * **/
		void printDOT(const char * fname);


        std::string createOutputTypeStr(const cv::Mat & m) const;
        std::string createOutputTypeStr() const;

		/**
		 * check that processing tree spanning from this is completely
		 * connected 
		 * **/
		bool checkProcessingTree() const;

		/**
		 * Helper function that gives the time of day in second
		 * using gettimeofday
		 * **/
		static double timeofday();

        /**
         * Register a callback to be called when the IP calls
         * signalTrigger
         * */
        void registerTriggerCallback(IPLTriggerCallback * cb) {
            triggerCallback = cb;
        }

		void checkState() const; // DEBUG
	protected :
		/** object id. \see idcounter **/
		unsigned int id;

		/** Object name. Useful for printing/browsing **/
		char * name;
		/** rw variable associated to frameid **/
		unsigned int _frameid;

        /** Definition of the role of this IP */
        IPLImageProcessorRole role;

        std::vector<cv::Mat> outputVector;
		
		/** Set current frame id **/
		void setFrameId(unsigned int i);

		/**
		 * Add an input name to this object
		 * **/
		unsigned int addInput(const char * name);

		/** 
		 * Retrieve input \arg name. 
		 * To be used in the processInput function.
		 * **/
		IPLImageProcessor * getInput(const char * name) const;
		/** 
		 * Retrieve input \arg id. 
		 * To be used in the processInput function.
		 * **/
		IPLImageProcessor * getInput(unsigned int id) const;
		/**
		 * store \arg input as the received input on entry \arg id
		 * **/
		void setInput(unsigned int id, IPLImageProcessor * input);
		/**
		 * Change input name associated with entry id
		 * Mainly useful to change the name of an inherited entry.
		 * Avoid to use it! 
		 * **/
		void setInputName(unsigned int id, const char * iname);

		/**
		 * Pure virtual function
		 * Must be implemented by any inherited class
		 * Will be called as soon as all the input have been received and when
		 * the condition is true.
		 * Must return true, if successfully terminated
		 * **/
		virtual bool processInput() = 0;

		/**
		 * Pure virtual function
		 * Must be implemented by any inherited class
		 * Will be called prior to processInput to check that all the input are
		 * of a correct type. The result of this function will be asserted.
		 * This is a debug tool
		 * **/
		virtual bool checkInput() const = 0;

		/**
		 * Used in checkInput functions to return an error
		 * **/
		bool error(const char * text,...) const;

		typedef enum {
			INCONSISTENT_TYPE,
			INVALID_TYPE,
			INVALID_IMAGE_SIZE,
			INCONSISTENT_INPUT_SIZE,
			INVALID_INPUT_SIZE,
			LAST_ERROR_ID
		} ErrorMessageId;
		bool error(ErrorMessageId id,const std::string & extra=std::string()) const;

		/**
		 * name of the input to transfer as output when the 
		 * condition is false
		 * **/
		void setPassThrough(const char * inputname);

		/**
		 * Compare the output of two image processor and check that 
		 * they have the same size.
		 * Useful in checkInput functions 
		 * **/
		static bool sameSize(const IPLImageProcessor* ip1, const IPLImageProcessor* ip2) {
			const std::vector<cv::Mat> &I1 = ip1->getOutput();
			const std::vector<cv::Mat> &I2 = ip2->getOutput();
            if (I1.size() != I2.size()) return false;
            for (unsigned int i=0;i<I1.size();i++) {
                if (I1[i].size() != I2[i].size()) {
                    return false;
                }
            }
			return true;
		}

		/**
		 * Compare the output of two image processor and check that 
		 * they have the same size and type.
		 * Useful in checkInput functions 
		 * **/
		static bool sameSizeAndType(const IPLImageProcessor* ip1, const IPLImageProcessor* ip2) {
			const std::vector<cv::Mat> &I1 = ip1->getOutput();
			const std::vector<cv::Mat> &I2 = ip2->getOutput();
            if (I1.size() != I2.size()) return false;
            for (unsigned int i=0;i<I1.size();i++) {
                if (I1[i].size() != I2[i].size()) {
                    return false;
                }
                if (I1[i].type() != I2[i].type()) {
                    return false;
                }
            }
			return true;
		}

        static bool saveMatToFile(const cv::Mat & mat, const std::string & filename);

		/** 
		 * Configuration function, called by the scheduler.
		 * Before calling it, the scheduler set the config section to the 
		 * name of the module. 
		 * */
		virtual void readConfig(Config * config);


        /** 
         * To be called when data is ready
         * */
        void signalTrigger() {
            if (triggerCallback) {
                triggerCallback->call(this);
            }
        }


	private :
		/**
		 * static counter to assign a unique id to each new image processor
		 * object 
		 * **/
		static unsigned int idcounter;
		/**
		 * id of the input to transfer as output when the condition is false
		 * **/
		unsigned int passthroughid;

		/** 
		 * flag to indicate that this object has been processed. used in 
		 * recursive traversal such as print, printVCG, printDOT
		 * **/
		bool printed;
		
		/** 
		 * The condition object that will be tested before calling
		 * processInput
		 * **/
		Condition * condition;
		/** should this condition object be destroyed in the destructor **/
		bool destroyCondition;
		/** name of the condition: display, browsing, debug **/
		char * condtext;

		/** 
		 * Set of image processor this image processor is connected to, and on
		 * which input id.
		 * **/
		typedef std::pair<IPLImageProcessor*,unsigned int> Receiver;
		std::vector<Receiver> receiver;

		/** \class IPLImageProcessor::Input
		 * Definition of an input entry point 
		 * **/
		struct Input{
			/** input name **/
			std::string name;
			/** received image processor on this input **/
			IPLImageProcessor * received;
			/** has been connected **/
			bool connected;
			Input(const char * n) : name(n), received(NULL), connected(false) {}
			/** mark input as not received **/
			void reset() {received = NULL;}
		} ;
		/** the list of input entries **/
		std::vector<Input> inputs;
		/**
		 * Get the inputid associated to name
		 * **/
		int getInputId(const char * name) const;
		/**
		 * Set all input state at "not received".
		 * **/
		void resetInputsReceived();
		/**
		 * Test if all the input have been received
		 * **/
		bool hasReceivedAllInputs() const;
		
		/** 
		 * Prepare the processing tree spanning from this to be printed.
		 * That is essentially setting printed to false
		 * **/
		void recPreparePrint();
		/** recursive function to pretty print the tree **/
		void recPrintTree(FILE * fp, unsigned int indent = 0);
		/** recursive function to print VCG representation of the tree **/
		void recPrintVCG(FILE * fp,bool edge,
				const OpStore & blacklist);
		/** recursive function to print DOT representation of the tree **/
		void recPrintDOT(FILE * fp,bool edge,
				const OpStore & blacklist);

		/** pointer to the profiling flag **/
		bool * profiling_flag;
		/** total time spent in this processInput function, in seconds **/
		float profiling_total;
		unsigned int profiling_numruns;

		/** 
		 * try to call this processInput function, and 
		 * signal output to receiver though scheduler
		 * **/
		void testAndProcessInput(IPLIPScheduler * scheduler);

		/**
		 * register the value of an input
		 * return true, if the IP is ready to be run
		 * **/
		bool registerInput(IPLImageProcessor * input, unsigned int inputid);

        /**
         * Stores a functor to be called when the image processor wants to
         * signal its readyness
         * */
        IPLTriggerCallback * triggerCallback;
        
};
		
#endif // IPL_GENERAL_IMAGE_PROCESSOR
