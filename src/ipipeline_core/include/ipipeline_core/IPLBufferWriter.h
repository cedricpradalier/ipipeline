#ifndef IPL_BUFFER_WRITER_H
#define IPL_BUFFER_WRITER_H

#include "IPLImageProcessor.h"
typedef void (*SignalFunction)(void *);

class IPLMonoBufferWriter : public IPLImageProcessor
{
	protected :
		unsigned int inputid;
		
		// destination pointer
		void * bufferDest;
		unsigned int bufferSize;
		char * ipname;
		unsigned int ipnameSize;
		
		
		// destination size
		unsigned int *widthDest, *heightDest;
		
		// Image to dump
		unsigned int inputIndex;

		// Function to call once image has been written,
		// to raise a semaphore for instance
		SignalFunction signalF;
		void * signalArg;
	public :
		IPLMonoBufferWriter(const char * n, IPLImageProcessor * in,
				void *  buffer, unsigned int bufsize,
				char * iname, unsigned int inamesize,
				unsigned int * widthp, unsigned int * heightp,
				unsigned int index = 0, 
				SignalFunction fdone=NULL,void * arg=NULL);

		~IPLMonoBufferWriter() {}

		virtual bool processInput();

		virtual bool checkInput() const;
};

#if 0
class IPLStereoBufferWriter : public IPLImageProcessor
{
	protected :
		unsigned int inputid;
		
		// destination pointer
		void * buffer1Dest;
		void * buffer2Dest;
		unsigned int bufferSize;
		
		char * ipname;
		unsigned int ipnameSize;
		
		// destination size
		unsigned int *widthDest, *heightDest;
		
		// Image to dump
		unsigned int inputIndex1,inputIndex2;

		// Function to call once image has been written,
		// to raise a semaphore for instance
		SignalFunction signalF;
		void * signalArg;
	public :
		IPLStereoBufferWriter(const char * n, IPLImageProcessor * in,
				void *  buffer1, void * buffer2, unsigned int bufsize,
				unsigned int * widthp, unsigned int * heightp,
				unsigned int index1 = 0, unsigned int index2 = 1,
				SignalFunction fdone=NULL,void * arg=NULL);

		~IPLStereoBufferWriter() {}

		virtual bool processInput();
};

#endif


#endif // IPL_BUFFER_WRITER_H
