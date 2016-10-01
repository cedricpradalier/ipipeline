#ifndef IPL_CACHING_SOURCE_H
#define IPL_CACHING_SOURCE_H

#include "IPLImageSource.h"

/**
 * \class IPLCachingSource : abstract class
 * The basis for sources able to use several time the same image
 * An image source has no input
 * output is a set of ImageProcessorOutput
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLCachingSource : public IPLImageSource
{
	protected :
		bool activate_cache;
		bool update_cache;

	public :
		/**
		 * \arg t : type of the output
		 * \arg n : number of output images, 
		 * \arg w,h : image size
		 * **/
		IPLCachingSource(const char * name, 
                IPLImageProcessorRole role, 
				int type, unsigned int n=1,
				unsigned int w=0, unsigned int h=0);
		virtual ~IPLCachingSource();

		bool cachingIsActive() {return activate_cache;}
		void activateCaching(bool act=true) {activate_cache = act;}
		void stopCaching() {activate_cache = false;}

		void updateCache() {update_cache = true;}

		bool needsUpdate() {return !activate_cache || update_cache;}
		void updateDone() {update_cache = false;}
};




#endif // IPL_CACHING_SOURCE_H
