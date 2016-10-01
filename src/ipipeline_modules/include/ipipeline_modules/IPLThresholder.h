#ifndef IPL_THRESHOLDER_H
#define IPL_THRESHOLDER_H

#include "ipipeline_core/IPLImageFilter.h"

/**
 * \class IPLThresholder : \see IPLImageFilter
 * Threshold input using ippiThreshold function 
 * input "Default" must output a set of Int8u images
 * output is as many image as input, with same type
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLThresholder : public IPLImageFilter
{
	protected :
        double threshold;
        double max_value;
        int threshold_type;
	public :
		/**
         * See OpenCV threshold function
		 * **/
		IPLThresholder(const char * n,
				 double thr, double max_val, int thr_type);

		virtual bool processInput();

		virtual bool checkInput() const;

        virtual void readConfig(Config *config);
};




#endif // IPL_THRESHOLDER_H
