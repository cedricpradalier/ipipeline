

/* Includes */

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <time.h>

#include <ros/ros.h>

#include "ipipeline_core/IPLGenericProbeFactory.h"
#include "ipipeline_io/IPLRosProbeFactory.h"
#include "IPLTestDriver.h"

/** 
 * This program instantiate a IPLDriver object.
 * Use SDL/sdlprobe to explore this processing
 * **/

unsigned int end = 0;

void sighdl(int n) 
{
	end += 1;
	ROS_INFO("Countdown : end = %d",4-end);
	if (end > 3) {
		kill(getpid(),SIGKILL);
	}
}
		
//#define ONLY_N_FRAMES 20


int main(int argc,char * argv[])
{
	bool first = true;
	unsigned int counter = 0;
#ifdef ONLY_N_FRAMES
	unsigned int nbreadframes = 0;
#endif
	struct timeval tv;
	double t0,t1;

    ros::init(argc,argv,"IPLTest");
    ros::NodeHandle nh;

	signal(SIGINT,sighdl);
	
	/****** IPL initialization ******/
	if (argc < 2) {
		ROS_ERROR("Expecting one cmdline argument (config file)");
		return 1;
	}
    IPLRosProbeFactory gpf(nh);
	IPLTestDriver spd; 

    if (!spd.loadParamsFromFile(argv[1])) {
		ROS_ERROR("Fail to load config file %s",argv[1]);
    }

	if (!spd.prepareScheduler(&gpf)) {
		ROS_ERROR("initialisation failed");
		return 1;
	}


	/**** Initilizing fps accounting ****/
	counter = 0;
	gettimeofday(&tv,NULL);
	t0 = tv.tv_sec + 1e-6*tv.tv_usec;

	ROS_INFO("In the loop");
    ros::AsyncSpinner spinner(1); 
    spinner.start();

	while (ros::ok() && (end<1)) {
        // Wait for triggers, if any
        if (!spd.waitTrigger(1.0)) {
            continue;
        }

		/***** Video Processing *****/
		switch (spd.nextFrame(first)) {
			case IPLDriver::ERROR :
				ROS_ERROR("Processing error");
				end++;
				break;
			case IPLDriver::COMPLETED :
				end+=2;
				break;
			case IPLDriver::READY_FOR_RESTART :
				counter += 1;
				spd.updateCachedSources();
				break;
			case IPLDriver::READY_FOR_NEXT_FRAME :
				break;
		}
		first = false;

		/***** FPS Accounting *****/
		gettimeofday(&tv,NULL);
		t1 = tv.tv_sec + 1e-6*tv.tv_usec;
		if ((counter >= 10) && (t1 - t0 > 3.0)) {
			double fps = counter / (t1-t0);
			t0 = t1; counter = 0;
			ROS_INFO("FPS : %.2f",fps);
		}

	}
	printf("Terminating\n");

	return 0;
}
