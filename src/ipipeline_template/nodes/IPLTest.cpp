

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
		
int main(int argc,char * argv[])
{
    // Standard ROS initialization
    ros::init(argc,argv,"IPLTest");
    ros::NodeHandle nh;

    // We want one argument: the config file
	if (argc < 2) {
		ROS_ERROR("Expecting one cmdline argument (config file)");
		return 1;
	}

    // We handle Ctrl-C ourselves.
	signal(SIGINT,sighdl);
	
    // Create a probe factory that use our node handle.
    IPLRosProbeFactory gpf(nh);

    // And the test driver, that will build the processing tree
	IPLTestDriver spd; 

    // Read the config file and distribute it to all image processors
    if (!spd.loadParamsFromFile(argv[1])) {
		ROS_ERROR("Fail to load config file %s",argv[1]);
    }

    // Create the scheduler and install the probes.
	if (!spd.prepareScheduler(&gpf)) {
		ROS_ERROR("initialisation failed");
		return 1;
	}


	ROS_INFO("In the loop");
	bool first = true;
    // Start a ROS Asyncspinner, because we need to control our loop ourselves.
    // Could be somewhat done with a spinOnce()
    ros::AsyncSpinner spinner(1); 
    spinner.start();

	while (ros::ok() && (end<1)) {
        // Wait for triggers, if any (resulting from subscribers' callbacks).
        if (!spd.waitTrigger(1.0)) {
            continue;
        }

		// Process the received image frame, and the first time, save the
        // processing tree as dot and vcg file
		switch (spd.nextFrame(first)) {
			case IPLDriver::ERROR :
				ROS_ERROR("Processing error");
				end++;
				break;
			case IPLDriver::COMPLETED :
                // If the driver told us that it is done.
				end+=2;
				break;
			case IPLDriver::READY_FOR_RESTART :
                // Only relevant if there are sources that stays constant
                // until something is finished.
				spd.updateCachedSources();
				break;
			case IPLDriver::READY_FOR_NEXT_FRAME :
                // Normal outcome for a real-time image processing application.
				break;
		}
		first = false;

	}
    // And we're done. We'll save the profiling data in the IPLDriver's
    // destructor.
	printf("Terminating\n");

	return 0;
}
