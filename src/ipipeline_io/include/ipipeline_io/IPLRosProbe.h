#ifndef IPL_ROS_PROBE_H
#define IPL_ROS_PROBE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include "ipipeline_core/IPLGenericProbe.h"
#include "ipipeline_io/ProbeCommand.h"

#include <boost/thread/mutex.hpp>

/**
 * \class IPLRosProbe 
 * Implementation of a probe to see output of other IPLImageProcessor
 * input "Default" can produce any output (even no output)
 * there is no output but input data are written to buffer when possible
 * \see IPLImageProcessor for virtual function 
 * **/
class IPLRosProbe : public IPLGenericProbe
{
	protected :
        bool firstpub_;
        boost::mutex mutex_;
		image_transport::ImageTransport it_;
        image_transport::Publisher pub_;
        ros::Publisher status_pub_;
        ros::Subscriber sub_;
        ipipeline_io::ProbeCommand command_;
        sensor_msgs::Image image_;
		virtual void probeFunction(const cv::Mat & in);
        void commandCallback(const ipipeline_io::ProbeCommandConstPtr & msg);
	public :
		/**
		 * Constructor:
		 * \arg n: Probe name
		 * **/
		IPLRosProbe(const char * n, ros::NodeHandle & nh);

		~IPLRosProbe() {}

		/**
		 * Read probeAct, and react accordingly
		 * Set probeAct to Probe_Still afterward
		 * Called by IPLIPScheduler::manageProbes
		 * **/
		virtual void updateProbe();
};


#endif // IPL_ROS_PROBE_H
