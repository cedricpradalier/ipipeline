#ifndef IPL_GENERIC_ROS_SOURCE_H
#define IPL_GENERIC_ROS_SOURCE_H


#include "ipipeline_core/IPLImageSource.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

/**
 * \class IPLGenericRosSubscriber
 * Base class for a Ros subscriber that exports a RGB image from a ROS topic
 * This class does not do the subscription, as it depends on what mechanism is
 * desired for the transport. Subclasses use either a basic ros subscriber, the
 * default image_transport framework, or the message_transport from the
 * ethz-asl package
 * This is a Source Image Processor, without inputs
 * output is one RGB or Grey image object
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLGenericRosSubscriber : public IPLImageSource
{
	protected :
        bool ready;
        // The CvBridge here is not strictly necessary, but it is simpler, 
        // and would work if we change to IplImage * in all the code
        cv::Mat imgmat_;
        boost::mutex mutex_;
        boost::condition datacond_;
	public :
		IPLGenericRosSubscriber(const char * n);
		virtual ~IPLGenericRosSubscriber();

		virtual bool processInput();

        void callback(const sensor_msgs::ImageConstPtr& msg);
};




#endif // IPL_ROS_SOURCE_H
