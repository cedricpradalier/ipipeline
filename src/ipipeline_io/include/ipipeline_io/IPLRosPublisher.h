#ifndef IPL_ROS_PUBLISHER_H
#define IPL_ROS_PUBLISHER_H


#include "ipipeline_core/IPLImageProcessor.h"

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
class IPLRosPublisher : public IPLGenericRosPublisher
{
	protected :
        ros::Publisher pub_;

        virtual void publish() {
            pub_.publish(image_);
        }
	public :
		IPLRosPublisher(const char * n, ros::NodeHandle & nh, const std::string & topic) 
            : IPLGenericRosPublisher(n) {
                pub_ = nh.advertise<sensor_msgs::Image>(topic,1);
            }
		virtual ~IPLRosPublisher() {}

};




#endif // IPL_ROS_PUBLISHER_H


