#ifndef IPL_IMAGE_TRANSPORT_ROS_PUBLISHER_H
#define IPL_IMAGE_TRANSPORT_ROS_PUBLISHER_H


#include "ipipeline_io/IPLGenericRosPublisher.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>

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
class IPLRosImageTransportPublisher : public IPLGenericRosPublisher
{
	protected :
		image_transport::ImageTransport it_;
        image_transport::Publisher pub_;

        virtual void publish() {
            pub_.publish(image_);
        }
	public :
		IPLRosImageTransportPublisher(const char * n, ros::NodeHandle & nh, const std::string & topic) 
            : IPLGenericRosPublisher(n), it_(nh) {
                pub_ = it_.advertise(topic,1);
            }
		virtual ~IPLRosImageTransportPublisher() {}

};




#endif // IPL_IMAGE_TRANSPORT_ROS_PUBLISHER_H


