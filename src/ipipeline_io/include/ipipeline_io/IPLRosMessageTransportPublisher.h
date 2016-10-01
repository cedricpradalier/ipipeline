#ifndef IPL_MESSAGE_TRANSPORT_ROS_PUBLISHER_H
#define IPL_MESSAGE_TRANSPORT_ROS_PUBLISHER_H


#include "ipipeline_core/IPLImageProcessor.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_transport/publisher.h>

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
class IPLMessageTransportRosPublisher : public IPLGenericRosPublisher
{
	protected :
		message_transport::MessageTransport<sensor_msgs::Image> it_;
        message_transport::Publisher pub_;

        virtual void publish() {
            pub_.publish(image_);
        }
	public :
		IPLMessageTransportRosPublisher(const char * n, ros::NodeHandle & nh, const std::string & topic) 
            : IPLGenericRosPublisher(n), 
            it_(nh,"imagem_transport","sensor_msgs::Image") {
                pub_ = it.advertise(topic,1);
            }
		virtual ~IPLMessageTransportRosPublisher() {}

};




#endif // IPL_MESSAGE_TRANSPORT_ROS_PUBLISHER_H


