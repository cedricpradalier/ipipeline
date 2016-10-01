#ifndef IPL_ROS_MESSAGE_TRANSPORT_SUBSCRIBER_H
#define IPL_ROS_MESSAGE_TRANSPORT_SUBSCRIBER_H


#include "ipipeline_io/IPLGenericRosSubscriber.h"


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_transport/message_transport.h>


/**
 * \class IPLRosMessageTransportSubscriber
 * Export a RGB image from a ROS topic
 * This is a Source Image Processor, without inputs
 * output is one RGB or Grey image object
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLRosMessageTransportSubscriber : public IPLGenericRosSubscriber
{
	protected :
        message_transport::Subscriber tsub_;
        message_transport::MessageTransport<sensor_msgs::Image> it_;
	public :
		IPLRosMessageTransportSubscriber(const char * n, ros::NodeHandle & nh, 
                const std::string & topic, 
                const std::string & transport = "imagem_transport/raw") :
            IPLGenericRosSubscriber(n), 
            it_(nh, "imagem_transport", "sensor_msgs::Image") {
            tsub_ = it_.subscribe(topic,1,&IPLGenericRosSubscriber::callback,this,transport);
        }
		virtual ~IPLRosMessageTransportSubscriber() {}

};




#endif // IPL_ROS_IMAGE_TRANSPORT_SUBSCRIBER_H
