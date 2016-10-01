#ifndef IPL_ROS_IMAGE_TRANSPORT_SUBSCRIBER_H
#define IPL_ROS_IMAGE_TRANSPORT_SUBSCRIBER_H


#include "ipipeline_io/IPLGenericRosSubscriber.h"


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/bind.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>


/**
 * \class IPLRosImageTransportSubscriber
 * Export a RGB image from a ROS topic
 * This is a Source Image Processor, without inputs
 * output is one RGB or Grey image object
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLRosImageTransportSubscriber : public IPLGenericRosSubscriber
{
	protected :
        image_transport::Subscriber tsub_;
        image_transport::ImageTransport it_;
	public :
		IPLRosImageTransportSubscriber(const char * n, ros::NodeHandle & nh, 
                const std::string & topic,
                const std::string & transport = "image_transport/raw") :
            IPLGenericRosSubscriber(n), it_(nh) {
            tsub_ = it_.subscribe<IPLRosImageTransportSubscriber>(topic,1, &IPLRosImageTransportSubscriber::callback,this,transport);
            // image_transport::TransportHints hints(transport);
            // tsub_ = it_.subscribe(topic,1, boost::bind(&IPLGenericRosSubscriber::callback,this,_1),hints);
        }
		virtual ~IPLRosImageTransportSubscriber() {}

};




#endif // IPL_ROS_IMAGE_TRANSPORT_SUBSCRIBER_H
