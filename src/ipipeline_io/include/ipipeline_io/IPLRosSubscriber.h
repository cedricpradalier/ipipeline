#ifndef IPL_ROS_SUBSCRIBER_H
#define IPL_ROS_SUBSCRIBER_H


#include "ipipeline_io/IPLGenericRosSubscriber.h"


#include <ros/ros.h>


/**
 * \class IPLGenericRosSubscriber
 * Export a RGB image from a ROS topic
 * This is a Source Image Processor, without inputs
 * output is one RGB or Grey image object
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLRosSubscriber : public IPLGenericRosSubscriber
{
	protected :
        ros::Subscriber sub_;
	public :
		IPLRosSubscriber(const char * n, ros::NodeHandle & nh, 
                const std::string & topic) : IPLGenericRosSubscriber(n) {
            sub_ = nh.subscribe(topic,1,&IPLGenericRosSubscriber::callback,this);
        }
		virtual ~IPLRosSubscriber() {}
};




#endif // IPL_ROS_SUBSCRIBER_H
