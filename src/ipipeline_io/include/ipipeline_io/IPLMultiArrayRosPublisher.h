#ifndef IPL_MULTI_ARRAY_ROS_PUBLISHER_H
#define IPL_MULTI_ARRAY_ROS_PUBLISHER_H


#include "ipipeline_core/IPLImageProcessor.h"


#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

/**
 * \class IPLMultiArrayRosSubscriber
 * Base class for a Ros publisher to export a generic cv::Mat (as CV_32FC1)
 * output is one cv::Mat object
 * \see IPLImageProcessor for definition of virtual functions
 * **/
class IPLMultiArrayRosPublisher : public IPLImageProcessor
{
	protected :
		unsigned int deftInput;
        std_msgs::Float32MultiArray array;
        ros::Publisher pub;
	public :
		IPLMultiArrayRosPublisher(const char * n, 
                ros::NodeHandle & nh, const std::string & topic);
		virtual ~IPLMultiArrayRosPublisher() {}

		virtual bool processInput();

		virtual bool checkInput() const;

};




#endif // IPL_MULTI_ARRAY_ROS_PUBLISHER_H
