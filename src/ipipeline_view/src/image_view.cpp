/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <ipipeline_io/ProbeCommand.h>
#include <ipipeline_io/ProbeStatus.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

#ifdef HAVE_GTK
#warning compiling for GTK
#include <gtk/gtk.h>

// Platform-specific workaround for #3026: image_view doesn't close when
// closing image window. On platforms using GTK+ we connect this to the
// window's "destroy" event so that image_view exits.
static void destroy(GtkWidget *widget, gpointer data)
{
    ros::shutdown();
}
#endif

class ImageView
{
    protected:
        ros::Publisher pub_;
        ros::Subscriber status_sub_;
        image_transport::Subscriber sub_;

        ipipeline_io::ProbeStatus status_;
        sensor_msgs::ImageConstPtr last_msg_;
        boost::mutex image_mutex_;

        std::string topic_;
        std::string window_name_;
        std::string window_title_;
        unsigned int width_, height_;
        int count_;

    public:
        ImageView(ros::NodeHandle& nh, const std::string& transport) : count_(0)
        {
            topic_ = nh.resolveName("probe");
            ros::NodeHandle local_nh("~");
            local_nh.param("window_name", window_name_, topic_);

            const char* name = window_name_.c_str();
            cvNamedWindow(name, CV_WINDOW_AUTOSIZE );
            cvSetMouseCallback(name, &ImageView::mouse_cb, this);
#ifdef HAVE_GTK
            GtkWidget *widget = GTK_WIDGET( cvGetWindowHandle(name) );
            g_signal_connect(widget, "destroy", G_CALLBACK(destroy), NULL);
#endif
            cvStartWindowThread();
            width_ = 0;
            height_ = 0;

            image_transport::ImageTransport it(nh);
            sub_ = it.subscribe(topic_+"/image", 1, &ImageView::image_cb, this, transport);
            status_sub_ = nh.subscribe(topic_+"/status", 1, &ImageView::status_cb, this);
            pub_ = nh.advertise<ipipeline_io::ProbeCommand>(topic_+"/command",1);
        }

        ~ImageView()
        {
            cvDestroyWindow(window_name_.c_str());
        }

        void image_cb(const sensor_msgs::ImageConstPtr& msg)
        {
            ROS_INFO("Image CB Locking");
            boost::lock_guard<boost::mutex> guard(image_mutex_);
            ROS_INFO("Image CB Locked");

            // Hang on to message pointer for sake of mouse_cb
            last_msg_ = msg;

            // May want to view raw bayer data
            // NB: This is hacky, but should be OK since we have only one image CB.
            if (msg->encoding.find("bayer") != std::string::npos)
                boost::const_pointer_cast<sensor_msgs::Image>(msg)->encoding = "mono8";

            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
            if (img.total()) {
                width_ = msg->width;
                height_ = msg->height;
                ROS_INFO("Showing img ");
                cv::imshow(window_name_.c_str(), img);
                ROS_INFO("Done Showing img ");
            } else {
                ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
            }

            char new_title[256];
            snprintf(new_title,256,"%s: %s[%d] %dx%d",topic_.c_str(),
                    status_.input.c_str(),status_.index, width_, height_);
            if (window_title_ != new_title) {
#ifdef HAVE_GTK
                ROS_INFO("Updating window title");
                GtkWidget * widget = (GtkWidget*)cvGetWindowHandle(window_name_.c_str());
                GtkWidget *toplevel = gtk_widget_get_toplevel (widget);
                if (gtk_widget_is_toplevel (toplevel))
                {
                    /* Perform action on toplevel. */
                    gtk_window_set_title(GTK_WINDOW(toplevel), new_title);
                    window_title_ = new_title;
                }
                ROS_INFO("Updated window title");
#endif
            }
            ROS_INFO("Image CB completed");
        }

        void status_cb(const ipipeline_io::ProbeStatusConstPtr& msg)
        {
            status_ = *msg;
        }

        static void mouse_cb(int event, int x, int y, int flags, void* param)
        {
            ImageView * iv = (ImageView*)param;
            ipipeline_io::ProbeCommand command;
            command.index = 0;
            command.action = ipipeline_io::ProbeCommand::Probe_Noop;
            switch (event) {
                case CV_EVENT_LBUTTONUP:
                    if (flags & CV_EVENT_FLAG_CTRLKEY) {
                        command.action = ipipeline_io::ProbeCommand::Probe_SetIndex;
                        command.index = iv->status_.index + 1;
                    } else {
                        command.action = ipipeline_io::ProbeCommand::Probe_ToNext;
                    }
                    ROS_INFO("Publishing NEXT command");
                    iv->pub_.publish(command);
                    ROS_INFO("Published NEXT command");
                    // publish next command
                    break;
                case CV_EVENT_RBUTTONUP:
                    if (flags & CV_EVENT_FLAG_CTRLKEY) {
                        command.action = ipipeline_io::ProbeCommand::Probe_SetIndex;
                        command.index = iv->status_.index - 1;
                    } else {
                        command.action = ipipeline_io::ProbeCommand::Probe_ToPrev;
                    }
                    ROS_INFO("Publishing PREV command");
                    iv->pub_.publish(command);
                    ROS_INFO("Published PREV command");
                    // publish prev command
                    break;
                case CV_EVENT_MBUTTONUP:
                    command.action = ipipeline_io::ProbeCommand::Probe_Reset;
                    ROS_INFO("Publishing RESET command");
                    iv->pub_.publish(command);
                    ROS_INFO("Published RESET command");
                    // publish reset command
                    break;
            }
        }

        void mainloop() {
#if 0
            while (ros::ok()) {
                int key;
                key = cvWaitKey(20);
                // Manage key and publish a index command
                ros::spinOnce();
            }
#else
            ros::spin();
#endif
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "probe_view", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    if (n.resolveName("probe") == "/probe") {
        ROS_WARN("probe_view: image has not been remapped! Typical command-line usage:\n"
                "\t$ ./probe_view probe:=<probe topic> [transport]");
    } 
    ROS_INFO("probe_view:\n"
            "\tUse left/right click to move the probe, middle click to reset.\n"
            "\tAdd ctrl to change image index.");

    ImageView view(n, (argc > 1) ? argv[1] : "raw");

    view.mainloop();


    return 0;
}
