#include <SDL/SDL.h>
#include <SDL/SDL_image.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <ipipeline_io/ProbeCommand.h>
#include <ipipeline_io/ProbeStatus.h>

#include <boost/thread.hpp>


// void spawnNewWindow(const char * progname, const char * drivname, unsigned int stream_idx) 
// {
// 	char cmd[1024];
// 	sprintf(cmd,"%s %s %d &",progname,drivname, stream_idx);
// 	system(cmd);
// }
// 
SDL_Surface * createSurface(unsigned int w, unsigned int h)
{
    Uint32 rmask, gmask, bmask, amask;

    /* SDL interprets each pixel as a 32-bit number, so our masks must depend
       on the endianness (byte order) of the machine */
#if SDL_BYTEORDER == SDL_BIG_ENDIAN
    rmask = 0xff000000; gmask = 0x00ff0000; 
    bmask = 0x0000ff00; amask = 0x00000000; /* this value for alpha : 0x000000ff; */
#else
    rmask = 0x000000ff; gmask = 0x0000ff00; 
    bmask = 0x00ff0000; amask = 0x00000000; /* this value for alpha : 0xff000000; */
#endif
    SDL_Surface * tmp_surf = SDL_CreateRGBSurface(SDL_SWSURFACE,
            w,h, 32,rmask,gmask,bmask,amask);
    assert(tmp_surf != NULL);
    return tmp_surf;
}




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

        cv::Mat scaled_screen_img;
        unsigned int screen_width_, screen_height_;
        SDL_Surface * screen_surf_;
        bool new_image_, imgsize_changed_;
    public:
        ImageView(ros::NodeHandle& nh, const std::string& transport) 
        {
            boost::lock_guard<boost::mutex> guard(image_mutex_);
            topic_ = nh.resolveName("probe");
            ros::NodeHandle local_nh("~");
            local_nh.param("window_name", window_name_, topic_);

            width_ = 0;
            height_ = 0;
            screen_width_ = 300;
            screen_height_ = 300;
            screen_surf_ = createSurface(screen_width_,screen_height_);
            new_image_ = true;
            imgsize_changed_ = false;

            image_transport::ImageTransport it(nh);
            sub_ = it.subscribe(topic_+"/image", 1, &ImageView::image_cb, this, transport);
            status_sub_ = nh.subscribe(topic_+"/status", 1, &ImageView::status_cb, this);
            pub_ = nh.advertise<ipipeline_io::ProbeCommand>(topic_+"/command",1);
        }

        ~ImageView()
        {
        }

        bool image_is_ready() const {
            return new_image_;
        }

        bool resize_needed() const {
            return imgsize_changed_;
        }

        unsigned int getWidth() const {
            return width_;
        }

        unsigned int getHeight() const {
            return height_;
        }

        void set_screen_size(unsigned int w, unsigned int h) 
        {
            boost::lock_guard<boost::mutex> guard(image_mutex_);
            screen_width_ = w;
            screen_height_ = h;
			SDL_FreeSurface(screen_surf_);
            screen_surf_ = createSurface(screen_width_,screen_height_);
            new_image_ = true;
        }

        void set_ideal_screen_size() 
        {
            boost::lock_guard<boost::mutex> guard(image_mutex_);
            screen_width_ = width_;
            screen_height_ = height_;
			SDL_FreeSurface(screen_surf_);
            screen_surf_ = createSurface(screen_width_,screen_height_);
            new_image_ = true;
            imgsize_changed_ = false;
        }

        void blit(SDL_Surface * dest) 
        {
            boost::lock_guard<boost::mutex> guard(image_mutex_);
            if (SDL_MUSTLOCK(dest)) { SDL_LockSurface(dest); }
			SDL_BlitSurface(screen_surf_,NULL,dest,NULL);
            if (SDL_MUSTLOCK(dest)) { SDL_UnlockSurface(dest); }
            new_image_ = false;
        }

        void status_cb(const ipipeline_io::ProbeStatusConstPtr& msg)
        {
            status_ = *msg;
        }

        void publish(const ipipeline_io::ProbeCommand & command) {
            pub_.publish(command);
        }

        int getIndex() {
            return status_.index;
        }

        void image_cb(const sensor_msgs::ImageConstPtr& msg)
        {
            // ROS_INFO("Image CB Locking");
            boost::lock_guard<boost::mutex> guard(image_mutex_);
            // ROS_INFO("Image CB Locked");

            // Hang on to message pointer for sake of mouse_cb
            last_msg_ = msg;

            // May want to view raw bayer data
            // NB: This is hacky, but should be OK since we have only one image CB.
            if (msg->encoding.find("bayer") != std::string::npos)
                boost::const_pointer_cast<sensor_msgs::Image>(msg)->encoding = "mono8";


            cv::Mat img = cv_bridge::toCvShare(msg, "bgra8")->image;
            if (img.total()) {
                imgsize_changed_ = imgsize_changed_ || 
                    (width_ != msg->width) || (height_ != msg->height);
                width_ = msg->width;
                height_ = msg->height;
                cv::Mat cvsurf(screen_height_,screen_width_,CV_8UC4, 
                            screen_surf_->pixels, screen_surf_->pitch);
                cv::resize(img,cvsurf, cv::Size(screen_width_,screen_height_));
                new_image_ = true;
            } else {
                ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
            }

            char new_title[256];
            snprintf(new_title,256,"%s: %s[%d] %dx%d",topic_.c_str(),
                    status_.input.c_str(),status_.index, width_, height_);
            if (window_title_ != new_title) {
                SDL_WM_SetCaption(new_title, topic_.c_str());
                window_title_ = new_title;
            }
            // ROS_INFO("Image CB completed");
        }
};




int main( int argc, char *argv[] )
{
    ros::init(argc, argv, "probe_view", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    if (n.resolveName("probe") == "/probe") {
        ROS_WARN("probe_view: image has not been remapped! Typical command-line usage:\n"
                "\t$ ./probe_view probe:=<probe topic> [transport]");
    } 
    ROS_INFO("probe_view:\n"
            "\tUse left/right click to move the probe, press r to reset.\n"
            "\tRoll to change image index.");

	

	if ( SDL_Init(SDL_INIT_VIDEO) < 0 ) {
		exit(3);
	}
    
    ImageView view(n, (argc > 1) ? argv[1] : "raw");

	SDL_Event event;
	SDL_Surface * screen = SDL_SetVideoMode(300,300, 32, SDL_ANYFORMAT | SDL_VIDEORESIZE);
	assert(screen != NULL);


	while (ros::ok()) {
        ros::spinOnce();

		if (view.resize_needed()) {
			SDL_FreeSurface(screen);
			screen = SDL_SetVideoMode(std::max(view.getWidth(),100u),std::max(view.getHeight(),100u),
					32, SDL_ANYFORMAT|SDL_VIDEORESIZE);	
            view.set_ideal_screen_size();
		}
		if (view.image_is_ready()) {
			if (SDL_MUSTLOCK(screen)) { SDL_LockSurface(screen); }
			Uint32 bg = SDL_MapRGB(screen->format,0x00,0x00,0x40);
			SDL_FillRect(screen,NULL,bg);


			if (SDL_MUSTLOCK(screen)) { SDL_UnlockSurface(screen); }

            view.blit(screen);

			SDL_Flip(screen);
		} else {
			SDL_Delay(20);
		}

		// wait click
		while (SDL_PollEvent(&event) > 0) {
            ipipeline_io::ProbeCommand command;
            command.index = 0;
            command.action = ipipeline_io::ProbeCommand::Probe_Noop;
			switch (event.type) {
				case SDL_VIDEORESIZE :
					SDL_FreeSurface(screen);
					screen = SDL_SetVideoMode(std::max(event.resize.w,100), std::max(event.resize.h,100), 
							32, SDL_ANYFORMAT|SDL_VIDEORESIZE);	
                    view.set_screen_size(event.resize.w,event.resize.h);
					break;
				case SDL_MOUSEBUTTONUP: 
                    switch (event.button.button) {
                        case 1:
                            command.action = ipipeline_io::ProbeCommand::Probe_ToNext;
                            view.publish(command);
                            break;
                        case 3:
                            command.action = ipipeline_io::ProbeCommand::Probe_ToPrev;
                            view.publish(command);
                            break;
						case 4 :
                            command.action = ipipeline_io::ProbeCommand::Probe_SetIndex;
                            command.index = view.getIndex() - 1;
                            view.publish(command);
							break;
						case 5 :
                            command.action = ipipeline_io::ProbeCommand::Probe_SetIndex;
                            command.index = view.getIndex() + 1;
                            view.publish(command);
							break;
                        default:
                            break;
                    }
					break;
				case SDL_KEYDOWN :
					switch (event.key.keysym.sym) {
						case SDLK_q :
						case SDLK_ESCAPE :
                            ros::shutdown();
							break;
						case SDLK_F2:
							// saveImage(screen_surf,"sdlprobe", currentframe,NULL);
							break;
						case SDLK_UP :
                            command.action = ipipeline_io::ProbeCommand::Probe_ToNext;
                            view.publish(command);
							break;
						case SDLK_DOWN :
                            command.action = ipipeline_io::ProbeCommand::Probe_ToPrev;
                            view.publish(command);
							break;
						case SDLK_r :
                            command.action = ipipeline_io::ProbeCommand::Probe_Reset;
                            view.publish(command);
							break;
						case SDLK_m :
							// saveImageMatrix(buffers,stream_idx,screen_img);
							break;
						case SDLK_n :
							// spawnNewWindow(argv[0],argv[1],
							// 		(stream_idx+1)%IPLSharedBuffers::MAX_EXPORTED_STREAM);
							break;
						case SDLK_LEFT :
                            command.action = ipipeline_io::ProbeCommand::Probe_SetIndex;
                            command.index = view.getIndex() - 1;
                            view.publish(command);
							break;
						case SDLK_RIGHT :
                            command.action = ipipeline_io::ProbeCommand::Probe_SetIndex;
                            command.index = view.getIndex() + 1;
                            view.publish(command);
							break;
						default : 
							break;
					}
					break;
				case SDL_KEYUP :
					break;
				case SDL_QUIT :
                    ros::shutdown();
					break;
				default :
					break;
			}
		}
	}

	SDL_FreeSurface(screen);
	return 0;
}
