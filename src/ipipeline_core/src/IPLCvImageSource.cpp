#define X_DISPLAY_MISSING
#include <Imlib2.h>

#include "ipipeline_core/IPLCvImageSource.h"

IPLCvImageSource::~IPLCvImageSource()
{
}

IPLCvImageSource::IPLCvImageSource(const char * name) :
	IPLImageSource(name, IPL_ROLE_SOURCE)
{
	outputVector.resize(1);
}

IPLCvImageSource::IPLCvImageSource(const char * name, 
		const cv::Mat & I) :
	IPLImageSource(name,IPL_ROLE_SOURCE,I.type())
{
	outputVector.resize(1);
    outputVector[0] = I;
}


IPLCvImageSource::IPLCvImageSource(const char * name, 
		const std::string & fname) :
	IPLImageSource(name,IPL_ROLE_SOURCE)
{
	if (!loadSource(fname)) {
#ifndef NDEBUG
	    bool loadSourceSucceeded = false;
#endif
	    assert(loadSourceSucceeded);
	}
	   
}

bool IPLCvImageSource::loadSource(const std::string & fname)
{
	Imlib_Image image;
	Imlib_Load_Error err;
	//printf("Loading %s\n",fname);
	image = imlib_load_image_with_error_return(fname.c_str(),&err);
	//printf("image = %p\n",image);
	if (image == NULL)  {
	    switch (err) {
		case IMLIB_LOAD_ERROR_FILE_DOES_NOT_EXIST:
		    return error("Error loading image '%s' - fname does not exist", fname.c_str());
		    break;
		case IMLIB_LOAD_ERROR_FILE_IS_DIRECTORY:
		    return error("Error loading image '%s' - Directory specified for image filename",
			    fname.c_str());
		    break;
		case IMLIB_LOAD_ERROR_PERMISSION_DENIED_TO_READ:
		    return error("Error loading image '%s' - No read access to directory", fname.c_str());
		    break;
		case IMLIB_LOAD_ERROR_UNKNOWN:
		case IMLIB_LOAD_ERROR_NO_LOADER_FOR_FILE_FORMAT:
		    return error("Error loading image '%s' - No Imlib2 loader for that fname format",
			    fname.c_str());
		    break;
		case IMLIB_LOAD_ERROR_PATH_TOO_LONG:
		    return error("Error loading image '%s' - Path specified is too long", fname.c_str());
		    break;
		case IMLIB_LOAD_ERROR_PATH_COMPONENT_NON_EXISTANT:
		    return error("Error loading image '%s' - Path component does not exist", fname.c_str());
		    break;
		case IMLIB_LOAD_ERROR_PATH_COMPONENT_NOT_DIRECTORY:
		    return error("Error loading image '%s' - Path component is not a directory",
			    fname.c_str());
		    break;
		case IMLIB_LOAD_ERROR_PATH_POINTS_OUTSIDE_ADDRESS_SPACE:
		    return error("Error loading image '%s' - Path points outside address space",
			    fname.c_str());
		    break;
		case IMLIB_LOAD_ERROR_TOO_MANY_SYMBOLIC_LINKS:
		    return error("Error loading image '%s' - Too many levels of symbolic links",
			    fname.c_str());
		    break;
		case IMLIB_LOAD_ERROR_OUT_OF_MEMORY:
		    return error("Error loading image '%s' - Out of memory", fname.c_str());
		    break;
		case IMLIB_LOAD_ERROR_OUT_OF_FILE_DESCRIPTORS:
		    return error("While loading '%s' - Out of fname descriptors", fname.c_str());
		    break;
		case IMLIB_LOAD_ERROR_PERMISSION_DENIED_TO_WRITE:
		    return error("Error loading image '%s' - Cannot write to directory", fname.c_str());
		    break;
		case IMLIB_LOAD_ERROR_OUT_OF_DISK_SPACE:
		    return error("Error loading image '%s' - Cannot write - out of disk space", fname.c_str());
		    break;
		default:
		    return error
			("Error loading image '%s' - Unknown error (%d). Attempting to continue",
			 fname.c_str(), err);
		    break;
	    }
	}
	imlib_context_set_image(image);
	setSourceInfo(CV_8UC3,1,
			imlib_image_get_width(),
			imlib_image_get_height());
	Imlib_Color color;
	int u,v;
	//printf("Image %d, %d \n",width,height);
	for (v=0;v<outputVector[0].rows;v++) {
		for (u=0;u<outputVector[0].cols;u++) {
			imlib_image_query_pixel(u,v,&color);
            cv::Vec3b & P = outputVector[0].at<cv::Vec3b>(v,u);
			P[2] = color.red;
			P[1] = color.green;
			P[0] = color.blue;
		}
	}
	imlib_free_image();
	return true;
}


bool IPLCvImageSource::processInput()
{
	// this one is always ready, since it only output its image from
	// memory
	return true;
}


