#include <math.h>

#include "ipipeline_core/IPLCheckFloat.h"

// #define ID_DEBUG

IPLCheckFloat::IPLCheckFloat(const char * n) : 
	IPLImageProcessor(n)
{
	deftInput = addInput("Default");
}


bool IPLCheckFloat::checkInput() const 
{
    unsigned int i;
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
    for (i=0;i<in.size();i++) {
        switch (in[i].type()) {
            case CV_32FC1:
            case CV_32FC2:
            case CV_32FC3:
            case CV_64FC1:
            case CV_64FC2:
            case CV_64FC3:
                break;
            default : 
                return error(INVALID_TYPE);
        }
	}
	return true;
}


bool IPLCheckFloat::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	ImageProcessorOutput in = input->getOutput();
	unsigned int i;
    int u,v;

    for (i=0;i<in.size();i++) {
        switch (in[i].type()) {
            case CV_32FC1:
                {
                    const cv::Mat1f & m = (const cv::Mat1f &) in[i];
                    for (v=0;v<m.rows;v++) {
                        for (u=0;u<m.cols;u++) {
                            assert(!isnan(m(u,v)));
                            assert(!isinf(m(u,v)));
                        }
                    }
                    break;
                }
            case CV_32FC2:
                {
                    const cv::Mat2f & m = (const cv::Mat2f &) in[i];
                    for (v=0;v<m.rows;v++) {
                        for (u=0;u<m.cols;u++) {
                            assert(!isnan(m(u,v)[0]));
                            assert(!isinf(m(u,v)[0]));
                            assert(!isnan(m(u,v)[1]));
                            assert(!isinf(m(u,v)[1]));
                        }
                    }
                    break;
                }
            case CV_32FC3:
                {
                    const cv::Mat3f & m = (const cv::Mat3f &) in[i];
                    for (v=0;v<m.rows;v++) {
                        for (u=0;u<m.cols;u++) {
                            assert(!isnan(m(u,v)[0]));
                            assert(!isinf(m(u,v)[0]));
                            assert(!isnan(m(u,v)[1]));
                            assert(!isinf(m(u,v)[1]));
                            assert(!isnan(m(u,v)[2]));
                            assert(!isinf(m(u,v)[2]));
                        }
                    }
                    break;
                }
            case CV_64FC1:
                {
                    const cv::Mat1d & m = (const cv::Mat1d &) in[i];
                    for (v=0;v<m.rows;v++) {
                        for (u=0;u<m.cols;u++) {
                            assert(!isnan(m(u,v)));
                            assert(!isinf(m(u,v)));
                        }
                    }
                    break;
                }
            case CV_64FC2:
                {
                    const cv::Mat2d & m = (const cv::Mat2d &) in[i];
                    for (v=0;v<m.rows;v++) {
                        for (u=0;u<m.cols;u++) {
                            assert(!isnan(m(u,v)[0]));
                            assert(!isinf(m(u,v)[0]));
                            assert(!isnan(m(u,v)[1]));
                            assert(!isinf(m(u,v)[1]));
                        }
                    }
                    break;
                }
            case CV_64FC3:
                {
                    const cv::Mat3d & m = (const cv::Mat3d &) in[i];
                    for (v=0;v<m.rows;v++) {
                        for (u=0;u<m.cols;u++) {
                            assert(!isnan(m(u,v)[0]));
                            assert(!isinf(m(u,v)[0]));
                            assert(!isnan(m(u,v)[1]));
                            assert(!isinf(m(u,v)[1]));
                            assert(!isnan(m(u,v)[2]));
                            assert(!isinf(m(u,v)[2]));
                        }
                    }
                    break;
                }
                break;
            default : 
                return error(INVALID_TYPE);
        }
	}

	for (i=0;i<in.size();i++) {
        const cv::Mat1f & m = (const cv::Mat1f &) in[i];
		for (v=0;v<m.rows;v++) {
			for (u=0;u<m.cols;u++) {
				assert(!isnan(m(u,v)));
				assert(!isinf(m(u,v)));
			}
		}
	}

	return true;
}

