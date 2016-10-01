#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPLLocalMax.h"

IPLLocalMax::IPLLocalMax(const char * n) : 
	IPLImageProcessor(n)
{
    maskInput = addInput("Mask");
    imageInput = addInput("Image");
}

bool IPLLocalMax::checkInput() const 
{
	IPLImageProcessor * inputIP = getInput(imageInput);
    IPLImageProcessor * maskIP = getInput(maskInput);
    ImageProcessorOutput mask = maskIP->getOutput();
	ImageProcessorOutput in = inputIP->getOutput();
    if (!sameSize(inputIP,maskIP)) {
        return error(INVALID_IMAGE_SIZE);
    }
    for (unsigned int i=0;i<in.size();i++) {
        // Only work for single channel
        if (in[i].channels() != 1) {
            return error(INVALID_TYPE);
        }
        switch (in[i].type()) {
            case CV_8UC1: 
            case CV_8SC1: 
            case CV_16UC1:
            case CV_16SC1:
            case CV_32SC1:
            case CV_32FC1:
                break;
            default:
                return error(INVALID_TYPE);
        }
        if (mask[i].type() != CV_8UC1) {
            return error(INVALID_TYPE);
        }
    }
	return true;
}

#define MAT(u,v) (mk[i].at<unsigned char>(u,v))

template <class T>
struct ValueOrder {
    const std::vector< std::pair<unsigned int,unsigned int> > & coord;
    const cv::Mat & I ;
    ValueOrder(const std::vector< std::pair<unsigned int,unsigned int> > & coord_,
        const cv::Mat & I_) : coord(coord_), I(I_) {}
    bool operator()(const unsigned int & i, const unsigned int & j) {
        return I.at<T>(coord[i].second,coord[i].first) > I.at<T>(coord[j].second,coord[j].first);
    }
};

template <class T>
void selectMaxima(const cv::Mat & I,const cv::Mat & M,
        std::vector<unsigned int> & indices,
        std::vector< std::pair<unsigned int,unsigned int> > & coord) {
    unsigned int width = I.size().width;
    unsigned int height = I.size().height;
    for (unsigned int u = 1;u<height-1; u++) {
        for (unsigned int v = 1;v<width-1; v++) {
            if (!M.at<unsigned char>(u,v)) continue;
            if (I.at<T>(u,v) < I.at<T>(u,v+1)) continue;
            if (I.at<T>(u,v) < I.at<T>(u,v-1)) continue;
            if (I.at<T>(u,v) < I.at<T>(u+1,v)) continue;
            if (I.at<T>(u,v) < I.at<T>(u-1,v)) continue;
            coord.push_back(std::pair<unsigned int,unsigned int>(v,u));
            indices.push_back(indices.size());
        }
    }
}

#define PROCESS(T) { \
    ValueOrder<T> vo(coord,in[i]);                \
    selectMaxima<T>(in[i],mk,indices,coord);   \
    std::sort(indices.begin(),indices.end(), vo); \
}

bool IPLLocalMax::processInput()
{
	IPLImageProcessor * inputIP = getInput(imageInput);
    IPLImageProcessor * maskIP = getInput(maskInput);
    ImageProcessorOutput mask = maskIP->getOutput();
	ImageProcessorOutput in = inputIP->getOutput();
    outputVector.resize(in.size());
    unsigned int i,k;
    for (i=0;i<in.size();i++) {
        cv::Mat mk = mask[i].clone();
        std::vector<unsigned int> indices;
        std::vector< std::pair<unsigned int,unsigned int> > coord, selected;
        switch (in[i].type()) {
            case CV_8UC1: PROCESS(uint8_t); break;
            case CV_8SC1: PROCESS(int8_t); break;
            case CV_16UC1: PROCESS(uint16_t); break;
            case CV_16SC1: PROCESS(int16_t); break;
            case CV_32SC1: PROCESS(int32_t); break;
            case CV_32FC1: PROCESS(float); break;
            default: break;
        }
#ifdef DEBUG_LM
        printf("%d indices considered\n",indices.size());
#endif
        for (k=0;k<indices.size();k++) {
#ifdef DEBUG_LM
            printf("Considering %d %d %d %d\n",coord[indices[k]].second,coord[indices[k]].first,
                in[i].at<short>(coord[indices[k]].second,coord[indices[k]].first),
                mk.at<unsigned char>(coord[indices[k]].second,coord[indices[k]].first));
#endif
            if (mk.at<unsigned char>(coord[indices[k]].second,coord[indices[k]].first)) {
#ifdef DEBUG_LM
                printf("Accepted %d %d\n",coord[indices[k]].second,coord[indices[k]].first);
#endif
                selected.push_back(coord[indices[k]]);
                cv::floodFill(mk,cv::Point(coord[indices[k]].first,coord[indices[k]].second),
                        cv::Scalar(0),NULL,cv::Scalar(5),cv::Scalar(5),8+cv::FLOODFILL_FIXED_RANGE);
#ifdef DEBUG_LM
                mk.at<unsigned char>(coord[indices[k]].second,coord[indices[k]].first) = 128;
                cv::imwrite("mask.png",mk);
                getchar();
#endif
            }
        }
#ifdef DEBUG_LM
        printf("%d indices selected\n",selected.size());
#endif
        outputVector[i].create(cv::Size(2,selected.size()),CV_32FC1);
        for (k=0;k<selected.size();k++) {
            outputVector[i].at<float>(k,0) =selected[k].first;
            outputVector[i].at<float>(k,1) =selected[k].second;
        }
    }
	return true;
}
#undef MAT


