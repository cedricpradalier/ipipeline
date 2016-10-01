#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>

#include "ipipeline_modules/IPLDrawCross.h"

IPLDrawCross::IPLDrawCross(const char * n, cv::Scalar col, unsigned int sz, unsigned int thick) : 
	IPLImageProcessor(n), color(col), scale(sz), thickness(thick)
{
    coordInput = addInput("Coord");
    imageInput = addInput("Image");
}

bool IPLDrawCross::checkInput() const 
{
	IPLImageProcessor * inputIP = getInput(imageInput);
    IPLImageProcessor * coordIP = getInput(coordInput);
    ImageProcessorOutput coord = coordIP->getOutput();
	ImageProcessorOutput in = inputIP->getOutput();
    for (unsigned int i=0;i<in.size();i++) {
        switch (in[i].type()) {
            case CV_8UC3: 
                break;
            default:
                return error(INVALID_TYPE);
        }
        if (coord[i].type() != CV_32FC1) {
            return error(INVALID_TYPE);
        }
        if ((coord[i].size().width < 2) || (coord[i].size().width > 3)){
            printf("Coord size: %d %d\n",coord[i].size().width,coord[i].size().height);
            return error(INVALID_IMAGE_SIZE);
        }
    }
	return true;
}

bool IPLDrawCross::processInput()
{
	IPLImageProcessor * inputIP = getInput(imageInput);
    IPLImageProcessor * coordIP = getInput(coordInput);
    ImageProcessorOutput coord = coordIP->getOutput();
	ImageProcessorOutput in = inputIP->getOutput();
    outputVector.resize(in.size());
    unsigned int i,k;
    for (i=0;i<in.size();i++) {
        outputVector[i] = in[i].clone();
        // printf("%s: %d crosses to draw (%d)\n",getName(),coord[i].size().height,coord[i].size().width);
        for (k=0;(signed)k<coord[i].size().height;k++) {
            switch (coord[i].size().width) {
                case 2:
                    {
                        // printf("Drawing cross %d at %.2f %.2f\n",k, coord[i].at<float>(k,0),coord[i].at<float>(k,1));
                        cv::Point V0 = cv::Point(coord[i].at<float>(k,0),coord[i].at<float>(k,1)-scale);
                        cv::Point V1 = cv::Point(coord[i].at<float>(k,0),coord[i].at<float>(k,1)+scale);
                        cv::Point H0 = cv::Point(coord[i].at<float>(k,0)-scale,coord[i].at<float>(k,1));
                        cv::Point H1 = cv::Point(coord[i].at<float>(k,0)+scale,coord[i].at<float>(k,1));
                        cv::line(outputVector[i],V0,V1,color, thickness);
                        cv::line(outputVector[i],H0,H1,color, thickness);
                    }
                    break;
                case 3:
                    {
                        // printf("Drawing cross %d at %.2f %.2f s %.2f\n",k, coord[i].at<float>(k,0),coord[i].at<float>(k,1),coord[i].at<float>(k,2));
                        cv::Point V0 = cv::Point(coord[i].at<float>(k,0),coord[i].at<float>(k,1)-coord[i].at<float>(k,2));
                        cv::Point V1 = cv::Point(coord[i].at<float>(k,0),coord[i].at<float>(k,1)+coord[i].at<float>(k,2));
                        cv::Point H0 = cv::Point(coord[i].at<float>(k,0)-coord[i].at<float>(k,2),coord[i].at<float>(k,1));
                        cv::Point H1 = cv::Point(coord[i].at<float>(k,0)+coord[i].at<float>(k,2),coord[i].at<float>(k,1));
                        cv::line(outputVector[i],V0,V1,color, thickness);
                        cv::line(outputVector[i],H0,H1,color, thickness);
                        cv::line(outputVector[i],H0,V1,color, thickness);
                        cv::line(outputVector[i],V1,H1,color, thickness);
                        cv::line(outputVector[i],H1,V0,color, thickness);
                        cv::line(outputVector[i],V0,H0,color, thickness);
                    }
                    break;
            }
        }
    }
    // getchar();
	return true;
}
#undef MAT


