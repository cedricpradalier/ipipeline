
#include "ipipeline_modules/IPLOptFlowLK.h"

IPLOptFlowLK::IPLOptFlowLK(const char * n) : 
	IPLBinaryOperator(n)
{
}

bool IPLOptFlowLK::checkInput() const 
{
  IPLImageProcessor * ipA = getInput(inputA);
  IPLImageProcessor * ipB = getInput(inputB);
  if (!sameSizeAndType(ipA,ipB)) return false;
  return true;
}


bool IPLOptFlowLK::processInput()
{
  IPLImageProcessor * ipA = getInput(inputA);
  IPLImageProcessor * ipB = getInput(inputB);
  ImageProcessorOutput iA = ipA->getOutput();
  ImageProcessorOutput iB = ipB->getOutput();

  unsigned int i;
  outputVector.resize(iA.size());
  for (i=0;i<iA.size();i++) {
    
    cv::Mat res1(iA[i].size(),CV_32FC1);
    cv::Mat res2(iA[i].size(),CV_32FC1);
    outputVector[i].create(iA[i].size(), CV_32FC1);
    
    CvMat c_iA = iA[i];
    CvMat c_iB = iB[i];
    

    CvMat c_res1 = res1;
    CvMat c_res2 = res2; 
    cvCalcOpticalFlowPyrLK( &c_iB, &c_iA, cvSize(3,3), &c_res1, &c_res2) ;
    
    
    
    //~ int blocksize = 4;
    //~ int shiftsize = 2;
    //~ res1.create((c_iA.height - blocksize)/shiftsize, (c_iA.width - blocksize)/shiftsize, CV_32FC1);
    //~ res2.create((c_iA.height - blocksize)/shiftsize, (c_iA.width - blocksize)/shiftsize, CV_32FC1);
    //~ 
    //~ CvMat c_res1 = res1;
    //~ CvMat c_res2 = res2; 
    //~ cvCalcOpticalFlowBM( &c_iB, &c_iA, cvSize(blocksize,blocksize), cvSize(shiftsize,shiftsize) , cvSize(10,10), 1, &c_res1, &c_res2);
    cv::absdiff(res1,cv::Scalar(0.),res1);
    cv::absdiff(res2,cv::Scalar(0.),res2);
    cv::add(res1,res2,outputVector[i]);
  }
  return true;
}

