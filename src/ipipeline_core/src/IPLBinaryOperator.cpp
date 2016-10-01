
#include "ipipeline_core/IPLBinaryOperator.h"


IPLBinaryOperator::IPLBinaryOperator(const char * n) :
	IPLImageProcessor(n)
{
	inputA = addInput("InputA");
	inputB = addInput("InputB");
}


IPLBinaryOperator::~IPLBinaryOperator() 
{
}



