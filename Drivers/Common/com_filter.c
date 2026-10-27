#include "com_filter.h"

float Com_LowPass_Filter(float alpha,float current_value,float last_value)
{
	float result = alpha * current_value + (1 - alpha) *  last_value;
	return result;
}



