#include "com_filter.h"

float Com_LowPass_Filter(float alpha, float current_value, float last_value)
{
	return alpha * current_value + (1.0f - alpha) * last_value;
}
