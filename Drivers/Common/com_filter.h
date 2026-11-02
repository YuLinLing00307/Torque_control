#ifndef __COM_FILTER_H
#define __COM_FILTER_H

#include "main.h"

/* 一阶低通：alpha 越大越跟手，越小越平滑（0 < alpha <= 1） */
float Com_LowPass_Filter(float alpha, float current_value, float last_value);

#endif
