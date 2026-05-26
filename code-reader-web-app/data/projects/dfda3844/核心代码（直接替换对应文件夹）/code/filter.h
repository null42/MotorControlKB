/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2025,逐飞科技
 * All rights reserved.
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file        filter
 * @brief       滤波器模块，包含IIR低通滤波、滑动均值滤波、加权滑动均值滤波
 * @author      GLM-5
 * @date        2025-03-08
 ********************************************************************************************************************/

#ifndef __FILTER_H__
#define __FILTER_H__

#include "zf_common_headfile.h"

#define IIR_ORDER 2

typedef struct
{
    float A[IIR_ORDER + 1];
    float B[IIR_ORDER + 1];
    float OriginData[IIR_ORDER + 1];
    float FilterData[IIR_ORDER + 1];
} IIR_2nd;

#define MAX_SIZE 10

typedef struct
{
    float weights[MAX_SIZE];
    int buffer[MAX_SIZE];
    float weightSum;
    int sum;
    int head;
    int tail;
    int size;
} RingBuffer;

extern RingBuffer speed_filter;

void LPF_IIR2nd(IIR_2nd* lpf, float rawData);
void initRingBuffer(RingBuffer* rb);
int16 MAF_With_RB(RingBuffer* rb, int16 data);
void initRingBuffer_WMAF(RingBuffer* rb, const float* customWeights);
int16 WMAF_With_RB(RingBuffer* rb, int16 data);

#endif
