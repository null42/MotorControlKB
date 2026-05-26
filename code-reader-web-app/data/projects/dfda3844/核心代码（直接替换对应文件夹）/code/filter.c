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

#include "filter.h"

/**
 * @brief       IIR二阶低通滤波
 * @param       lpf         IIR滤波器结构体指针
 * @param       rawData     原始数据
 * @return      void
 */
void LPF_IIR2nd(IIR_2nd* lpf, float rawData)
{
    uint8 i = 0;
    for (i = IIR_ORDER; i > 0; i--)
    {
        lpf->OriginData[i] = lpf->OriginData[i - 1];
        lpf->FilterData[i] = lpf->FilterData[i - 1];
    }
    lpf->OriginData[0] = rawData;
    lpf->FilterData[0] = lpf->B[0] * lpf->OriginData[0];
    for (i = 1; i <= IIR_ORDER; i++)
    {
        lpf->FilterData[0] = lpf->FilterData[0] + lpf->B[i] * lpf->OriginData[i] - lpf->A[i] * lpf->FilterData[i];
    }
}

RingBuffer speed_filter;

/**
 * @brief       判断环形队列是否已满
 * @param       rb          环形队列结构体指针
 * @return      true-已满，false-未满
 */
static bool isFull(const RingBuffer* rb)
{
    return rb->size == MAX_SIZE;
}

/**
 * @brief       判断环形队列是否为空
 * @param       rb          环形队列结构体指针
 * @return      true-为空，false-不为空
 */
static bool isEmpty(const RingBuffer* rb)
{
    return rb->size == 0;
}

/**
 * @brief       初始化环形队列
 * @param       rb          环形队列结构体指针
 * @return      void
 */
void initRingBuffer(RingBuffer* rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->size = 0;
    rb->sum = 0;
    memset(rb->buffer, 0, sizeof(rb->buffer));
}

/**
 * @brief       滑动均值滤波
 * @param       rb          环形队列结构体指针
 * @param       data        输入数据
 * @return      滤波后的数据
 */
int16 MAF_With_RB(RingBuffer* rb, int16 data)
{
    if (isFull(rb))
    {
        rb->sum -= rb->buffer[rb->head];
        rb->head = (rb->head + 1) % MAX_SIZE;
        rb->size--;
    }
    rb->buffer[rb->tail] = data;
    rb->sum += data;
    rb->tail = (rb->tail + 1) % MAX_SIZE;
    rb->size++;

    return rb->sum / rb->size;
}

/**
 * @brief       初始化加权环形队列
 * @param       rb              环形队列结构体指针
 * @param       customWeights   自定义权重数组，长度为MAX_SIZE
 * @return      void
 * @note        例如：{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1} 即为普通均值
 *              或者：{1, 2, 3, 4, 5, 6, 7, 8, 9, 10} 越新的数据权重越大
 */
void initRingBuffer_WMAF(RingBuffer* rb, const float* customWeights)
{
    rb->head = 0;
    rb->tail = 0;
    rb->size = 0;
    rb->weightSum = 0;
    memset(rb->buffer, 0, sizeof(rb->buffer));

    for (int i = 0; i < MAX_SIZE; i++)
    {
        rb->weights[i] = customWeights[i];
    }
}

/**
 * @brief       滑动加权均值滤波
 * @param       rb          环形队列结构体指针
 * @param       data        输入数据
 * @return      滤波后的数据
 */
int16 WMAF_With_RB(RingBuffer* rb, int16 data)
{
    if (rb->size == MAX_SIZE)
    {
        rb->head = (rb->head + 1) % MAX_SIZE;
        rb->size--;
    }

    rb->buffer[rb->tail] = data;
    rb->tail = (rb->tail + 1) % MAX_SIZE;
    rb->size++;

    float WeightedSum = 0;
    float currentWeightTotal = 0;
    int index = rb->head;

    for (int i = 0; i < rb->size; i++)
    {
        float w = rb->weights[i];
        WeightedSum += rb->buffer[index] * w;
        currentWeightTotal += w;
        index = (index + 1) % MAX_SIZE;
    }

    if (currentWeightTotal == 0) return data;
    return (int16)(WeightedSum / currentWeightTotal);
}
