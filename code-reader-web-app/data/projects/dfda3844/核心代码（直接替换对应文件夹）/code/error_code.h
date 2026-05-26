/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2025,逐飞科技
 * All rights reserved.
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       error_code
 * @brief      统一错误码定义头文件
 * @author     GLM-5
 * @date       2025-01-01
 * @version    1.0
 ********************************************************************************************************************/

#ifndef __ERROR_CODE_H__
#define __ERROR_CODE_H__

typedef enum {
    RET_OK = 0,
    RET_ERROR_INVALID_PARAM,
    RET_ERROR_NULL_POINTER,
    RET_ERROR_TIMEOUT,
    RET_ERROR_BUSY,
    RET_ERROR_BUFFER_OVERFLOW,
    RET_ERROR_NOT_INITIALIZED,
    RET_ERROR_OUT_OF_RANGE,
    RET_ERROR_FLASH_WRITE,
    RET_ERROR_FLASH_READ,
    RET_ERROR_GPS_NO_SIGNAL,
    RET_ERROR_IMU_NOT_READY,
    RET_ERROR_UNKNOWN
} RetCode_e;

#define ASSERT_PTR(ptr)         ((ptr) != NULL)
#define ASSERT_RANGE(val, min, max) ((val) >= (min) && (val) <= (max))

#endif
