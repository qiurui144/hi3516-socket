/*
 * Copyright (C) Hisilicon Technologies Co., Ltd. 2019. All rights reserved.
 * Description : sample_gyro_dis.h
 * Author : ISP SW
 * Create : 2019-06-03
 * Version : Initial Draft
 */

#ifndef __HI_SAMPLE_GYRO_DIS_H__
#define __HI_SAMPLE_GYRO_DIS_H__

#include "hi_type.h"
#include "hi_comm_vo.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

extern HI_BOOL g_bGyroStarted;

HI_S32 SAMPLE_DIS_IPC_GYRO(HI_VOID);

HI_S32 SAMPLE_DIS_DV_GYRO(HI_VOID);

HI_S32 SAMPLE_MOTIONSENSOR_Stop(HI_VOID);

HI_S32 SAMPLE_MOTIONSENSOR_DeInit(HI_VOID);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of __cplusplus */

#endif /* __HI_SAMPLE_GYRO_DIS_H__ */

