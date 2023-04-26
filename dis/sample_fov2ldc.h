/*
 * Copyright (C) Hisilicon Technologies Co., Ltd. 2019. All rights reserved.
 * Description : sample_fov2ldc.h
 * Author : ISP SW
 * Create : 2019-05-28
 * Version : Initial Draft
 */
#ifndef __HI_SAMPLE_FOV2LDC_H__
#define __HI_SAMPLE_FOV2LDC_H__

#include "hi_type.h"
#include "hi_comm_video.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#define FOV_PREC_BITS 20

typedef enum {
    fov_type_diagonal = 0,
    fov_type_horozontal,
    fov_type_vertical,
    fov_type_butt
} hi_fov_type;

typedef struct {
    hi_u32 width;
    hi_u32 height;
	hi_fov_type type;  /* 0--diagonal,1--horizontal,2--vertical */
	hi_u32      fov;   /*decimal bits 20bit*/
} hi_fov_attr;

hi_s32 hi_sample_fov_to_ldcv2(const hi_fov_attr *fov_attr, LDCV2_ATTR_S *ldcv2_attr);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of __cplusplus */

#endif /* __HI_SAMPLE_FOV2LDC_H__ */

