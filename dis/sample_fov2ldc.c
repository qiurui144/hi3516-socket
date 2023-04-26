/*
 * Copyright (C) Hisilicon Technologies Co., Ltd. 2019. All rights reserved.
 * Description : sample_fov2ldc.c
 * Author : ISP SW
 * Create : 2019-05-28
 * Version : Initial Draft
 */

#include "sample_fov2ldc.h"

#include <math.h>
#include <stdio.h>

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* #ifdef __cplusplus */

#define PI   3.141593
#define MAX_OF_FOV  90

hi_s32 hi_sample_fov_to_ldcv2(const hi_fov_attr *fov_attr, LDCV2_ATTR_S *ldcv2_attr)
{
    hi_s32 ret = HI_SUCCESS;

    const hi_s32 dist_coef_prec = 100000;
    hi_float fov_horizontal = 0.0;
    hi_float fov_vertical = 0.0;
    hi_float fov = (hi_float)fov_attr->fov / (hi_float)(1 << FOV_PREC_BITS);

    if ((fov_attr == HI_NULL) || (ldcv2_attr == HI_NULL))
    {
        return HI_FAILURE;
    }

    memset(ldcv2_attr, 0, sizeof(LDCV2_ATTR_S));

    switch (fov_attr->type)
    {
        case 0:     /* input diagonal field of view angle */
            fov_horizontal = fov * fov_attr->width / sqrt(fov_attr->width * fov_attr->width + fov_attr->height * fov_attr->height);
            fov_vertical   = fov * fov_attr->height / sqrt(fov_attr->width * fov_attr->width + fov_attr->height * fov_attr->height);
            break;
        case 1:     /* input horizontal field of view angle */
            fov_horizontal = fov;
            fov_vertical   = fov * fov_attr->height / fov_attr->width;
            break;
        case 2:     /* input vertical field of view angle */
            fov_horizontal = fov * fov_attr->width / fov_attr->height;
            fov_vertical   = fov;
            break;
        default:
            break;
    }

    ldcv2_attr->s32FocalLenX = (HI_S32)(round((fov_attr->width / 2) / tan(fov_horizontal * PI / 180 / 2) * 100));
    ldcv2_attr->s32FocalLenY = (HI_S32)(round((fov_attr->height / 2) / tan(fov_vertical * PI / 180 / 2) * 100));

    ldcv2_attr->s32CoorShiftX = (HI_S32)(round(fov_attr->width / 2 * 100));
    ldcv2_attr->s32CoorShiftY = (HI_S32)(round(fov_attr->height / 2 * 100));

    ldcv2_attr->as32DstJunPt[0] = 800000;
    ldcv2_attr->as32DstJunPt[1] = 800000;
    ldcv2_attr->s32SrcJunPt = 800000;

    ldcv2_attr->as32SrcCaliRatio[0][0] = 1 * dist_coef_prec;
    ldcv2_attr->as32SrcCaliRatio[0][1] = 0;
    ldcv2_attr->as32SrcCaliRatio[0][2] = 0;
    ldcv2_attr->as32SrcCaliRatio[0][3] = 0;
    ldcv2_attr->as32SrcCaliRatio[1][0] = 0;
    ldcv2_attr->as32SrcCaliRatio[1][1] = 0;
    ldcv2_attr->as32SrcCaliRatio[1][2] = 0;
    ldcv2_attr->as32SrcCaliRatio[1][3] = 0;

    ldcv2_attr->as32DstCaliRatio[0][0] = 1 * dist_coef_prec;
    ldcv2_attr->as32DstCaliRatio[0][1] = 0;
    ldcv2_attr->as32DstCaliRatio[0][2] = 0;
    ldcv2_attr->as32DstCaliRatio[0][3] = 0;
    ldcv2_attr->as32DstCaliRatio[1][0] = 0;
    ldcv2_attr->as32DstCaliRatio[1][1] = 0;
    ldcv2_attr->as32DstCaliRatio[1][2] = 0;
    ldcv2_attr->as32DstCaliRatio[1][3] = 0;
    ldcv2_attr->as32DstCaliRatio[2][0] = 0;
    ldcv2_attr->as32DstCaliRatio[2][1] = 0;
    ldcv2_attr->as32DstCaliRatio[2][2] = 0;
    ldcv2_attr->as32DstCaliRatio[2][3] = 0;
    ldcv2_attr->s32MaxDu = 1048576;     /* max value */

    printf("\nInput:");
    printf("\n\tw=%d,h=%d,fovtype=%d (%s),fov=%f",
           fov_attr->width, fov_attr->height, fov_attr->type,
           (fov_attr->type == 0) ? ("diagonal") : ((fov_attr->type == 1) ? ("horizontal") : ("vertical")), fov);
    printf("\n\nOutput:");
    printf("\n\tFocalLen:%d,%d", ldcv2_attr->s32FocalLenX, ldcv2_attr->s32FocalLenY);
    printf("\n\tCoorShift:%d,%d", ldcv2_attr->s32CoorShiftX, ldcv2_attr->s32CoorShiftY);
    printf("\n\tDstJunPt:%d,%d", ldcv2_attr->as32DstJunPt[0], ldcv2_attr->as32DstJunPt[1]);
    printf("\n\ts32SrcJunPt:%d", ldcv2_attr->s32SrcJunPt);
    printf("\n\tas32SrcCaliRatio:");
    printf("\n\t\t%d,%d,%d,%d",
           ldcv2_attr->as32SrcCaliRatio[0][0],
           ldcv2_attr->as32SrcCaliRatio[0][1],
           ldcv2_attr->as32SrcCaliRatio[0][2],
           ldcv2_attr->as32SrcCaliRatio[0][3]);
    printf("\n\t\t%d,%d,%d,%d",
           ldcv2_attr->as32SrcCaliRatio[1][0],
           ldcv2_attr->as32SrcCaliRatio[1][1],
           ldcv2_attr->as32SrcCaliRatio[1][2],
           ldcv2_attr->as32SrcCaliRatio[1][3]);
    printf("\n\tas32DstCaliRatio");
    printf("\n\t\t%d,%d,%d,%d",
           ldcv2_attr->as32DstCaliRatio[0][0],
           ldcv2_attr->as32DstCaliRatio[0][1],
           ldcv2_attr->as32DstCaliRatio[0][2],
           ldcv2_attr->as32DstCaliRatio[0][3]);
    printf("\n\t\t%d,%d,%d,%d",
           ldcv2_attr->as32DstCaliRatio[1][0],
           ldcv2_attr->as32DstCaliRatio[1][1],
           ldcv2_attr->as32DstCaliRatio[1][2],
           ldcv2_attr->as32DstCaliRatio[1][3]);
    printf("\n\t\t%d,%d,%d,%d",
           ldcv2_attr->as32DstCaliRatio[2][0],
           ldcv2_attr->as32DstCaliRatio[2][1],
           ldcv2_attr->as32DstCaliRatio[2][2],
           ldcv2_attr->as32DstCaliRatio[2][3]);
    printf("\n\ts32MaxDu=%d", ldcv2_attr->s32MaxDu);
    printf("\n\n");

    return ret;
}


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

