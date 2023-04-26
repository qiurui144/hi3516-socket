/******************************************************************************

  Copyright (C), 2016-2016, Hisilicon Tech. Co., Ltd.

 ******************************************************************************
  File Name     : Sample_awb_correction.c
  Version       : Initial Draft
  Author        : Hisilicon BVT PQ group
  Created       : 2016/12/15
  Description   :
  History       :
  1.Date        : 2016/12/15
    Author      : h00372898
    Modification: Created file

******************************************************************************/

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <math.h>


#include "mpi_isp.h"
#include "hi_comm_isp.h"
#include "mpi_ae.h"

#include "sample_comm.h"
#include "awb_calb_prev.h"

HI_S32 SAMPLE_VIO_StartViVo(SAMPLE_VI_CONFIG_S* pstViConfig, SAMPLE_VO_CONFIG_S* pstVoConfig)
{
    HI_S32  s32Ret;

    s32Ret = SAMPLE_COMM_VI_StartVi(pstViConfig);

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("start vi failed!\n");
        return s32Ret;
    }

    s32Ret = SAMPLE_COMM_VO_StartVO(pstVoConfig);

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("SAMPLE_VIO start VO failed with %#x!\n", s32Ret);
        goto EXIT;
    }

    return s32Ret;

EXIT:
    SAMPLE_COMM_VI_StopVi(pstViConfig);

    return s32Ret;
}

HI_S32 SAMPLE_VIO_StopViVo(SAMPLE_VI_CONFIG_S* pstViConfig, SAMPLE_VO_CONFIG_S* pstVoConfig)
{
    SAMPLE_COMM_VO_StopVO(pstVoConfig);

    SAMPLE_COMM_VI_StopVi(pstViConfig);

    return HI_SUCCESS;
}

HI_S32 SAMPLE_AWB_CALI_START_PREV(AWB_CALI_PREV_S *pstAwb_CaliPrev)
{
    HI_S32             s32Ret;
    VI_DEV             ViDev               = 0;
    HI_S32             s32WorkSnsId        = 0;

    SIZE_S             stSize;
    VB_CONFIG_S        stVbConf;
    PIC_SIZE_E         enPicSize           = PIC_3840x2160;
    HI_U32             u32BlkSize;
    combo_dev_t          ComboDev;

    DYNAMIC_RANGE_E    enDynamicRange = DYNAMIC_RANGE_SDR8;
    PIXEL_FORMAT_E     enPixFormat    = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    VIDEO_FORMAT_E     enVideoFormat  = VIDEO_FORMAT_LINEAR;
    COMPRESS_MODE_E    enCompressMode = COMPRESS_MODE_NONE;
    VI_VPSS_MODE_E     enMastPipeMode = VI_ONLINE_VPSS_OFFLINE;

    pstAwb_CaliPrev->VoDev               = SAMPLE_VO_DEV_DHD0;
    pstAwb_CaliPrev->VoChn               = 0;
    pstAwb_CaliPrev->ViPipe              = 0;
    pstAwb_CaliPrev->ViChn               = 0;

    /************************************************
    step1:  Get all sensors information
    *************************************************/
    SAMPLE_COMM_VI_GetSensorInfo(&pstAwb_CaliPrev->stViConfig);
    ComboDev = SAMPLE_COMM_VI_GetComboDevBySensor(pstAwb_CaliPrev->stViConfig.astViInfo[s32WorkSnsId].stSnsInfo.enSnsType, s32WorkSnsId);

    pstAwb_CaliPrev->stViConfig.s32WorkingViNum                           = 1;

    pstAwb_CaliPrev->stViConfig.as32WorkingViId[0]                        = 0;
    pstAwb_CaliPrev->stViConfig.astViInfo[0].stSnsInfo.MipiDev            = ComboDev;
    pstAwb_CaliPrev->stViConfig.astViInfo[0].stSnsInfo.s32BusId           = 0;

    pstAwb_CaliPrev->stViConfig.astViInfo[0].stDevInfo.ViDev              = ViDev;
    pstAwb_CaliPrev->stViConfig.astViInfo[0].stDevInfo.enWDRMode          = WDR_MODE_NONE;

    pstAwb_CaliPrev->stViConfig.astViInfo[0].stPipeInfo.enMastPipeMode    = enMastPipeMode;
    pstAwb_CaliPrev->stViConfig.astViInfo[0].stPipeInfo.aPipe[0]          = pstAwb_CaliPrev->ViPipe;
    pstAwb_CaliPrev->stViConfig.astViInfo[0].stPipeInfo.aPipe[1]          = -1;
    pstAwb_CaliPrev->stViConfig.astViInfo[0].stPipeInfo.aPipe[2]          = -1;
    pstAwb_CaliPrev->stViConfig.astViInfo[0].stPipeInfo.aPipe[3]          = -1;

    pstAwb_CaliPrev->stViConfig.astViInfo[0].stChnInfo.ViChn              = pstAwb_CaliPrev->ViChn;
    pstAwb_CaliPrev->stViConfig.astViInfo[0].stChnInfo.enPixFormat        = enPixFormat;
    pstAwb_CaliPrev->stViConfig.astViInfo[0].stChnInfo.enDynamicRange     = enDynamicRange;
    pstAwb_CaliPrev->stViConfig.astViInfo[0].stChnInfo.enVideoFormat      = enVideoFormat;
    pstAwb_CaliPrev->stViConfig.astViInfo[0].stChnInfo.enCompressMode     = enCompressMode;

    /************************************************
    step2:  Get  input size
    *************************************************/
    s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(pstAwb_CaliPrev->stViConfig.astViInfo[s32WorkSnsId].stSnsInfo.enSnsType, &enPicSize);

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("SAMPLE_COMM_VI_GetSizeBySensor failed!\n");
        return s32Ret;
    }

    s32Ret = SAMPLE_COMM_SYS_GetPicSize(enPicSize, &stSize);

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("SAMPLE_COMM_SYS_GetPicSize failed!\n");
        return s32Ret;
    }

    /************************************************
    step3:  Init SYS and common VB
    *************************************************/

    memset_s(&stVbConf, sizeof(VB_CONFIG_S), 0, sizeof(VB_CONFIG_S));
    stVbConf.u32MaxPoolCnt              = 2;

    u32BlkSize = COMMON_GetPicBufferSize(stSize.u32Width, stSize.u32Height, SAMPLE_PIXEL_FORMAT, DATA_BITWIDTH_10, COMPRESS_MODE_SEG, DEFAULT_ALIGN);
    stVbConf.astCommPool[0].u64BlkSize  = u32BlkSize;
    stVbConf.astCommPool[0].u32BlkCnt   = 10;

    u32BlkSize = VI_GetRawBufferSize(stSize.u32Width, stSize.u32Height, PIXEL_FORMAT_RGB_BAYER_16BPP, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
    stVbConf.astCommPool[1].u64BlkSize  = u32BlkSize;
    stVbConf.astCommPool[1].u32BlkCnt   = 4;

    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConf);

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("system init failed with %d!\n", s32Ret);
        SAMPLE_COMM_SYS_Exit();
        return s32Ret;
    }

    s32Ret = SAMPLE_COMM_VI_SetParam(&pstAwb_CaliPrev->stViConfig);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_COMM_SYS_Exit();
        return s32Ret;
    }


    /************************************************
    step4:  Init VI and VO
    *************************************************/
    SAMPLE_COMM_VO_GetDefConfig(&pstAwb_CaliPrev->stVoConfig);
    s32Ret = SAMPLE_VIO_StartViVo(&pstAwb_CaliPrev->stViConfig, &pstAwb_CaliPrev->stVoConfig);

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("SAMPLE_VIO_StartViVo failed witfh %d\n", s32Ret);
        goto EXIT;
    }


    /************************************************
    step5:  Bind VI and VO
    *************************************************/
    s32Ret = SAMPLE_COMM_VI_Bind_VO(pstAwb_CaliPrev->ViPipe, pstAwb_CaliPrev->ViChn, pstAwb_CaliPrev->VoDev, pstAwb_CaliPrev->VoChn);

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("SAMPLE_COMM_VI_Bind_VO failed with %#x!\n", s32Ret);
        goto EXIT1;
    }


    return s32Ret;

EXIT1:
    SAMPLE_VIO_StopViVo(&pstAwb_CaliPrev->stViConfig, &pstAwb_CaliPrev->stVoConfig);
EXIT:
    SAMPLE_COMM_SYS_Exit();

    return s32Ret;

}

HI_S32 SAMPLE_AWB_CALI_STOP_PREV(AWB_CALI_PREV_S *pstAwb_CaliPrev)
{
    if(pstAwb_CaliPrev == HI_NULL)
    {
        SAMPLE_PRT("Err: pstAwb_CaliPrev is NULL \n");
        return HI_FAILURE;
    }

    SAMPLE_COMM_VI_UnBind_VO(pstAwb_CaliPrev->ViPipe, pstAwb_CaliPrev->ViChn, pstAwb_CaliPrev->VoDev, pstAwb_CaliPrev->VoChn);
    SAMPLE_VIO_StopViVo(&pstAwb_CaliPrev->stViConfig, &pstAwb_CaliPrev->stVoConfig);
    SAMPLE_COMM_SYS_Exit();

    return HI_SUCCESS;
}

void SAMPLE_AWB_CORRECTION_Usage(char* sPrgNm)
{
    printf("Usage : %s <mode> <intf1> <intf2> <intf3>\n", sPrgNm);
    printf("mode:\n");
    printf("\t 0) Calculate Sample gain.\n");
    printf("\t 1) Adjust Sample gain according to Golden Sample.\n");

    printf("intf1:\n");
    printf("\t The value of Rgain of Golden Sample.\n");

    printf("intf2:\n");
    printf("\t The value of Bgain of Golden Sample.\n");

    printf("intf3:\n");
    printf("\t The value of Alpha ranging from 0 to 1024 (The strength of adusting Sampe Gain will increase with the value of Alpha) .\n");

    return;
}
#ifdef __HuaweiLite__
int app_main(int argc, char *argv[])
#else
int main(int argc, char *argv[])
#endif
{
    HI_S16 total_count = 0;
    HI_S16 stable_count = 0;
    VI_PIPE ViPipe = 0;
    HI_U16 u16GoldenRgain = 0;
    HI_U16 u16GoldenBgain = 0;
    HI_S16 s16Alpha = 0;
    HI_U32 u32Mode = 0;
    HI_S32 s32ret;
    ISP_EXP_INFO_S stExpInfo;
    ISP_EXPOSURE_ATTR_S stExpAttr;


    if ( (argc < 2) || (1 != strlen(argv[1])))
    {
        SAMPLE_AWB_CORRECTION_Usage(argv[0]);
        return HI_FAILURE;
    }
    if ( (argc < 5) && ('1' == *argv[1]))
    {
        SAMPLE_AWB_CORRECTION_Usage(argv[0]);
        return HI_FAILURE;
    }
#ifdef __HuaweiLite__
    AWB_CALI_PREV_S stAwb_CaliPrev;
    s32ret = SAMPLE_AWB_CALI_START_PREV(&stAwb_CaliPrev);

    if (HI_SUCCESS == s32ret)
    {
        SAMPLE_PRT("ISP is now running normally\n");
    }
    else
    {
        SAMPLE_PRT("ISP is not running normally!Please check it\n");
        return -1;
    }
    printf("input anything to continue....\n");
    getchar();
#endif
    HI_MPI_ISP_GetExposureAttr(ViPipe, &stExpAttr);

    printf("set antiflicker enable and the value of frequency to 50Hz\n");
    stExpAttr.stAuto.stAntiflicker.bEnable = HI_TRUE;
    stExpAttr.stAuto.stAntiflicker.u8Frequency = 50;
    HI_MPI_ISP_SetExposureAttr(ViPipe, &stExpAttr);

    switch (*argv[1])
    {
        case '0':
            u32Mode = 0;
            u16GoldenRgain = 0;
            u16GoldenBgain = 0;
            break;

        case '1':
            u32Mode = 1;
            u16GoldenRgain = atoi(argv[2]);
            u16GoldenBgain = atoi(argv[3]);
            s16Alpha = atoi(argv[4]);
            break;

        default:
            SAMPLE_PRT("the mode is invaild!\n");
            SAMPLE_AWB_CORRECTION_Usage(argv[0]);
            s32ret = HI_FAILURE;
            goto EXIT;
    }

    do
    {
        HI_MPI_ISP_QueryExposureInfo(ViPipe, &stExpInfo);
        usleep(100000000 / DIV_0_TO_1(stExpInfo.u32Fps));

        /*judge whether AE is stable*/
        if (stExpInfo.s16HistError > stExpAttr.stAuto.u8Tolerance)
        {
            stable_count = 0;
        }
        else
        {
            stable_count ++;
        }
        total_count ++;
    }
    while ((stable_count < 5) && (total_count < 20));


    if (stable_count >= 5)
    {
        ISP_AWB_Calibration_Gain_S stAWBCalibGain;
        HI_MPI_ISP_GetLightboxGain(ViPipe, &stAWBCalibGain);
        /*Adjust the value of Rgain and Bgain of Sample according to Golden Sample*/
        if (1 == u32Mode)
        {
            stAWBCalibGain.u16AvgRgain =  (HI_U16)((HI_S16)(stAWBCalibGain.u16AvgRgain)  + ((((HI_S16)u16GoldenRgain - (HI_S16)(stAWBCalibGain.u16AvgRgain))* s16Alpha) >> 10));
            stAWBCalibGain.u16AvgBgain = (HI_U16)((HI_S16)(stAWBCalibGain.u16AvgBgain) + ((((HI_S16)u16GoldenBgain  - (HI_S16)(stAWBCalibGain.u16AvgBgain))* s16Alpha) >> 10 ));
        }

#if 1
    if (0 == u32Mode)
    {
        printf("Calculate Sample gain:\n");
    }
    else if (1 == u32Mode)
    {
        printf("Adjust Sample gain:\n");
    }
    printf("\tu16AvgRgain =%8d, u16AvgBgain = %8d\n", stAWBCalibGain.u16AvgRgain, stAWBCalibGain.u16AvgBgain);
#endif
        s32ret = HI_SUCCESS;
        goto EXIT;

    }
    else
    {
        printf("AE IS NOT STABLE,PLEASE WAIT");
        s32ret = HI_FAILURE;
        goto EXIT;
    }
EXIT:
#ifdef __HuaweiLite__
    printf("input anything to continue....\n");
    getchar();
    SAMPLE_AWB_CALI_STOP_PREV(&stAwb_CaliPrev);
#endif
    return s32ret;
}
