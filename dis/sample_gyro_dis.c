/*
 * Copyright (C) Hisilicon Technologies Co., Ltd. 2019. All rights reserved.
 * Description : sample_gyro_dis.c
 * Author : ISP SW
 * Create : 2019-06-03
 * Version : Initial Draft
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>

#include "hi_comm_motionsensor.h"
#include "motionsensor_ext.h"
#include "motionsensor_chip_cmd.h"
#include "motionsensor_mng_cmd.h"
#include "hi_comm_motionfusion.h"
#include "mpi_motionfusion.h"
#include "sample_gyro_dis.h"
#include "sample_fov2ldc.h"
#include "sample_comm.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#define FOV_TO_LDCV2   0

#define X_BUF_LEN               (4000)
#define GYRO_BUF_LEN            ((4*4*X_BUF_LEN)+8*X_BUF_LEN)

#define RAW_GYRO_PREC_BITS   15
#define DRIFT_GYRO_PREC_BITS 15
#define IMU_TEMP_RANGE_MIN   (-40)
#define IMU_TEMP_RANGE_MAX   85
#define IMU_TEMP_PREC_BITS   16
#define INT_MAX  0x7fffffff        /* max value for an int */

#define GYRO_DATA_IPC_RANGE   250
#define GYRO_DATA_DV_RANGE    1000
HI_S32 gyro_data_range = 250;  /* IPC:250; DV:1000 */
HI_S32 g_s32MotionsensorDevFd = -1;
HI_S32 g_s32MotionsensorMngDevFd = -1;
HI_BOOL g_bGyroStarted = HI_FALSE;
hi_msensor_buf_attr  g_stMotionsensorAttr;
FILE* gyro_fp = HI_NULL;

typedef enum
{
    DIS_GYRO_MODE_IPC  = 0,
    DIS_GYRO_MODE_DV   = 1,

    DIS_GYRO_MODE_BUTT
} DIS_GYRO_MODE_E;

HI_S32 SAMPLE_SPI_Init(void)
{
    HI_S32  fd, s32Ret;
    HI_S32 s32mode = SPI_MODE_3/* | SPI_LSB_FIRST | SPI_LOOP | SPI_CS_HIGH*/;
    HI_S32 s32bits = 8;
    HI_U64 u64speed = 10000000;

    fd = open("/dev/spidev1.0", O_RDWR);

    if (fd < 0)
    {
        SAMPLE_PRT("open");
        return HI_FAILURE;
    }

    /*set spi mode */
    s32Ret = ioctl(fd, SPI_IOC_WR_MODE, &s32mode);//SPI_IOC_WR_MODE32

    if (s32Ret == HI_FAILURE)
    {
        SAMPLE_PRT("can't set spi mode");
        close(fd);
        return HI_FAILURE;
    }

    s32Ret = ioctl(fd, SPI_IOC_RD_MODE, &s32mode);//SPI_IOC_RD_MODE32

    if (s32Ret == HI_FAILURE)
    {
        SAMPLE_PRT("can't get spi mode");
        close(fd);
        return HI_FAILURE;
    }

    /*
     * bits per word
     */
    s32Ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &s32bits);

    if (s32Ret == HI_FAILURE)
    {
        SAMPLE_PRT("can't set bits per word");
        close(fd);
        return HI_FAILURE;
    }

    s32Ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &s32bits);

    if (s32Ret == HI_FAILURE)
    {
        SAMPLE_PRT("can't get bits per word");
        close(fd);
        return HI_FAILURE;
    }

    /*set spi max speed*/
    s32Ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &u64speed);

    if (s32Ret == HI_FAILURE)
    {
        SAMPLE_PRT("can't set bits max speed HZ");
        close(fd);
        return HI_FAILURE;
    }

    s32Ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &u64speed);

    if (s32Ret == HI_FAILURE)
    {
        SAMPLE_PRT("can't set bits max speed HZ");
        close(fd);
        return HI_FAILURE;
    }

    printf("spi mode: 0x%x\n", s32mode);
    printf("bits per word: %d\n", s32bits);
    printf("max speed: %lld Hz (%lld KHz)\n", u64speed, u64speed / 1000);

    close(fd);
    return HI_SUCCESS;
}

HI_S32 SAMPLE_MOTIONSENSOR_Init(DIS_GYRO_MODE_E enGyroMode)
{
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 u32BufSize = 0;
    hi_msensor_param stMSensorParamSet;

    u32BufSize = GYRO_BUF_LEN;

    g_s32MotionsensorDevFd = open("/dev/motionsensor_chip", O_RDWR);
    if (g_s32MotionsensorDevFd < 0)
    {
        SAMPLE_PRT("Error: cannot open MotionSensor device.may not load motionsensor driver !\n");
        return HI_FAILURE;
    }

    s32Ret = HI_MPI_SYS_MmzAlloc(&g_stMotionsensorAttr.phy_addr, (HI_VOID **)&g_stMotionsensorAttr.vir_addr,
                                 "MotionsensorData", NULL, u32BufSize);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("alloc mmz for Motionsensor failed,s32Ret:%x !\n", s32Ret);
        s32Ret =  HI_ERR_VI_NOMEM;
        return s32Ret;
    }

    memset((HI_VOID *)(HI_UINTPTR_T)g_stMotionsensorAttr.vir_addr, 0, u32BufSize);

    g_stMotionsensorAttr.buflen = u32BufSize;

    if (enGyroMode == DIS_GYRO_MODE_DV)
    {
        gyro_data_range = GYRO_DATA_DV_RANGE;
    }
    else if(enGyroMode == DIS_GYRO_MODE_IPC)
    {
        gyro_data_range = GYRO_DATA_IPC_RANGE;
    }

    //set device work mode
    stMSensorParamSet.attr.device_mask = MSENSOR_DEVICE_GYRO | MSENSOR_DEVICE_ACC;
    stMSensorParamSet.attr.temperature_mask = MSENSOR_TEMP_GYRO | MSENSOR_TEMP_ACC;
    //set gyro samplerate and full scale range
    stMSensorParamSet.config.gyro_config.odr = 1000 * GRADIENT ;
    stMSensorParamSet.config.gyro_config.fsr = gyro_data_range * GRADIENT;
    //set accel samplerate and full scale range
    stMSensorParamSet.config.acc_config.odr = 1000 * GRADIENT ;
    stMSensorParamSet.config.acc_config.fsr = 16 * GRADIENT;

    memcpy(&stMSensorParamSet.buf_attr, &g_stMotionsensorAttr, sizeof(hi_msensor_buf_attr));

    //stStatus.u8AxisMode = MODE_3_AXIS_GYRO_FIFO;
    //stStatus.u8AxisMode.u32DeviceMask = MOTIONSENSOR_DEVICE_GYRO | MOTIONSENSOR_DEVICE_ACC;


    s32Ret =  ioctl(g_s32MotionsensorDevFd, MSENSOR_CMD_INIT, &stMSensorParamSet);
    if (s32Ret)
    {
        SAMPLE_PRT("MSENSOR_CMD_INIT");
        return -1;
    }

    return s32Ret;
}

HI_S32 SAMPLE_MOTIONSENSOR_DeInit(void)
{
    HI_S32 s32Ret = HI_SUCCESS;

    if (g_s32MotionsensorDevFd < 0)
    {
        return HI_FAILURE;
    }

    s32Ret = ioctl(g_s32MotionsensorDevFd, MSENSOR_CMD_DEINIT, NULL);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("motionsensor deinit failed , s32Ret:0x%x !\n", s32Ret);
    }

    s32Ret = HI_MPI_SYS_MmzFree(g_stMotionsensorAttr.phy_addr, (HI_VOID *)(HI_UINTPTR_T)g_stMotionsensorAttr.vir_addr);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("motionsensor mmz free failed, s32Ret:0x%x !\n", s32Ret);
    }

    g_stMotionsensorAttr.phy_addr = 0;
    g_stMotionsensorAttr.vir_addr = (hi_u64)(HI_UINTPTR_T)NULL;

    close(g_s32MotionsensorDevFd);
    g_s32MotionsensorDevFd = -1;

    return s32Ret;
}

HI_S32 SAMPLE_MOTIONSENSOR_Start()
{
    HI_S32 s32Ret = HI_SUCCESS;
    s32Ret =  ioctl(g_s32MotionsensorDevFd, MSENSOR_CMD_START, NULL);
    if (s32Ret)
    {
        perror("IOCTL_CMD_START_MPU");
        return -1;
    }

    g_bGyroStarted = HI_TRUE;
    return s32Ret;
}

HI_S32 SAMPLE_MOTIONSENSOR_Stop(void)
{
    HI_S32 s32Ret = HI_SUCCESS;
    s32Ret = ioctl(g_s32MotionsensorDevFd, MSENSOR_CMD_STOP, NULL);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("stop motionsensor failed!\n");
    }

    return s32Ret;
}

HI_S32 SAMPLE_MOTIONFUSION_InitParam(DIS_GYRO_MODE_E enMode)
{
    HI_S32 s32Ret                   = HI_SUCCESS;
    HI_BOOL bEnDrift                = HI_TRUE;
    HI_BOOL bEnGyroTempDrift        = HI_TRUE;
    HI_BOOL bEnSixSideCal           = HI_TRUE;
    IMU_MATRIX aRotationMatrix      = { -32768, 0, 0, 0, 32768, 0, 0, 0, -32768};
    hi_mfusion_attr stMFusionAttr;
    HI_U32 u32FusionID = 0;
    IMU_DRIFT aGyroDrift = {0,0,0};
    hi_mfusion_temp_drift stTempDrift = {0};

    stMFusionAttr.steady_detect_attr.steady_time_thr = 3;
    stMFusionAttr.steady_detect_attr.gyro_offset = (HI_S32)(10 * (1 << 15));
    stMFusionAttr.steady_detect_attr.acc_offset = (HI_S32)(0.1 * (1 << 15));
    stMFusionAttr.steady_detect_attr.gyro_rms = (HI_S32)(0.054 * (1 << 15));
    stMFusionAttr.steady_detect_attr.acc_rms = (HI_S32)(1.3565 * (1 << 15) / 1000);
    stMFusionAttr.steady_detect_attr.gyro_offset_factor = (HI_S32)(2 * (1 << 4));
    stMFusionAttr.steady_detect_attr.acc_offset_factor = (HI_S32)(2 * (1 << 4));
    stMFusionAttr.steady_detect_attr.gyro_rms_factor = (HI_S32)(8 * (1 << 4));
    stMFusionAttr.steady_detect_attr.acc_rms_factor = (HI_S32)(10 * (1 << 4));

    stMFusionAttr.device_mask      = MFUSION_DEVICE_GYRO | MFUSION_DEVICE_ACC;
    stMFusionAttr.temperature_mask = MFUSION_TEMP_GYRO | MFUSION_TEMP_ACC;

    if (enMode == DIS_GYRO_MODE_IPC)
    {
        stMFusionAttr.steady_detect_attr.steady_time_thr = 3;
        stMFusionAttr.steady_detect_attr.gyro_rms_factor = (HI_S32)(8 * (1 << 4));
        stMFusionAttr.steady_detect_attr.acc_rms_factor = (HI_S32)(10 * (1 << 4));
    }
    else if (enMode == DIS_GYRO_MODE_DV)
    {
        stMFusionAttr.steady_detect_attr.steady_time_thr = 1;
        stMFusionAttr.steady_detect_attr.gyro_rms_factor = (HI_S32)(12.5 * (1 << 4));
        stMFusionAttr.steady_detect_attr.acc_rms_factor = (HI_S32)(100 * (1 << 4));
    }

    s32Ret = hi_mpi_motionfusion_set_attr(u32FusionID, &stMFusionAttr);
    if (HI_SUCCESS != s32Ret)
    {
        goto end;
    }

    //s32Ret = hi_mpi_motionfusion_set_gyro_drift(u32FusionID, bEnDrift, aGyroDrift);
    //if (HI_SUCCESS != s32Ret)
    //{
    //    SAMPLE_PRT("HI_MPI_MOTIONFUSION_SetAttr failed!\n");
    //    goto end;
    //}

    s32Ret = hi_mpi_motionfusion_set_gyro_six_side_cal(u32FusionID, bEnSixSideCal, aRotationMatrix);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("hi_mpi_motionfusion_set_gyro_six_side_cal failed!\n");
        goto end;
    }

    if (enMode == DIS_GYRO_MODE_IPC)
    {
        s32Ret = hi_mpi_motionfusion_set_gyro_online_drift(u32FusionID, bEnDrift, aGyroDrift);
        if (HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("hi_mpi_motionfusion_set_gyro_online_drift failed!\n");
            goto end;
        }
    }
    else if (enMode == DIS_GYRO_MODE_DV)
    {
        hi_u32 i;
        stTempDrift.mode = IMU_TEMP_DRIFT_LUT;
        //stTempDrift.stTempLut.s32RangeMin = 28 * 1024;
        //stTempDrift.stTempLut.s32RangeMax = 57 * 1024;
        //stTempDrift.stTempLut.u32Step     = 1024;
        //memcpy(stTempDrift.stTempLut.as32IMULut, as32IMULut, TEMP_LUT_SAMPLES * AXIS_NUM * sizeof(HI_S32));
        stTempDrift.temp_lut.range_min = 20 * 1024;
        stTempDrift.temp_lut.range_max = 49 * 1024;
        stTempDrift.temp_lut.step     = 1 * 1024;

        for (i = 0; i < TEMP_LUT_SAMPLES; i++)
        {
          stTempDrift.temp_lut.gyro_lut_status[i][0] = INT_MAX;
          stTempDrift.temp_lut.gyro_lut_status[i][1] = INT_MAX;
        }

        memset(stTempDrift.temp_lut.imu_lut, 0, TEMP_LUT_SAMPLES * AXIS_NUM * sizeof(HI_S32));

        s32Ret = hi_mpi_motionfusion_set_gyro_online_temp_drift(u32FusionID, bEnGyroTempDrift, &stTempDrift);
        if (HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("hi_mpi_motionfusion_set_gyro_online_temp_drift failed!\n");
            goto end;
        }
    }

end:
    return s32Ret;
}

HI_S32 SAMPLE_MOTIONFUSION_DeInitParam()
{
    HI_U32 u32FusionID = 0;
    HI_S32 s32Ret;
    hi_mfusion_temp_drift stTempDrift = {0};
    IMU_DRIFT aGyroDrift = {0};
    HI_BOOL bEn = HI_TRUE;

    s32Ret = hi_mpi_motionfusion_get_gyro_online_drift(u32FusionID, &bEn, aGyroDrift);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_MOTIONFUSION_GetGyroOnLineDrift failed!\n");
        goto end;
    }

    s32Ret = hi_mpi_motionfusion_set_gyro_online_drift(u32FusionID, HI_FALSE, aGyroDrift);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("hi_mpi_motionfusion_set_gyro_online_drift failed!\n");
        goto end;
    }

    s32Ret = hi_mpi_motionfusion_get_gyro_online_temp_drift(u32FusionID, &bEn, &stTempDrift);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_MOTIONFUSION_GetGyroOnLineTempDrift failed!\n");
        goto end;
    }

    s32Ret = hi_mpi_motionfusion_set_gyro_online_temp_drift(u32FusionID, HI_FALSE, &stTempDrift);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("hi_mpi_motionfusion_set_gyro_temp_drift failed!\n");
        goto end;
    }

    sleep(1);
end:
    return s32Ret;
}


HI_S32 SAMPLE_DIS_GYRO(DIS_GYRO_MODE_E enGyroMode)
{
    HI_S32 s32Ret               = HI_SUCCESS;
    HI_U32 u32BlkSize;
    SIZE_S stSize;
    VB_CONFIG_S stVbConfig;
    PIC_SIZE_E  enPicSize       = PIC_3840x2160;
    VI_VPSS_MODE_E     enMastPipeMode = VI_ONLINE_VPSS_OFFLINE;

    VI_DEV  ViDev               = 0;
    VI_PIPE ViPipe              = 0;
    VI_CHN  ViChn               = 0;
    HI_S32  s32WorkSnsId        = 0;
    SAMPLE_VI_CONFIG_S stViConfig;

    VPSS_GRP VpssGrp            = 0;
    VPSS_CHN VpssChn            = 0;
    VPSS_GRP_ATTR_S         stVpssGrpAttr       = {0};
    VPSS_CHN_ATTR_S         stVpssChnAttr[VPSS_MAX_PHY_CHN_NUM];
    HI_BOOL                 abChnEnable[VPSS_MAX_PHY_CHN_NUM] = {0};

    VO_CHN VoChn                = 0;
    SAMPLE_VO_CONFIG_S stVoConfig;

    HI_U32 u32Profile           = 0;
    HI_BOOL bRcnRefShareBuf     = HI_FALSE;
    HI_U32 s32ChnNum            = 1;
    VENC_CHN VencChn            = 0;
    VENC_GOP_ATTR_S stGopAttr;
    SAMPLE_RC_E enRcMode        = SAMPLE_RC_CBR;
    PAYLOAD_TYPE_E enPayLoad    = PT_H265;

    DIS_CONFIG_S stDISConfig     = {0};
    DIS_ATTR_S stDISAttr         = {0};

    VI_LDCV2_ATTR_S stLDCV2Attr  = {0};
    HI_U32         u32FrameRate  = 30;
#if FOV_TO_LDCV2
    hi_fov_attr fov_attr;
#endif

    /************************************************
    step 1:  get all sensors information
    *************************************************/
    SAMPLE_COMM_VI_GetSensorInfo(&stViConfig);
    stViConfig.s32WorkingViNum                           = 1;

    stViConfig.as32WorkingViId[0]                        = s32WorkSnsId;
    stViConfig.astViInfo[0].stSnsInfo.MipiDev            = SAMPLE_COMM_VI_GetComboDevBySensor(stViConfig.astViInfo[0].stSnsInfo.enSnsType, 0);
    stViConfig.astViInfo[0].stSnsInfo.s32BusId           = 0;

    stViConfig.astViInfo[0].stDevInfo.ViDev              = ViDev;
    stViConfig.astViInfo[0].stDevInfo.enWDRMode          = WDR_MODE_NONE;

    stViConfig.astViInfo[0].stPipeInfo.enMastPipeMode    = enMastPipeMode;

    stViConfig.astViInfo[0].stPipeInfo.aPipe[0]          = ViPipe;
    stViConfig.astViInfo[0].stPipeInfo.aPipe[1]          = -1;
    stViConfig.astViInfo[0].stPipeInfo.aPipe[2]          = -1;
    stViConfig.astViInfo[0].stPipeInfo.aPipe[3]          = -1;

    stViConfig.astViInfo[0].stChnInfo.ViChn              = ViChn;
    stViConfig.astViInfo[0].stChnInfo.enPixFormat        = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stViConfig.astViInfo[0].stChnInfo.enDynamicRange     = DYNAMIC_RANGE_SDR8;
    stViConfig.astViInfo[0].stChnInfo.enVideoFormat      = VIDEO_FORMAT_LINEAR;
    stViConfig.astViInfo[0].stChnInfo.enCompressMode     = COMPRESS_MODE_NONE;

    /************************************************
    step 2:  get input size
    *************************************************/
    s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(stViConfig.astViInfo[s32WorkSnsId].stSnsInfo.enSnsType, &enPicSize);
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

    s32Ret = SAMPLE_COMM_VI_GetFrameRateBySensor(stViConfig.astViInfo[s32WorkSnsId].stSnsInfo.enSnsType, &u32FrameRate);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("SAMPLE_COMM_VI_GetFrameRateBySensor failed!\n");
        return s32Ret;
    }

    printf("input size:%dx%d, frame rate:%d\n",stSize.u32Width, stSize.u32Height, u32FrameRate);
    /************************************************
    step 3: init SYS and common VB
    *************************************************/
    memset(&stVbConfig, 0, sizeof(VB_CONFIG_S));

    u32BlkSize = COMMON_GetPicBufferSize(stSize.u32Width, stSize.u32Height, PIXEL_FORMAT_YVU_SEMIPLANAR_420, DATA_BITWIDTH_8, COMPRESS_MODE_SEG, 0);
    stVbConfig.u32MaxPoolCnt                = 128;
    stVbConfig.astCommPool[0].u64BlkSize    = u32BlkSize;
    stVbConfig.astCommPool[0].u32BlkCnt     = stViConfig.s32WorkingViNum * 20;

    u32BlkSize = VI_GetRawBufferSize(stSize.u32Width, stSize.u32Height, PIXEL_FORMAT_RGB_BAYER_16BPP, COMPRESS_MODE_NONE, 0);
    stVbConfig.astCommPool[1].u64BlkSize  = u32BlkSize;
    stVbConfig.astCommPool[1].u32BlkCnt   = 4;

    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConfig);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("init sys fail.s32Ret:0x%x !\n", s32Ret);
        goto EXIT;
    }

    s32Ret = SAMPLE_COMM_VI_SetParam(&stViConfig);
    if (HI_SUCCESS != s32Ret)
    {
        goto EXIT;
    }

    /************************************************
    step 4: start VI
    *************************************************/
    s32Ret = SAMPLE_COMM_VI_StartVi(&stViConfig);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("start vi failed.s32Ret:0x%x !\n", s32Ret);
        goto EXIT1;
    }

    /************************************************
    step 5: init and start gyro
    *************************************************/
    s32Ret = SAMPLE_SPI_Init();
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("init spi fail.s32Ret:0x%x !\n", s32Ret);
        goto EXIT1;
    }

    s32Ret = SAMPLE_MOTIONSENSOR_Init(enGyroMode);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("init gyro fail.s32Ret:0x%x !\n", s32Ret);
        goto EXIT1;
    }

    s32Ret = SAMPLE_MOTIONSENSOR_Start();
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("start gyro fail.s32Ret:0x%x !\n", s32Ret);
        goto EXIT1;
    }

    s32Ret = SAMPLE_MOTIONFUSION_InitParam(enGyroMode);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("motionfusion set param fail.s32Ret:0x%x !\n", s32Ret);
        goto EXIT1;
    }

    /************************************************
    step 6: set ldcV2 config & attribute
    *************************************************/

    if (enGyroMode == DIS_GYRO_MODE_DV)
    {
        stDISConfig.enPdtType           = DIS_PDT_TYPE_DV;
        stDISConfig.bCameraSteady       = HI_FALSE;

        if (1920 == stSize.u32Width && 1080 == stSize.u32Height)
        {
            /*Hi3559V200 DV 1080P*/
            stLDCV2Attr.bEnable = HI_FALSE;
            stLDCV2Attr.stAttr.s32FocalLenX = 111709;
            stLDCV2Attr.stAttr.s32FocalLenY = 108474;
            stLDCV2Attr.stAttr.s32CoorShiftX = 92487;
            stLDCV2Attr.stAttr.s32CoorShiftY = 50727;

            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][0] = 100000;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][1] = -31685;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][2] = 10571;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][3] = -1645;
            stLDCV2Attr.stAttr.s32SrcJunPt = 800000;

            stLDCV2Attr.stAttr.as32DstCaliRatio[0][0] = 0;
            stLDCV2Attr.stAttr.as32DstCaliRatio[0][1] = 0;
            stLDCV2Attr.stAttr.as32DstCaliRatio[0][2] = 0;
            stLDCV2Attr.stAttr.as32DstCaliRatio[0][3] = 0;
            stLDCV2Attr.stAttr.as32DstCaliRatio[1][0] = 100032;
            stLDCV2Attr.stAttr.as32DstCaliRatio[1][1] = 31213;
            stLDCV2Attr.stAttr.as32DstCaliRatio[1][2] = 20512;
            stLDCV2Attr.stAttr.as32DstCaliRatio[1][3] = 16565;
            stLDCV2Attr.stAttr.as32DstCaliRatio[2][0] = 61533;
            stLDCV2Attr.stAttr.as32DstCaliRatio[2][1] = 195176;
            stLDCV2Attr.stAttr.as32DstCaliRatio[2][2] = -206546;
            stLDCV2Attr.stAttr.as32DstCaliRatio[2][3] = 117773;
            stLDCV2Attr.stAttr.as32DstJunPt[0] = 0;
            stLDCV2Attr.stAttr.as32DstJunPt[1] = 77560;

            stLDCV2Attr.stAttr.s32MaxDu = (HI_S32)(1.6473 * (1 << 16));
            if (u32FrameRate == 30)
            {
                stDISAttr.s32Timelag            = -30030;
            }
            else if (u32FrameRate == 60)
            {
                stDISAttr.s32Timelag            = -14646;
            }
            else
            {
                stDISAttr.s32Timelag            = -30030;
            }
        }
        else
        {
            /*Hi3559V200 DV 4K*/
            /*8 mm,, no LDC 4K*/
            stLDCV2Attr.bEnable = HI_FALSE;
            stLDCV2Attr.stAttr.s32FocalLenX = 2281550;
            stLDCV2Attr.stAttr.s32FocalLenY = 2303229;
            stLDCV2Attr.stAttr.s32CoorShiftX = 191950;
            stLDCV2Attr.stAttr.s32CoorShiftY = 107950;

            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][0] = 100000;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][1] = 0;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][2] = 0;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][3] = 0;
            stLDCV2Attr.stAttr.s32SrcJunPt = 800000;

            stLDCV2Attr.stAttr.as32DstCaliRatio[0][0] = 100000;
            stLDCV2Attr.stAttr.as32DstCaliRatio[0][1] = 0;
            stLDCV2Attr.stAttr.as32DstCaliRatio[0][2] = 0;
            stLDCV2Attr.stAttr.as32DstCaliRatio[0][3] = 0;
            stLDCV2Attr.stAttr.as32DstJunPt[0] = 800000;
            stLDCV2Attr.stAttr.as32DstJunPt[1] = 800000;
            stLDCV2Attr.stAttr.s32MaxDu        = (HI_S32)(16 * (1 << 16));

            stDISAttr.s32Timelag            = -30830; //30fps
        }
    }
    else if (enGyroMode == DIS_GYRO_MODE_IPC)
    {
        stDISConfig.enPdtType           = DIS_PDT_TYPE_IPC;
        stDISConfig.bCameraSteady       = HI_TRUE;

        if (1920 == stSize.u32Width && 1080 == stSize.u32Height)
        {

            /*6 mm,, no LDC 1080P*/
            stLDCV2Attr.bEnable = HI_FALSE;
            stLDCV2Attr.stAttr.s32FocalLenX = 214549;
            stLDCV2Attr.stAttr.s32FocalLenY = 214995;
            stLDCV2Attr.stAttr.s32CoorShiftX = 95726;
            stLDCV2Attr.stAttr.s32CoorShiftY = 57317;

            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][0] = 100000;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][1] = -49193;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][2] = 31677;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][3] = -15235;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[1][0] = 0;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[1][1] = 0;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[1][2] = 0;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[1][3] = 0;
            stLDCV2Attr.stAttr.s32SrcJunPt = 3100000;

            stLDCV2Attr.stAttr.as32DstCaliRatio[0][0] = 0;
            stLDCV2Attr.stAttr.as32DstCaliRatio[0][1] = 0;
            stLDCV2Attr.stAttr.as32DstCaliRatio[0][2] = 0;
            stLDCV2Attr.stAttr.as32DstCaliRatio[0][3] = 0;
            stLDCV2Attr.stAttr.as32DstCaliRatio[1][0] = 99999;
            stLDCV2Attr.stAttr.as32DstCaliRatio[1][1] = 49261;
            stLDCV2Attr.stAttr.as32DstCaliRatio[1][2] = 39357;
            stLDCV2Attr.stAttr.as32DstCaliRatio[1][3] = 45920;
            stLDCV2Attr.stAttr.as32DstCaliRatio[2][0] = 99839;
            stLDCV2Attr.stAttr.as32DstCaliRatio[2][1] = 52391;
            stLDCV2Attr.stAttr.as32DstCaliRatio[2][2] = 18590;
            stLDCV2Attr.stAttr.as32DstCaliRatio[2][3] = 92626;
            stLDCV2Attr.stAttr.as32DstJunPt[0] = 0;
            stLDCV2Attr.stAttr.as32DstJunPt[1] = 38565;
            stLDCV2Attr.stAttr.s32MaxDu        = (HI_S32)40109;
        }
        else
        {
            /*8 mm,, no LDC 4K*/
            stLDCV2Attr.bEnable = HI_FALSE;
            stLDCV2Attr.stAttr.s32FocalLenX = 2281550;
            stLDCV2Attr.stAttr.s32FocalLenY = 2303229;
            stLDCV2Attr.stAttr.s32CoorShiftX = 191950;
            stLDCV2Attr.stAttr.s32CoorShiftY = 107950;

            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][0] = 100000;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][1] = 0;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][2] = 0;
            stLDCV2Attr.stAttr.as32SrcCaliRatio[0][3] = 0;
            stLDCV2Attr.stAttr.s32SrcJunPt = 800000;

            stLDCV2Attr.stAttr.as32DstCaliRatio[0][0] = 100000;
            stLDCV2Attr.stAttr.as32DstCaliRatio[0][1] = 0;
            stLDCV2Attr.stAttr.as32DstCaliRatio[0][2] = 0;
            stLDCV2Attr.stAttr.as32DstCaliRatio[0][3] = 0;
            stLDCV2Attr.stAttr.as32DstJunPt[0] = 800000;
            stLDCV2Attr.stAttr.as32DstJunPt[1] = 800000;
            stLDCV2Attr.stAttr.s32MaxDu        = (HI_S32)(16 * (1 << 16));
        }

        stDISAttr.s32Timelag            = -30100;
    }

#if FOV_TO_LDCV2
    fov_attr.width  = stSize.u32Width;
    fov_attr.height = stSize.u32Height;
    fov_attr.type   = fov_type_diagonal;
    fov_attr.fov    = 90 * (1 << FOV_PREC_BITS);

    s32Ret = hi_sample_fov_to_ldcv2(&fov_attr, &stLDCV2Attr.stAttr);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("hi_sample_fov_to_ldcv2 failed.s32Ret:0x%x !\n", s32Ret);
        goto EXIT1;
    }
#endif

    s32Ret = HI_MPI_VI_SetChnLDCV2Attr(ViPipe, ViChn, &stLDCV2Attr);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("set ldcV2 config failed.s32Ret:0x%x !\n", s32Ret);
        goto EXIT1;
    }

    /************************************************
    step 6: set DIS config & attribute
    *************************************************/
    stDISConfig.enMode              = DIS_MODE_GYRO;
    stDISConfig.enMotionLevel       = DIS_MOTION_LEVEL_NORMAL;
    stDISConfig.u32CropRatio        = 80;
    stDISConfig.u32BufNum           = 5;
    stDISConfig.u32GyroOutputRange  = 1000;
    stDISConfig.u32GyroDataBitWidth = 15;
    stDISConfig.bScale              = HI_FALSE;
    stDISConfig.u32FrameRate        = u32FrameRate;

    stDISAttr.bEnable               = HI_TRUE;
    stDISAttr.u32MovingSubjectLevel = 0;
    stDISAttr.s32RollingShutterCoef = 0;
    stDISAttr.u32ViewAngle          = 120;
    stDISAttr.bStillCrop            = HI_FALSE;
    stDISAttr.u32HorizontalLimit    = 512;
    stDISAttr.u32VerticalLimit      = 512;
    stDISAttr.u32Strength           = 1024;

    s32Ret = HI_MPI_VI_SetChnDISConfig(ViPipe, ViChn, &stDISConfig);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("set dis config failed.s32Ret:0x%x !\n", s32Ret);
        goto EXIT1;
    }

    s32Ret = HI_MPI_VI_SetChnDISAttr(ViPipe, ViChn, &stDISAttr);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("set dis attr failed.s32Ret:0x%x !\n", s32Ret);
        goto EXIT1;
    }

    /************************************************
    step 7:  start VPSS
    *************************************************/
    stVpssGrpAttr.u32MaxW                        = stSize.u32Width;
    stVpssGrpAttr.u32MaxH                        = stSize.u32Height;
    stVpssGrpAttr.enPixelFormat                  = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stVpssGrpAttr.enDynamicRange                 = DYNAMIC_RANGE_SDR8;
    stVpssGrpAttr.stFrameRate.s32SrcFrameRate    = -1;
    stVpssGrpAttr.stFrameRate.s32DstFrameRate    = -1;

    abChnEnable[0]                               = HI_TRUE;
    stVpssChnAttr[0].u32Width                    = stSize.u32Width;
    stVpssChnAttr[0].u32Height                   = stSize.u32Height;
    stVpssChnAttr[0].enChnMode                   = VPSS_CHN_MODE_USER;
    stVpssChnAttr[0].enCompressMode              = COMPRESS_MODE_NONE;
    stVpssChnAttr[0].enDynamicRange              = DYNAMIC_RANGE_SDR8;
    stVpssChnAttr[0].enPixelFormat               = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stVpssChnAttr[0].enVideoFormat               = VIDEO_FORMAT_LINEAR;
    stVpssChnAttr[0].stFrameRate.s32SrcFrameRate = -1;
    stVpssChnAttr[0].stFrameRate.s32DstFrameRate = -1;
    stVpssChnAttr[0].u32Depth                    = 1;
    stVpssChnAttr[0].bMirror                     = HI_FALSE;
    stVpssChnAttr[0].bFlip                       = HI_FALSE;
    stVpssChnAttr[0].stAspectRatio.enMode        = ASPECT_RATIO_NONE;

    s32Ret = SAMPLE_COMM_VPSS_Start(VpssGrp, abChnEnable, &stVpssGrpAttr, stVpssChnAttr);

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("start vpss failed. s32Ret: 0x%x !\n", s32Ret);
        goto EXIT1;
    }

    /************************************************
    step 8:  start VO
    *************************************************/
    SAMPLE_COMM_VO_GetDefConfig(&stVoConfig);

    s32Ret = SAMPLE_COMM_VO_StartVO(&stVoConfig);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("start vo failed. s32Ret: 0x%x !\n", s32Ret);
        goto EXIT2;
    }

    /************************************************
    step 9:  VO bind VPSS
    *************************************************/
    s32Ret = SAMPLE_COMM_VPSS_Bind_VO(VpssGrp, VpssChn, stVoConfig.VoDev, VoChn);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("vo bind vpss failed. s32Ret: 0x%x !\n", s32Ret);
        goto EXIT3;
    }

    /************************************************
    step 10:  VI bind VPSS
    *************************************************/
    s32Ret = SAMPLE_COMM_VI_Bind_VPSS(ViPipe, ViChn, VpssGrp);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("vi bind vpss failed. s32Ret: 0x%x !\n", s32Ret);
        goto EXIT4;
    }

    /************************************************
    step 11:  start VENC
    *************************************************/
    s32Ret = SAMPLE_COMM_VENC_GetGopAttr(VENC_GOPMODE_NORMALP, &stGopAttr);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("venc get Gop attr failed. s32Ret: 0x%x !\n", s32Ret);
        goto EXIT5;
    }

    s32Ret = SAMPLE_COMM_VENC_Start(VencChn, enPayLoad, enPicSize, enRcMode, u32Profile, bRcnRefShareBuf, &stGopAttr);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("start venc failed. s32Ret: 0x%x !\n", s32Ret);
        goto EXIT5;
    }

    s32Ret = SAMPLE_COMM_VPSS_Bind_VENC(VpssGrp, VpssChn, VencChn);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("vpss bind venc failed. s32Ret: 0x%x !\n", s32Ret);
        goto EXIT6;
    }

    /************************************************
    step 12: stream VENC process -- get stream, then save it to file.
    *************************************************/
    s32Ret = SAMPLE_COMM_VENC_StartGetStream(&VencChn, s32ChnNum);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("venc start get stream failed. s32Ret: 0x%x !\n", s32Ret);
        goto EXIT7;
    }

    printf("\nplease hit the Enter key to Disable DIS!\n");
    getchar();

    stDISAttr.bEnable = HI_FALSE;
    s32Ret = HI_MPI_VI_SetChnDISAttr(ViPipe, ViChn, &stDISAttr);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("set dis attr failed.s32Ret:0x%x !\n", s32Ret);
        goto EXIT8;
    }

    printf("\nplease hit the Enter key to enable DIS!\n");
    getchar();

    stDISAttr.bEnable = HI_TRUE;
    s32Ret = HI_MPI_VI_SetChnDISAttr(ViPipe, ViChn, &stDISAttr);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("set dis attr failed.s32Ret:0x%x !\n", s32Ret);
        goto EXIT8;
    }

    printf("\nplease hit the Enter key to exit!\n");
    getchar();

    /************************************************
    step 13: exit process
    *************************************************/
EXIT8:
    SAMPLE_COMM_VENC_StopGetStream();
EXIT7:
    SAMPLE_COMM_VPSS_UnBind_VENC(VpssGrp, VpssChn, VencChn);
EXIT6:
    SAMPLE_COMM_VENC_Stop(VencChn);
EXIT5:
    SAMPLE_COMM_VI_UnBind_VPSS(ViPipe, ViChn, VpssGrp);
EXIT4:
    SAMPLE_COMM_VPSS_UnBind_VO(VpssGrp, VpssChn, stVoConfig.VoDev, VoChn);
EXIT3:
    SAMPLE_COMM_VO_StopVO(&stVoConfig);
EXIT2:
    SAMPLE_COMM_VPSS_Stop(VpssGrp, abChnEnable);
EXIT1:
    SAMPLE_COMM_VI_StopVi(&stViConfig);
EXIT:

    SAMPLE_MOTIONFUSION_DeInitParam();
    SAMPLE_MOTIONSENSOR_Stop();
    SAMPLE_MOTIONSENSOR_DeInit();
    SAMPLE_COMM_SYS_Exit();

    return s32Ret;
}

HI_S32 SAMPLE_DIS_IPC_GYRO()
{
    return SAMPLE_DIS_GYRO(DIS_GYRO_MODE_IPC);
}

HI_S32 SAMPLE_DIS_DV_GYRO()
{
    return SAMPLE_DIS_GYRO(DIS_GYRO_MODE_DV);
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of __cplusplus */

