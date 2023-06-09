#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#ifndef __HuaweiLite__
#include <sys/poll.h>
#endif
#include <sys/time.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#include "hi_common.h"
#include "sample_comm.h"
#include "hi_comm_video.h"
#include "hi_comm_sys.h"
#include "hi_comm_vi.h"
#include "hi_comm_isp.h"
#include "mpi_vb.h"
#include "mpi_sys.h"
#include "mpi_vi.h"
#include "mpi_isp.h"

#include "lsc_cali_prev.h"
#define DUMP_RAW_AND_SAVE_LSC (1)

#define MAX_FRM_CNT     25
#define MAX_FRM_WIDTH   8192

#define ALIGN_BACK(x, a)              ((a) * (((x) / (a))))

#if 1
#define MEM_DEV_OPEN() \
    do {\
        if (s_s32MemDev <= 0)\
        {\
            s_s32MemDev = open("/dev/mem", O_CREAT|O_RDWR|O_SYNC);\
            if (s_s32MemDev < 0)\
            {\
                perror("Open dev/mem error");\
                return -1;\
            }\
        }\
    }while(0)

#define MEM_DEV_CLOSE() \
    do {\
        HI_S32 s32Ret;\
        if (s_s32MemDev > 0)\
        {\
            s32Ret = close(s_s32MemDev);\
            if(HI_SUCCESS != s32Ret)\
            {\
                perror("Close mem/dev Fail");\
                return s32Ret;\
            }\
            s_s32MemDev = -1;\
        }\
    }while(0)

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

HI_S32 SAMPLE_LSC_CALI_START_PREV(LSC_CALI_PREV_S *pstLsc_CaliPrev)
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

    pstLsc_CaliPrev->VoDev               = SAMPLE_VO_DEV_DHD0;
    pstLsc_CaliPrev->VoChn               = 0;
    pstLsc_CaliPrev->ViPipe              = 0;
    pstLsc_CaliPrev->ViChn               = 0;


    /************************************************
    step1:  Get all sensors information
    *************************************************/
    SAMPLE_COMM_VI_GetSensorInfo(&pstLsc_CaliPrev->stViConfig);
    ComboDev = SAMPLE_COMM_VI_GetComboDevBySensor(pstLsc_CaliPrev->stViConfig.astViInfo[s32WorkSnsId].stSnsInfo.enSnsType, s32WorkSnsId);


    pstLsc_CaliPrev->stViConfig.s32WorkingViNum                           = 1;

    pstLsc_CaliPrev->stViConfig.as32WorkingViId[0]                        = 0;
    pstLsc_CaliPrev->stViConfig.astViInfo[0].stSnsInfo.MipiDev            = ComboDev;
    pstLsc_CaliPrev->stViConfig.astViInfo[0].stSnsInfo.s32BusId           = 0;

    pstLsc_CaliPrev->stViConfig.astViInfo[0].stDevInfo.ViDev              = ViDev;
    pstLsc_CaliPrev->stViConfig.astViInfo[0].stDevInfo.enWDRMode          = WDR_MODE_NONE;

    pstLsc_CaliPrev->stViConfig.astViInfo[0].stPipeInfo.enMastPipeMode    = enMastPipeMode;
    pstLsc_CaliPrev->stViConfig.astViInfo[0].stPipeInfo.aPipe[0]          = pstLsc_CaliPrev->ViPipe;
    pstLsc_CaliPrev->stViConfig.astViInfo[0].stPipeInfo.aPipe[1]          = -1;
    pstLsc_CaliPrev->stViConfig.astViInfo[0].stPipeInfo.aPipe[2]          = -1;
    pstLsc_CaliPrev->stViConfig.astViInfo[0].stPipeInfo.aPipe[3]          = -1;

    pstLsc_CaliPrev->stViConfig.astViInfo[0].stChnInfo.ViChn              = pstLsc_CaliPrev->ViChn;
    pstLsc_CaliPrev->stViConfig.astViInfo[0].stChnInfo.enPixFormat        = enPixFormat;
    pstLsc_CaliPrev->stViConfig.astViInfo[0].stChnInfo.enDynamicRange     = enDynamicRange;
    pstLsc_CaliPrev->stViConfig.astViInfo[0].stChnInfo.enVideoFormat      = enVideoFormat;
    pstLsc_CaliPrev->stViConfig.astViInfo[0].stChnInfo.enCompressMode     = enCompressMode;

    /************************************************
    step2:  Get  input size
    *************************************************/
    s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(pstLsc_CaliPrev->stViConfig.astViInfo[s32WorkSnsId].stSnsInfo.enSnsType, &enPicSize);

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
    stVbConf.astCommPool[0].u32BlkCnt   = 5;

    u32BlkSize = VI_GetRawBufferSize(stSize.u32Width, stSize.u32Height, PIXEL_FORMAT_RGB_BAYER_16BPP, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
    stVbConf.astCommPool[1].u64BlkSize  = u32BlkSize;
    stVbConf.astCommPool[1].u32BlkCnt   = 10;

    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConf);

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("system init failed with %d!\n", s32Ret);
        SAMPLE_COMM_SYS_Exit();
        return s32Ret;
    }

    s32Ret = SAMPLE_COMM_VI_SetParam(&pstLsc_CaliPrev->stViConfig);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_COMM_SYS_Exit();
        return s32Ret;
    }


    /************************************************
    step4:  Init VI and VO
    *************************************************/
    SAMPLE_COMM_VO_GetDefConfig(&pstLsc_CaliPrev->stVoConfig);
    s32Ret = SAMPLE_VIO_StartViVo(&pstLsc_CaliPrev->stViConfig, &pstLsc_CaliPrev->stVoConfig);

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("SAMPLE_VIO_StartViVo failed witfh %d\n", s32Ret);
        goto EXIT;
    }


    /************************************************
    step5:  Bind VI and VO
    *************************************************/
    s32Ret = SAMPLE_COMM_VI_Bind_VO(pstLsc_CaliPrev->ViPipe, pstLsc_CaliPrev->ViChn, pstLsc_CaliPrev->VoDev, pstLsc_CaliPrev->VoChn);

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("SAMPLE_COMM_VI_Bind_VO failed with %#x!\n", s32Ret);
        goto EXIT1;
    }


    return s32Ret;

EXIT1:
    SAMPLE_VIO_StopViVo(&pstLsc_CaliPrev->stViConfig, &pstLsc_CaliPrev->stVoConfig);
EXIT:
    SAMPLE_COMM_SYS_Exit();

    return s32Ret;

}

HI_S32 SAMPLE_LSC_CALI_STOP_PREV(LSC_CALI_PREV_S *pstLsc_CaliPrev)
{
    if(pstLsc_CaliPrev == HI_NULL)
    {
        SAMPLE_PRT("Err: pstLsc_CaliPrev is NULL \n");
        return HI_FAILURE;
    }

    SAMPLE_COMM_VI_UnBind_VO(pstLsc_CaliPrev->ViPipe, pstLsc_CaliPrev->ViChn, pstLsc_CaliPrev->VoDev, pstLsc_CaliPrev->VoChn);
    SAMPLE_VIO_StopViVo(&pstLsc_CaliPrev->stViConfig, &pstLsc_CaliPrev->stVoConfig);
    SAMPLE_COMM_SYS_Exit();

    return HI_SUCCESS;
}

void usage(void)
{
    printf(
        "\n"
        "*************************************************\n"
        "Usage: ./lsc_online_cali [ViPipe] [scale] \n"
        "ViPipe: \n"
        "    0:ViPipe0 ~ 3:ViPipe 3\n"
        "scale: \n"
        "   scale value to be used to calculate gain(range:[0,7])\n"
        "e.g : ./lsc_online_cali 0 0\n"
        "*************************************************\n"
        "\n");
    exit(1);
}
#endif

/******************************************************************************
* function : to process abnormal case
******************************************************************************/
void SAMPLE_LSCCALI_HandleSig(HI_S32 signo)
{
    if (SIGINT == signo || SIGTERM == signo)
    {
        SAMPLE_COMM_All_ISP_Stop();
        SAMPLE_COMM_SYS_Exit();
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
    }
    exit(-1);
}

HI_S32 get_raw_pts(VIDEO_FRAME_INFO_S** pastFrame, HI_U64 u64RawPts, HI_U32 u32RowCnt,
                   HI_U32 u32ColCnt, HI_U32 u32Col,  HI_U32* pu32Index)
{
    HI_S32 i;

    for (i = 0; i < u32RowCnt; i++)
    {
        printf("get_raw_pts  --pts is %lld.\n", pastFrame[i][u32Col].stVFrame.u64PTS);

        if (u64RawPts == pastFrame[i][u32Col].stVFrame.u64PTS)
        {
            *pu32Index = i;
            return 0;
        }
    }

    return -1;
}

HI_S32 getDumpPipe(VI_DEV ViDev, WDR_MODE_E enInWDRMode, HI_U32 *pu32PipeNum, VI_PIPE ViPipeId[])
{
    HI_S32 s32Ret;
    HI_U32 u32PipeNum, i;
    VI_DEV_BIND_PIPE_S stDevBindPipe;

    memset(&stDevBindPipe, 0, sizeof(stDevBindPipe));
    s32Ret = HI_MPI_VI_GetDevBindPipe(ViDev, &stDevBindPipe);
    if (HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_VI_GetDevBindPipe error 0x%x !\n", s32Ret);
        return s32Ret;
    }

    u32PipeNum = 0;

    switch (enInWDRMode)
    {
        case WDR_MODE_NONE:
        case WDR_MODE_BUILT_IN:
            if (stDevBindPipe.u32Num < 1)
            {
                printf("PipeNum(%d) enInWDRMode(%d) don't match !\n", stDevBindPipe.u32Num, enInWDRMode);
                return HI_FAILURE;
            }
            u32PipeNum = 1;
            ViPipeId[0] = stDevBindPipe.PipeId[0];
            ViPipeId[4] = stDevBindPipe.PipeId[0];
            break;

        case WDR_MODE_2To1_LINE:
        case WDR_MODE_2To1_FRAME:
        case WDR_MODE_2To1_FRAME_FULL_RATE:
            if (2 != stDevBindPipe.u32Num)
            {
                printf("PipeNum(%d) enInWDRMode(%d) don't match !\n", stDevBindPipe.u32Num, enInWDRMode);
                return HI_FAILURE;
            }
            u32PipeNum = 2;
            for (i = 0; i < u32PipeNum; i++)
            {
                ViPipeId[i] = stDevBindPipe.PipeId[i];
            }
            ViPipeId[4] = stDevBindPipe.PipeId[0];
            break;

        default:
            printf("enInWDRMode(%d) error !\n", enInWDRMode);
            return HI_FAILURE;
    }

    *pu32PipeNum = u32PipeNum;

    return HI_SUCCESS;
}


static inline HI_S32 bitWidth2PixelFormat(HI_U32 u32Nbit, PIXEL_FORMAT_E *penPixelFormat)
{
    PIXEL_FORMAT_E enPixelFormat;

    if (8 == u32Nbit)
    {
        enPixelFormat = PIXEL_FORMAT_RGB_BAYER_8BPP;
    }
    else if (10 == u32Nbit)
    {
        enPixelFormat = PIXEL_FORMAT_RGB_BAYER_10BPP;
    }
    else if (12 == u32Nbit)
    {
        enPixelFormat = PIXEL_FORMAT_RGB_BAYER_12BPP;
    }
    else if (14 == u32Nbit)
    {
        enPixelFormat = PIXEL_FORMAT_RGB_BAYER_14BPP;
    }
    else if (16 == u32Nbit)
    {
        enPixelFormat = PIXEL_FORMAT_RGB_BAYER_16BPP;
    }
    else
    {
        enPixelFormat = PIXEL_FORMAT_RGB_BAYER_16BPP;
    }

    *penPixelFormat = enPixelFormat;
    return HI_SUCCESS;
}

static HI_S32 pixelFormat2BitWidth(PIXEL_FORMAT_E *penPixelFormat)
{
    PIXEL_FORMAT_E enPixelFormat;
    enPixelFormat = *penPixelFormat;
    HI_S32 s32BitWidth;
    switch(enPixelFormat)
    {
    case PIXEL_FORMAT_RGB_BAYER_8BPP:
        {
            s32BitWidth = 8;
        }
        break;
    case PIXEL_FORMAT_RGB_BAYER_10BPP:
        {
            s32BitWidth = 10;
        }
        break;
    case PIXEL_FORMAT_RGB_BAYER_12BPP:
        {
            s32BitWidth = 12;
        }
        break;
    case PIXEL_FORMAT_RGB_BAYER_14BPP:
        {
            s32BitWidth = 14;
        }
        break;
    case PIXEL_FORMAT_RGB_BAYER_16BPP:
        {
            s32BitWidth = 16;
        }
        break;
    default:
        s32BitWidth = HI_FAILURE;
        break;
    }

    return s32BitWidth;
}

HI_S32 convertBitPixel(HI_U8 *pu8Data, HI_U32 u32DataNum, HI_U32 u32BitWidth, HI_U16 *pu16OutData)
{
    HI_S32 i, u32Tmp, s32OutCnt;
    HI_U32 u32Val;
    HI_U64 u64Val;
    HI_U8 *pu8Tmp = pu8Data;

    s32OutCnt = 0;
    switch(u32BitWidth)
    {
    case 10:
        {
            /* 4 pixels consist of 5 bytes  */
            u32Tmp = u32DataNum / 4;

            for (i = 0; i < u32Tmp; i++)
            {
                /* byte4 byte3 byte2 byte1 byte0 */
                pu8Tmp = pu8Data + 5 * i;
                u64Val = pu8Tmp[0] + ((HI_U32)pu8Tmp[1] << 8) + ((HI_U32)pu8Tmp[2] << 16) +
                         ((HI_U32)pu8Tmp[3] << 24) + ((HI_U64)pu8Tmp[4] << 32);

                pu16OutData[s32OutCnt++] = u64Val & 0x3ff;
                pu16OutData[s32OutCnt++] = (u64Val >> 10) & 0x3ff;
                pu16OutData[s32OutCnt++] = (u64Val >> 20) & 0x3ff;
                pu16OutData[s32OutCnt++] = (u64Val >> 30) & 0x3ff;
            }
        }
        break;
    case 12:
        {
            /* 2 pixels consist of 3 bytes  */
            u32Tmp = u32DataNum / 2;

            for (i = 0; i < u32Tmp; i++)
            {
                /* byte2 byte1 byte0 */
                pu8Tmp = pu8Data + 3 * i;
                u32Val = pu8Tmp[0] + (pu8Tmp[1] << 8) + (pu8Tmp[2] << 16);
                pu16OutData[s32OutCnt++] = u32Val & 0xfff;
                pu16OutData[s32OutCnt++] = (u32Val >> 12) & 0xfff;
            }
        }
        break;
    case 14:
        {
            /* 4 pixels consist of 7 bytes  */
            u32Tmp = u32DataNum / 4;

            for (i = 0; i < u32Tmp; i++)
            {
                pu8Tmp = pu8Data + 7 * i;
                u64Val = pu8Tmp[0] + ((HI_U32)pu8Tmp[1] << 8) + ((HI_U32)pu8Tmp[2] << 16) +
                         ((HI_U32)pu8Tmp[3] << 24) + ((HI_U64)pu8Tmp[4] << 32) +
                         ((HI_U64)pu8Tmp[5] << 40) + ((HI_U64)pu8Tmp[6] << 48);

                pu16OutData[s32OutCnt++] = u64Val & 0x3fff;
                pu16OutData[s32OutCnt++] = (u64Val >> 14) & 0x3fff;
                pu16OutData[s32OutCnt++] = (u64Val >> 28) & 0x3fff;
                pu16OutData[s32OutCnt++] = (u64Val >> 42) & 0x3fff;
            }
        }
        break;
    case 16:
        {
            /* 1 pixels consist of 2 bytes */
            u32Tmp = u32DataNum;

            for (i = 0; i < u32Tmp; i++)
            {
                /* byte1 byte0 */
                pu8Tmp = pu8Data + 2 * i;
                u32Val = pu8Tmp[0] + (pu8Tmp[1] << 8);
                pu16OutData[s32OutCnt++] = u32Val & 0xffff;
            }
        }
        break;
    default:
        fprintf(stderr, "unsupport bitWidth: %d\n", u32BitWidth);
        return HI_FAILURE;
        break;
    }

    return s32OutCnt;
}

HI_S32 mesh_calibration_proc(VI_PIPE ViPipe,VIDEO_FRAME_S* pVBuf, HI_U32 u32MeshScale, HI_U32 u32ByteAlign, ISP_MESH_SHADING_TABLE_S *pstMLSCTable)
{
    HI_U16 *pu16Data = NULL;
    HI_U64 phy_addr, size;
    HI_U8* pUserPageAddr[2];
    HI_U8  *pu8Data;
    HI_U32 u32Nbit = 0;

    HI_S32 s32Ret;
    PIXEL_FORMAT_E enPixelFormat = pVBuf->enPixelFormat;

    ISP_MLSC_CALIBRATION_CFG_S  stMLSCCaliCfg;
    ISP_PUB_ATTR_S    stIspPubAttr;
    ISP_BLACK_LEVEL_S stBlackLevel;

    u32Nbit = pixelFormat2BitWidth(&enPixelFormat);

    size = (pVBuf->u32Stride[0]) * (pVBuf->u32Height);
    phy_addr = pVBuf->u64PhyAddr[0];

    pUserPageAddr[0] = (HI_U8*) HI_MPI_SYS_Mmap(phy_addr, size);
    if (NULL == pUserPageAddr[0])
    {
        return HI_FAILURE;
    }

    if (10 != u32Nbit &&  12 != u32Nbit &&  14 != u32Nbit &&  16 != u32Nbit)
    {
        printf("can't not support %d bits raw, only support 10bits,12bits,14bits,16bits\n", u32Nbit);
        exit(HI_FAILURE);
    }

#if  DUMP_RAW_AND_SAVE_LSC
    HI_U32 u32H;
    FILE* pfd;

    printf("Dump raw frame of vi  to file: \n");

    /* open file */
    pfd = fopen("lsc.raw", "wb");
    if (NULL == pfd)
    {
        printf("open file failed:%s!\n", strerror(errno));
        return HI_FAILURE;
    }

    pu8Data = pUserPageAddr[0];
    if (8 != u32Nbit)
    {
        pu16Data = (HI_U16*)malloc(pVBuf->u32Width * 2);
        if (NULL == pu16Data)
        {
            fprintf(stderr, "alloc memory failed\n");
            HI_MPI_SYS_Munmap(pUserPageAddr[0], size);
            pUserPageAddr[0] = NULL;
            return HI_FAILURE;
        }
    }

    /* save Y ----------------------------------------------------------------*/
    fprintf(stderr, "saving......dump data......u32Stride[0]: %d, width: %d\n", pVBuf->u32Stride[0], pVBuf->u32Width);
    fflush(stderr);

    for (u32H = 0; u32H < pVBuf->u32Height; u32H++)
    {
        if (8 == u32Nbit)
        {
            fwrite(pu8Data, pVBuf->u32Width, 1, pfd);
        }
        else if (16 == u32Nbit)
        {
            fwrite(pu8Data, pVBuf->u32Width, 2, pfd);
            fflush(pfd);
        }
        else
        {
            convertBitPixel(pu8Data, pVBuf->u32Width, u32Nbit, pu16Data);
            fwrite(pu16Data, pVBuf->u32Width, 2, pfd);
        }
        pu8Data += pVBuf->u32Stride[0];
    }
    fflush(pfd);

    fprintf(stderr, "done u32TimeRef: %d!\n", pVBuf->u32TimeRef);
    fflush(stderr);
    fclose(pfd);
#endif

    if (NULL != pu16Data)
    {
        free(pu16Data);
    }

    pu8Data = pUserPageAddr[0];
    pu16Data = (HI_U16*)malloc(sizeof(HI_U16)*pVBuf->u32Width * pVBuf->u32Height);
    if (NULL == pu16Data)
    {
        fprintf(stderr, "alloc memory failed\n");
        HI_MPI_SYS_Munmap(pUserPageAddr[0], size);
        pUserPageAddr[0] = NULL;
        return HI_FAILURE;
    }

    for (u32H = 0; u32H < pVBuf->u32Height; u32H++)
    {
        convertBitPixel(pu8Data, pVBuf->u32Width, u32Nbit, pu16Data);
        pu8Data += pVBuf->u32Stride[0];
        pu16Data += pVBuf->u32Width;
    }
    pu16Data -= pVBuf->u32Width * pVBuf->u32Height;

    /*Calibration parameter preset*/
    stMLSCCaliCfg.enRawBit = u32Nbit;
    stMLSCCaliCfg.u32MeshScale = u32MeshScale;

    HI_MPI_ISP_GetPubAttr(ViPipe, &stIspPubAttr);
    stMLSCCaliCfg.enBayer  = stIspPubAttr.enBayer;
    stMLSCCaliCfg.u16ImgWidth = pVBuf->u32Width;
    stMLSCCaliCfg.u16ImgHeight = pVBuf->u32Height;

    //Default setting without crop, if need to crop, please set the right crop parameters.
    stMLSCCaliCfg.u16DstImgWidth = stMLSCCaliCfg.u16ImgWidth;
    stMLSCCaliCfg.u16DstImgHeight = stMLSCCaliCfg.u16ImgHeight;
    stMLSCCaliCfg.u16OffsetX = 0;
    stMLSCCaliCfg.u16OffsetY = 0;

    HI_MPI_ISP_GetBlackLevelAttr(ViPipe, &stBlackLevel);
    stMLSCCaliCfg.u16BLCOffsetR  = stBlackLevel.au16BlackLevel[0];
    stMLSCCaliCfg.u16BLCOffsetGr = stBlackLevel.au16BlackLevel[1];
    stMLSCCaliCfg.u16BLCOffsetGb = stBlackLevel.au16BlackLevel[2];
    stMLSCCaliCfg.u16BLCOffsetB  = stBlackLevel.au16BlackLevel[3];

    s32Ret = HI_MPI_ISP_MeshShadingCalibration(ViPipe, pu16Data, &stMLSCCaliCfg, pstMLSCTable);
    if(HI_SUCCESS != s32Ret)
    {
        if (NULL != pu16Data)
        {
            free(pu16Data);
        }
        HI_MPI_SYS_Munmap(pUserPageAddr[0], size);
        pUserPageAddr[0] = NULL;

        return HI_FAILURE;
    }

    if (NULL != pu16Data)
    {
        free(pu16Data);
    }

    HI_MPI_SYS_Munmap(pUserPageAddr[0], size);
    pUserPageAddr[0] = NULL;
    fprintf(stderr, "------Done!\n");

    return HI_SUCCESS;
}

HI_S32 lsc_online_cali_proc(VI_PIPE ViPipe, HI_U32 u32MeshScale, HI_U32 u32Cnt, HI_U32 u32ByteAlign, HI_U32 u32RatioShow)
{
    int                    i, j;
    HI_S32                 s32MilliSec = 4000;
    HI_U32                 u32CapCnt = 0;
    VIDEO_FRAME_INFO_S     astFrame[MAX_FRM_CNT];
    ISP_MESH_SHADING_TABLE_S  stIspMLSCTable;

    /* get VI frame  */
    for (i = 0; i < u32Cnt; i++)
    {
        if (HI_SUCCESS != HI_MPI_VI_GetPipeFrame(ViPipe, &astFrame[i], s32MilliSec))
        {
            printf("get vi Pipe %d frame err\n", ViPipe);
            printf("only get %d frame\n", i);
            break;
        }

        printf("get vi Pipe %d frame num %d ok\n",ViPipe,  i);
    }

    u32CapCnt = i;

    if (0 == u32CapCnt)
    {
        return HI_FAILURE;
    }

    /* dump file */
    for (j = 0; j < u32CapCnt; j++)
    {
        /* save VI frame to file */
        mesh_calibration_proc(ViPipe, &astFrame[j].stVFrame, u32MeshScale, u32ByteAlign, &stIspMLSCTable);

        /* release frame after using */
        HI_MPI_VI_ReleasePipeFrame(ViPipe, &astFrame[j]);
    }

#if DUMP_RAW_AND_SAVE_LSC
    FILE *pFile = fopen("gain.txt", "wb");
    if(!pFile)
    {
        printf("create file fails\n");
        return -1;
    }
    fprintf(pFile,"stIspShardingTable.au32XGridWidth = ");
    for(i= 0;i < 16;i++)
    {
        fprintf(pFile,"%d,",stIspMLSCTable.au16XGridWidth[i]);
    }
    fprintf(pFile,"\n");
    fprintf(pFile,"stIspShardingTable.au32YGridHeight = ");
    for(i= 0;i < 16;i++)
    {
        fprintf(pFile,"%d,",stIspMLSCTable.au16YGridWidth[i]);
    }
    fprintf(pFile,"\n");
    fprintf(pFile,"R = \n");
    for(i=0;i<33;i++)
    {
        for(j = 0;j<33;j++)
        {
            fprintf(pFile,"%d,",stIspMLSCTable.stLscGainLut.au16RGain[i*33+j]);
        }
        fprintf(pFile,"\n");
    }
    fprintf(pFile,"Gr\n");
    for(i=0;i<33;i++)
    {
        for(j = 0;j<33;j++)
        {
            fprintf(pFile,"%d,",stIspMLSCTable.stLscGainLut.au16GrGain[i*33+j]);
        }
        fprintf(pFile,"\n");
    }
    fprintf(pFile,"Gb\n");
    for(i=0;i<33;i++)
    {
        for(j = 0;j<33;j++)
        {
            fprintf(pFile,"%d,",stIspMLSCTable.stLscGainLut.au16GbGain[i*33+j]);
        }
        fprintf(pFile,"\n");
    }
    fprintf(pFile,"B\n");
    for(i=0;i<33;i++)
    {
        for(j = 0;j<33;j++)
        {
            fprintf(pFile,"%d,",stIspMLSCTable.stLscGainLut.au16BGain[i*33+j]);
        }
        fprintf(pFile,"\n");
    }

    fprintf(pFile,"\nR for BNR");
    for (i = 0; i < 129; i++)
    {
        if (!(i % 20))
            fprintf(pFile, "\n");
        fprintf(pFile, "%6d,", stIspMLSCTable.stBNRLscGainLut.au16RGain[i]);
    }
    fprintf(pFile,"\n");
    fprintf(pFile,"\nGr for BNR");
    for (i = 0; i < 129; i++)
    {
        if (!(i % 20))
            fprintf(pFile, "\n");
        fprintf(pFile, "%6d,", stIspMLSCTable.stBNRLscGainLut.au16GrGain[i]);
    }
    fprintf(pFile,"\n");
    fprintf(pFile,"\nGb for BNR");
    for (i = 0; i < 129; i++)
    {
        if (!(i % 20))
            fprintf(pFile, "\n");
        fprintf(pFile, "%6d,", stIspMLSCTable.stBNRLscGainLut.au16GbGain[i]);
    }
    fprintf(pFile,"\n");
    fprintf(pFile,"\nB for BNR");
    for (i = 0; i < 129; i++)
    {
        if (!(i % 20))
            fprintf(pFile, "\n");
        fprintf(pFile, "%6d,", stIspMLSCTable.stBNRLscGainLut.au16BGain[i]);
    }
    fprintf(pFile,"\n");

    fclose(pFile);

#endif

    return HI_SUCCESS;
}

#ifdef __HuaweiLite__
HI_S32 app_main(int argc, char* argv[])
#else
HI_S32 main(int argc, char* argv[])
#endif
{
    VI_DEV            ViDev                = 0;
    VI_PIPE           ViPipe               = 0;
    VI_PIPE           ViPipeId[4]          = {0};  /* save main pipe to [4] */
    HI_S32            s32Ret               = 0;
    HI_U32            i                    = 0;
    HI_U32            u32FrmCnt            = 1;
    HI_U32            u32RawDepth          = 2;
    HI_U32            u32ByteAlign         = 1;
    HI_U32            u32PipeNum           = 0;   /* LineMode -> 1, WDRMode -> 2~3 */
    HI_U32            u32RatioShow         = 1;
    HI_U32            u32MeshScale         = 1;
    COMPRESS_MODE_E   enCompressMode       = COMPRESS_MODE_NONE;
    VI_DUMP_ATTR_S    stRawDumpAttr;
    VI_DUMP_ATTR_S    stDumpAttr;
    VI_DEV_ATTR_S     stDevAttr;
    VI_PIPE_ATTR_S    astBackUpPipeAttr[4];
    VI_PIPE_ATTR_S    stPipeAttr;

    printf("\nNOTICE: This tool only can be used for TESTING !!!\n");
    printf("\t To see more usage, please enter: ./lsc_online_cali -h\n\n");

    if ((argc > 3) || (argc < 3))
    {
        usage();
        exit(HI_FAILURE);
    }

    ViPipe       = atoi(argv[1]);          /* pipe*/
    u32MeshScale = atoi(argv[2]);    /* Scale value of Mesh calibration*/

    if (u32MeshScale > 7)
    {
        printf("can't not support scale mode %d, can choose only from 0~7!\n", u32MeshScale);
        usage();
        exit(HI_FAILURE);
    }

#ifdef __HuaweiLite__
    LSC_CALI_PREV_S stLsc_CaliPrev;
    s32Ret = SAMPLE_LSC_CALI_START_PREV(&stLsc_CaliPrev);

    if (HI_SUCCESS == s32Ret)
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
#else
    signal(SIGINT, SAMPLE_LSCCALI_HandleSig);
    signal(SIGTERM, SAMPLE_LSCCALI_HandleSig);
#endif
    //enCompressMode = COMPRESS_MODE_NONE; /* non_compress_mode */
    u32FrmCnt      = 1;  /* frame number is 1 */
    u32ByteAlign   = 1;  /* convert to Byte align */
    u32RatioShow   = 0;  /* ratio not shown */

    if (1 > u32FrmCnt || MAX_FRM_CNT < u32FrmCnt)
    {
        printf("invalid FrmCnt %d, FrmCnt range from 1 to %d\n", u32FrmCnt, MAX_FRM_CNT);
        exit(HI_FAILURE);
    }

    s32Ret = HI_MPI_VI_GetDevAttr(ViDev, &stDevAttr);

    if (HI_SUCCESS != s32Ret)
    {
        printf("Get dev %d attr failed!\n", ViDev);
        goto EXIT;
    }

    s32Ret = getDumpPipe(ViDev, stDevAttr.stWDRAttr.enWDRMode, &u32PipeNum, ViPipeId);

    if (HI_SUCCESS != s32Ret)
    {
        printf("getDumpPipe failed 0x%x!\n", s32Ret);
        goto EXIT;
    }

    printf("Setting Parameter ==> MeshScale =%d\n", u32MeshScale);

    for (i = 0; i < u32PipeNum; i++)
    {
        s32Ret = HI_MPI_VI_GetPipeDumpAttr(ViPipeId[i], &stRawDumpAttr);

        if (HI_SUCCESS != s32Ret)
        {
            printf("Get Pipe %d dump attr failed!\n", ViPipe);
            goto EXIT;
        }

        memcpy(&stDumpAttr, &stRawDumpAttr, sizeof(VI_DUMP_ATTR_S));
        stDumpAttr.bEnable                       = HI_TRUE;
        stDumpAttr.u32Depth                      = u32RawDepth;
        s32Ret = HI_MPI_VI_SetPipeDumpAttr(ViPipeId[i], &stDumpAttr);

        if (HI_SUCCESS != s32Ret)
        {
            printf("Set Pipe %d dump attr failed!\n", ViPipeId[i]);
            goto EXIT;
        }

        s32Ret = HI_MPI_VI_GetPipeAttr(ViPipeId[i], &astBackUpPipeAttr[i]);

        if (HI_SUCCESS != s32Ret)
        {
            printf("Get Pipe %d attr failed!\n", ViPipe);
            goto EXIT;
        }

        memcpy(&stPipeAttr, &astBackUpPipeAttr[i], sizeof(VI_PIPE_ATTR_S));
        stPipeAttr.enCompressMode = enCompressMode;
        s32Ret = HI_MPI_VI_SetPipeAttr(ViPipeId[i], &stPipeAttr);

        if (HI_SUCCESS != s32Ret)
        {
            printf("Set Pipe %d attr failed!\n", ViPipe);
            goto EXIT;
        }
    }

    sleep(1);
    printf("--> u32PipeNum=%d\n", u32PipeNum);

    if (1 == u32PipeNum || 1 == u32FrmCnt)
    {
        lsc_online_cali_proc(ViPipe, u32MeshScale, u32FrmCnt, u32ByteAlign, u32RatioShow);
    }
    else
    {
        printf("Please check if PipeNum is equal to 1!\n");
        exit(HI_FAILURE);
    }


    for (i = 0; i < u32PipeNum; i++)
    {
        s32Ret = HI_MPI_VI_SetPipeAttr(ViPipeId[i], &astBackUpPipeAttr[i]);

        if (HI_SUCCESS != s32Ret)
        {
            printf("Set Pipe %d attr failed!\n", ViPipe);
            goto EXIT;
        }

        s32Ret = HI_MPI_VI_SetPipeDumpAttr(ViPipeId[i], &stRawDumpAttr);

        if (HI_SUCCESS != s32Ret)
        {
            printf("Set Pipe %d dump attr failed!\n", ViPipe);
            goto EXIT;
        }
    }
EXIT:

#ifdef __HuaweiLite__
    printf("input anything to exit....\n");
    getchar();
    SAMPLE_LSC_CALI_STOP_PREV(&stLsc_CaliPrev);
#endif
    return s32Ret;
}
