#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "sample_comm.h"
#include "hi_mipi_tx.h"
#include "sample_vo.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#ifdef __HuaweiLite__
#define RES_PATH "/sharefs/yuv/"
#else
#define RES_PATH "../res/"
#endif

#define YUV_320_240   RES_PATH"320_240_420.yuv"
#define YUV_1920_1080 RES_PATH"1080_1920_420.yuv"

typedef struct stSAMPLE_VOU_ThreadCtrl_Info {
    HI_CHAR          filename[1024];
    HI_U32           u32Width;
    HI_U32           u32Height;
    PIXEL_FORMAT_E   enPixelFmt;
    VIDEO_FORMAT_E   enVideoFmt;
    HI_BOOL          bQuit;
    HI_BOOL          bDestroy;
    HI_S32           s32ToDev;
    DYNAMIC_RANGE_E  enSrcDynamicRange;
    HI_BOOL          abToChn[VO_MAX_CHN_NUM];
    COLOR_GAMUT_E    enColrGamut;
    HI_U32           u32ChnNum;

    pthread_t  tid;

} SAMPLE_VOU_ThreadCtrl_Info;

typedef struct stSAMPLE_USER_VO_CONFIG_S {
    VO_SYNC_INFO_S stSyncInfo;
    VO_USER_INTFSYNC_ATTR_S stUserIntfSyncAttr;
    HI_U32 u32PreDiv;
    HI_U32 u32DevDiv;
    HI_U32 u32Framerate;
    combo_dev_cfg_t stcombo_dev_cfgl;
}SAMPLE_USER_VO_CONFIG_S;

#define ALIGN_BACK(x, a)        ((a) * (((x + a -1) / (a))))
#define ALIGN_UP(x, a)           ( ( ((x) + ((a) - 1) ) / a ) * a )

#define SAMPLE_CHECK_RET(express,name)\
    do {\
        HI_S32 Ret;\
        Ret = express;\
        if (Ret != HI_SUCCESS) {\
            printf("%s failed at %s : LINE: %d with %#x!\n",name, __FUNCTION__,__LINE__,Ret);\
            SAMPLE_VOU_SYS_Exit();\
            return Ret;\
            }\
        }while(0)

#define VO_VB_PIC_BLK_SIZE(Width, Height, Type, size)\
    do{\
            unsigned int u32AlignWidth;\
            unsigned int u32AlignHeight;\
            unsigned int u32HeadSize;\
            u32AlignWidth = ALIGN_UP(Width, 16);\
            u32AlignHeight= ALIGN_UP(Height, 2);\
            u32HeadSize = 16 * u32AlignHeight;/* compress header stride 16 */\
            if (Type == PIXEL_FORMAT_YVU_SEMIPLANAR_422)\
            {\
                size = (u32AlignWidth * u32AlignHeight + u32HeadSize) * 2;\
            }\ else if (Type == PIXEL_FORMAT_YUV_400)\
            {\
                size = (u32AlignWidth * u32AlignHeight + u32HeadSize);\
            }\
            else\
            {\
                size = ((u32AlignWidth * u32AlignHeight + u32HeadSize) * 3) >> 1;\
            }\
    }while(0)


HI_S32 SAMPLE_VOU_SYS_Init(void)
{
    VB_CONFIG_S stVbConf = {0};
    HI_U32      u32BlkSize;

    HI_MPI_SYS_Exit();

    HI_MPI_VB_Exit();

    stVbConf.u32MaxPoolCnt = 64;

    u32BlkSize = COMMON_GetPicBufferSize(1920, 1080, SAMPLE_PIXEL_FORMAT, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
    stVbConf.astCommPool[0].u64BlkSize  = u32BlkSize;
    stVbConf.astCommPool[0].u32BlkCnt   = 20;

    u32BlkSize = COMMON_GetPicBufferSize(720, 576, SAMPLE_PIXEL_FORMAT, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
    stVbConf.astCommPool[1].u64BlkSize  = u32BlkSize;
    stVbConf.astCommPool[1].u32BlkCnt   = 20;

    SAMPLE_CHECK_RET(SAMPLE_COMM_SYS_Init(&stVbConf),"SAMPLE_COMM_SYS_Init");

    return HI_SUCCESS;
}

HI_VOID SAMPLE_VOU_SYS_Exit(void)
{
    HI_MPI_SYS_Exit();
    HI_MPI_VB_Exit();
}

void SAMPLE_VIO_HandleSig(HI_S32 signo)
{
    signal(SIGINT, SIG_IGN);
    signal(SIGTERM, SIG_IGN);

    if (SIGINT == signo || SIGTERM == signo) {
        SAMPLE_COMM_SYS_Exit();
        SAMPLE_PRT("\033[0;31mprogram termination abnormally!\033[0;39m\n");
    }
    exit(-1);
}

HI_S32 SAMPLE_VOU_ReadOneFrame( FILE * fp, HI_U8 * pY, HI_U8 * pU, HI_U8 * pV,
                                              HI_U32 width, HI_U32 height, HI_U32 stride, HI_U32 stride2,
                                              PIXEL_FORMAT_E enPixFrm)
{
    HI_U8 * pDst;
    HI_U32 u32UVHeight;
    HI_U32 u32Row;

    if(enPixFrm == PIXEL_FORMAT_YVU_SEMIPLANAR_422) {
        u32UVHeight = height;
    } else {
        u32UVHeight = height / 2;
    }

    pDst = pY;
    for ( u32Row = 0; u32Row < height; u32Row++ ) {
        if (fread( pDst, 1, width, fp ) != width) {
            return -1;
        }
        pDst += stride;
    }

    pDst = pU;
    for ( u32Row = 0; u32Row < u32UVHeight; u32Row++ ) {
        if(fread( pDst, 1, width/2, fp ) != width/2) {
            return -2;
        }
        pDst += stride2;
    }

    pDst = pV;
    for ( u32Row = 0; u32Row < u32UVHeight; u32Row++ ) {
        if(fread( pDst, 1, width/2, fp ) != width/2) {
            return -3;
        }
        pDst += stride2;
    }

   return HI_SUCCESS;
}

HI_S32 SAMPLE_VO_PlanToSemi(HI_U8 *pY, HI_S32 yStride,
                       HI_U8 *pU, HI_S32 uStride,
                       HI_U8 *pV, HI_S32 vStride,
                       HI_S32 picWidth, HI_S32 picHeight, PIXEL_FORMAT_E enPixFrm)
{
    HI_S32 i;
    HI_U8* pTmpU, *ptu;
    HI_U8* pTmpV, *ptv;

    HI_S32 s32HafW = uStride >>1 ;
    HI_S32 s32HafH;

    if(enPixFrm == PIXEL_FORMAT_YVU_SEMIPLANAR_422) {
        s32HafH = picHeight;
    } else {
        s32HafH = picHeight >>1 ;
    }

    HI_S32 s32Size = s32HafW*s32HafH;

    pTmpU = malloc( s32Size ); ptu = pTmpU;
    pTmpV = malloc( s32Size ); ptv = pTmpV;

    memcpy(pTmpU,pU,s32Size);
    memcpy(pTmpV,pV,s32Size);

    for(i = 0;i<s32Size>>1;i++) {
        *pU++ = *pTmpV++;
        *pU++ = *pTmpU++;

    }
    for(i = 0;i<s32Size>>1;i++) {
        *pV++ = *pTmpV++;
        *pV++ = *pTmpU++;
    }

    free( ptu );
    free( ptv );

    return HI_SUCCESS;
}

HI_VOID *SAMPLE_VO_FileVO(HI_VOID* pData)
{
    HI_S32 i;
    HI_S32 s32Ret;
    FILE *pfd;
    VB_BLK hBlkHdl;
    HI_U32 u32Size;
    HI_U32 u32SrcWidth;
    HI_U32 u32SrcHeight;
    VB_POOL Pool;
    VIDEO_FRAME_INFO_S stUserFrame;
    VB_POOL_CONFIG_S stVbPoolCfg;
    HI_U32 u32LumaSize = 0;
    HI_U32 u32ChromaSize = 0;

    SAMPLE_VOU_ThreadCtrl_Info *pInfo = (SAMPLE_VOU_ThreadCtrl_Info*)pData;
    VO_LAYER VoLayer = pInfo->s32ToDev;
    memset(&stUserFrame,0x0,sizeof(VIDEO_FRAME_INFO_S));

    u32SrcWidth = pInfo->u32Width;
    u32SrcHeight = pInfo->u32Height;

    pfd = fopen(pInfo->filename,"rb");
    if (pfd == HI_NULL) {
        printf("open file %s fail \n", pInfo->filename);
        return HI_NULL;
    } else {
        printf("open file %s success!\n", pInfo->filename);
    }

    fflush(stdout);

    u32Size = u32SrcWidth * u32SrcHeight * 2;
    memset(&stVbPoolCfg, 0, sizeof(VB_POOL_CONFIG_S));
    stVbPoolCfg.u64BlkSize = u32Size;
    stVbPoolCfg.u32BlkCnt = 10;
    stVbPoolCfg.enRemapMode = VB_REMAP_MODE_NONE;
    Pool = HI_MPI_VB_CreatePool(&stVbPoolCfg);
    if (Pool == VB_INVALID_POOLID) {
        printf("Maybe you not call sys init\n");
        return HI_NULL;
    }

    stUserFrame.stVFrame.enField = VIDEO_FIELD_INTERLACED;
    stUserFrame.stVFrame.enCompressMode = COMPRESS_MODE_NONE;
    stUserFrame.stVFrame.enPixelFormat = pInfo->enPixelFmt;
    stUserFrame.stVFrame.enVideoFormat = pInfo->enVideoFmt;
    stUserFrame.stVFrame.enColorGamut = COLOR_GAMUT_BT709;
    stUserFrame.stVFrame.u32Width = u32SrcWidth;
    stUserFrame.stVFrame.u32Height = u32SrcHeight;
    stUserFrame.stVFrame.u32Stride[0] = ALIGN_BACK(u32SrcWidth, 16);
    stUserFrame.stVFrame.u32Stride[1] = ALIGN_BACK(u32SrcWidth, 16);
    stUserFrame.stVFrame.u32Stride[2] = ALIGN_BACK(u32SrcWidth, 16);
    stUserFrame.stVFrame.u32TimeRef = 0;
    stUserFrame.stVFrame.u64PTS = 0;
    stUserFrame.stVFrame.enDynamicRange = DYNAMIC_RANGE_SDR8;

    u32LumaSize =  stUserFrame.stVFrame.u32Stride[0] * u32SrcHeight;
    if (pInfo->enPixelFmt == PIXEL_FORMAT_YVU_SEMIPLANAR_422) {
        u32ChromaSize =  stUserFrame.stVFrame.u32Stride[0] * u32SrcHeight / 2;
    } else if ((pInfo->enPixelFmt == PIXEL_FORMAT_YVU_SEMIPLANAR_420)) {
        u32ChromaSize =  stUserFrame.stVFrame.u32Stride[0] * u32SrcHeight / 4;
    } else if ((pInfo->enPixelFmt == PIXEL_FORMAT_YUV_400)) {
        u32ChromaSize =  0;
    }

    do {
        if (feof(pfd) != 0) {
            fseek(pfd, 0, SEEK_SET);
        }

        hBlkHdl = HI_MPI_VB_GetBlock( Pool, u32Size, NULL);
        if (hBlkHdl == VB_INVALID_HANDLE) {
            printf("[VOU_MST_File2VO] get vb fail!!!\n");
            sleep(1);
            continue;
        }

        stUserFrame.u32PoolId = HI_MPI_VB_Handle2PoolId(hBlkHdl);
        stUserFrame.stVFrame.u64PhyAddr[0] = HI_MPI_VB_Handle2PhysAddr( hBlkHdl );
        stUserFrame.stVFrame.u64PhyAddr[1] = stUserFrame.stVFrame.u64PhyAddr[0] + u32LumaSize;
        stUserFrame.stVFrame.u64PhyAddr[2] = stUserFrame.stVFrame.u64PhyAddr[1] + u32ChromaSize;

        stUserFrame.stVFrame.u64VirAddr[0] = (HI_UL)HI_MPI_SYS_Mmap(stUserFrame.stVFrame.u64PhyAddr[0], u32Size);
        stUserFrame.stVFrame.u64VirAddr[1] = (HI_UL)(stUserFrame.stVFrame.u64VirAddr[0]) + u32LumaSize;
        stUserFrame.stVFrame.u64VirAddr[2] = (HI_UL)(stUserFrame.stVFrame.u64VirAddr[1]) + u32ChromaSize;

        if(pInfo->enPixelFmt != PIXEL_FORMAT_YUV_400) {
           s32Ret = SAMPLE_VOU_ReadOneFrame( pfd, (HI_U8*)(HI_UL)stUserFrame.stVFrame.u64VirAddr[0],
                                       (HI_U8*)(HI_UL)stUserFrame.stVFrame.u64VirAddr[1], (HI_U8*)(HI_UL)stUserFrame.stVFrame.u64VirAddr[2],
                                       stUserFrame.stVFrame.u32Width, stUserFrame.stVFrame.u32Height,
                                       stUserFrame.stVFrame.u32Stride[0], stUserFrame.stVFrame.u32Stride[1] >> 1,
                                       stUserFrame.stVFrame.enPixelFormat);
           if(s32Ret == HI_SUCCESS) {
               SAMPLE_VO_PlanToSemi( (HI_U8*)(HI_UL)stUserFrame.stVFrame.u64VirAddr[0], stUserFrame.stVFrame.u32Stride[0],
                                                (HI_U8*)(HI_UL)stUserFrame.stVFrame.u64VirAddr[1], stUserFrame.stVFrame.u32Stride[1],
                                                (HI_U8*)(HI_UL)stUserFrame.stVFrame.u64VirAddr[2], stUserFrame.stVFrame.u32Stride[1],
                                                stUserFrame.stVFrame.u32Width,    stUserFrame.stVFrame.u32Height,
                                                stUserFrame.stVFrame.enPixelFormat);
           } else {
               goto OUT;
           }
        } else {
             s32Ret = SAMPLE_VOU_ReadOneFrame( pfd, (HI_U8*)(HI_UL)stUserFrame.stVFrame.u64VirAddr[0],
                                       (HI_U8*)(HI_UL)stUserFrame.stVFrame.u64VirAddr[1], (HI_U8*)(HI_UL)stUserFrame.stVFrame.u64VirAddr[2],
                                       stUserFrame.stVFrame.u32Width, stUserFrame.stVFrame.u32Height,
                                       stUserFrame.stVFrame.u32Stride[0], stUserFrame.stVFrame.u32Stride[1] >> 1,
                                       stUserFrame.stVFrame.enPixelFormat);
         }

        stUserFrame.stVFrame.u64PTS += 40000;
        stUserFrame.stVFrame.u32TimeRef += 40000;

        for (i = 0; i < pInfo->u32ChnNum; i++) {
            s32Ret = HI_MPI_VO_SendFrame(VoLayer, i , &stUserFrame, 0);
        }

OUT:
        HI_MPI_VB_ReleaseBlock(hBlkHdl);
        HI_MPI_SYS_Munmap((HI_VOID*)(HI_UL)stUserFrame.stVFrame.u64VirAddr[0], u32Size);

        }while(pInfo->bQuit == HI_FALSE);

    while (pInfo->bDestroy == HI_FALSE) {
            ;
    }

    fclose(pfd);

    return NULL;
}

HI_VOID SAMPLE_VO_Get_USER_CONFIG(SAMPLE_USER_VO_CONFIG_S *pstUserVoConfig,combo_dev_cfg_t *pstComboDevCfg)
{
    /* USER SET SYNCINFO CONFIG */
    pstUserVoConfig->stSyncInfo.u16Vact = 1080;
    pstUserVoConfig->stSyncInfo.u16Vbb = 28;
    pstUserVoConfig->stSyncInfo.u16Vfb = 130;
    pstUserVoConfig->stSyncInfo.u16Vpw = 10;
    pstUserVoConfig->stSyncInfo.u16Hact = 1920;
    pstUserVoConfig->stSyncInfo.u16Hbb  = 36;
    pstUserVoConfig->stSyncInfo.u16Hfb = 16;
    pstUserVoConfig->stSyncInfo.u16Hpw = 8;

    /* USER SET FRAME TARE */
    pstUserVoConfig->u32Framerate = 60;

    /* USER SET Div INFOMATION */
    pstUserVoConfig->u32PreDiv = 1;
    pstUserVoConfig->u32DevDiv = 1;

    /* USER SET INTFSYNC ATTR */
    pstUserVoConfig->stUserIntfSyncAttr.enClkSource = VO_CLK_SOURCE_PLL;
    pstUserVoConfig->stUserIntfSyncAttr.stUserSyncPll.u32Fbdiv = 73;
    pstUserVoConfig->stUserIntfSyncAttr.stUserSyncPll.u32Frac = 0x7aebc4;
    pstUserVoConfig->stUserIntfSyncAttr.stUserSyncPll.u32Refdiv = 2;
    pstUserVoConfig->stUserIntfSyncAttr.stUserSyncPll.u32Postdiv1 = 3;
    pstUserVoConfig->stUserIntfSyncAttr.stUserSyncPll.u32Postdiv2 = 2;

    /* USER SET MIPI ATTR */
    pstComboDevCfg->devno = 0;
    pstComboDevCfg->lane_id[0] = 0;
    pstComboDevCfg->lane_id[1] = 1;
    pstComboDevCfg->lane_id[2] = 2;
    pstComboDevCfg->lane_id[3] = 3;
    pstComboDevCfg->output_format = OUTPUT_MODE_DSI_VIDEO;
    pstComboDevCfg->output_mode = OUT_FORMAT_RGB_24_BIT;
    pstComboDevCfg->video_mode = BURST_MODE;
    pstComboDevCfg->phy_data_rate = 879;
    pstComboDevCfg->pixel_clk = 146481;
    pstComboDevCfg->sync_info.vid_pkt_size = 1920;
    pstComboDevCfg->sync_info.vid_hbp_pixels = 28;
    pstComboDevCfg->sync_info.vid_hsa_pixels = 8;
    pstComboDevCfg->sync_info.vid_hline_pixels = 1972;
    pstComboDevCfg->sync_info.vid_active_lines = 1080;
    pstComboDevCfg->sync_info.vid_vbp_lines = 18;
    pstComboDevCfg->sync_info.vid_vfp_lines = 130;
    pstComboDevCfg->sync_info.vid_vsa_lines = 10;

    return;

}

HI_VOID SAMPLE_VO_GetUserPubBaseAttr(VO_PUB_ATTR_S *pstPubAttr)
{
    pstPubAttr->u32BgColor = COLOR_RGB_BLUE;
    pstPubAttr->enIntfSync = VO_OUTPUT_USER;
    pstPubAttr->stSyncInfo.bSynm = 0;
    pstPubAttr->stSyncInfo.u8Intfb = 0;
    pstPubAttr->stSyncInfo.bIop = 1;

    pstPubAttr->stSyncInfo.u16Hmid = 1;
    pstPubAttr->stSyncInfo.u16Bvact = 1;
    pstPubAttr->stSyncInfo.u16Bvbb = 1;
    pstPubAttr->stSyncInfo.u16Bvfb = 1;

    pstPubAttr->stSyncInfo.bIdv = 0;
    pstPubAttr->stSyncInfo.bIhs = 0;
    pstPubAttr->stSyncInfo.bIvs = 0;

    return;
}

HI_VOID SAMPLE_VO_GetUserLayerAttr(VO_VIDEO_LAYER_ATTR_S *pstLayerAttr,SIZE_S  *pstDevSize)
{
    pstLayerAttr->bClusterMode = HI_FALSE;
    pstLayerAttr->bDoubleFrame = HI_FALSE;
    pstLayerAttr->enDstDynamicRange = DYNAMIC_RANGE_SDR8;
    pstLayerAttr->enPixFormat = PIXEL_FORMAT_YVU_SEMIPLANAR_420;

    pstLayerAttr->stDispRect.s32X = 0;
    pstLayerAttr->stDispRect.s32Y = 0;
    pstLayerAttr->stDispRect.u32Height = pstDevSize->u32Height;
    pstLayerAttr->stDispRect.u32Width  = pstDevSize->u32Width;

    pstLayerAttr->stImageSize.u32Height = pstDevSize->u32Height;
    pstLayerAttr->stImageSize.u32Width = pstDevSize->u32Width;

    return;
}

HI_VOID SAMPLE_VO_GetUserChnAttr(VO_CHN_ATTR_S *pstChnAttr,SIZE_S *pstDevSize,HI_S32 VoChnNum)
{
    HI_S32 i;
    for (i = 0; i < VoChnNum; i++) {
        pstChnAttr[i].bDeflicker = HI_FALSE;
        pstChnAttr[i].u32Priority = 0;
        pstChnAttr[i].stRect.s32X = 0;
        pstChnAttr[i].stRect.s32Y = 0;
        pstChnAttr[i].stRect.u32Height = pstDevSize->u32Height;
        pstChnAttr[i].stRect.u32Width = pstDevSize->u32Width;
        }

    return;
}

HI_VOID SAMPLE_VO_GetBaseThreadInfo(SAMPLE_VOU_ThreadCtrl_Info *pstThreadInfo,SIZE_S *pstFrmSize)
{
    pstThreadInfo->bDestroy = HI_FALSE;
    pstThreadInfo->bQuit    = HI_FALSE;
    pstThreadInfo->enColrGamut = COLOR_GAMUT_BT709;
    pstThreadInfo->enPixelFmt = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    pstThreadInfo->enVideoFmt = VIDEO_FORMAT_LINEAR;
    pstThreadInfo->u32Width = pstFrmSize->u32Width;
    pstThreadInfo->u32Height = pstFrmSize->u32Height;

    return;
}

HI_VOID SAMPLE_VO_StartUserThd(SAMPLE_VOU_ThreadCtrl_Info *pstThreadInfo,HI_S32 VoLayer,HI_S32 VoChnNum,
    HI_CHAR filename[256],SIZE_S *pstFrmSize)
{
    /* CREATE USER THREAD */
    SAMPLE_VO_GetBaseThreadInfo(pstThreadInfo,pstFrmSize);

    pstThreadInfo->s32ToDev = VoLayer;
    pstThreadInfo->u32ChnNum = VoChnNum;

    strncpy(pstThreadInfo->filename, filename, sizeof(pstThreadInfo->filename) - 1);
    pthread_create(&pstThreadInfo->tid, NULL, SAMPLE_VO_FileVO, (HI_VOID*)pstThreadInfo);

    return;
}

HI_VOID SAMPLE_VO_StopUserThd(SAMPLE_VOU_ThreadCtrl_Info *pstThdInfo)
{
    pstThdInfo->bQuit = HI_TRUE;
    pstThdInfo->bDestroy = HI_TRUE;
    pthread_join(pstThdInfo->tid,HI_NULL);

    return;
}

HI_S32 SAMPLE_OPEN_MIPITx_FD(HI_VOID)
{
    HI_S32 fd;

    fd = open("/dev/hi_mipi_tx", O_RDWR);
    if (fd < 0) {
        printf("open hi_mipi_tx dev failed\n");
    }

    return fd;
}

HI_VOID SAMPLE_CLOSE_MIPITx_FD(HI_S32 fd)
{
    close(fd);
    return;
}

HI_VOID SAMPLE_GetMipiTxConfig(combo_dev_cfg_t *pstMipiTxConfig)
{
    /* USER NEED SET MIPI DEV CONFIG */
    pstMipiTxConfig->devno = 0;
    pstMipiTxConfig->lane_id[0] = 0;
    pstMipiTxConfig->lane_id[1] = 1;
    pstMipiTxConfig->lane_id[2] = 2;
    pstMipiTxConfig->lane_id[3] = 3;
    pstMipiTxConfig->output_mode = OUTPUT_MODE_DSI_VIDEO;
    pstMipiTxConfig->output_format = OUT_FORMAT_RGB_24_BIT;
    pstMipiTxConfig->video_mode = BURST_MODE;
    pstMipiTxConfig->sync_info.vid_pkt_size = 1080;
    pstMipiTxConfig->sync_info.vid_hsa_pixels = 8;
    pstMipiTxConfig->sync_info.vid_hbp_pixels = 20;
    pstMipiTxConfig->sync_info.vid_hline_pixels = 1238;
    pstMipiTxConfig->sync_info.vid_vsa_lines = 10;
    pstMipiTxConfig->sync_info.vid_vbp_lines = 26;
    pstMipiTxConfig->sync_info.vid_vfp_lines = 16;
    pstMipiTxConfig->sync_info.vid_active_lines = 1920;
    pstMipiTxConfig->sync_info.edpi_cmd_size = 0;
    pstMipiTxConfig->phy_data_rate = 945;
    pstMipiTxConfig->pixel_clk = 148500;

    return;
}

HI_S32 SAMPLE_SetMipiTxConfig(HI_S32 fd,combo_dev_cfg_t *pstMipiTxConfig)
{
    HI_S32 s32Ret;
    s32Ret = ioctl(fd, HI_MIPI_TX_SET_DEV_CFG, pstMipiTxConfig);
    if (s32Ret != HI_SUCCESS) {
        printf("MIPI_TX SET_DEV_CONFIG failed\n");
        SAMPLE_CLOSE_MIPITx_FD(fd);
        return s32Ret;
    }
    return s32Ret;
}

HI_S32 SAMPLE_SET_MIPITx_Dev_ATTR(HI_S32 fd)
{
    HI_S32 s32Ret;
    combo_dev_cfg_t stMipiTxConfig;

    /* USER SET MIPI DEV CONFIG */
    SAMPLE_GetMipiTxConfig(&stMipiTxConfig);

    /* USER SET MIPI DEV CONFIG */
    s32Ret = SAMPLE_SetMipiTxConfig(fd,&stMipiTxConfig);

    return s32Ret;
}

HI_S32 SAMPLE_USER_INIT_MIPITx(HI_S32 fd,cmd_info_t *pcmd_info)
{
    HI_S32 s32Ret;

    s32Ret = ioctl(fd, HI_MIPI_TX_SET_CMD, pcmd_info);
    if (s32Ret !=  HI_SUCCESS) {
        printf("MIPI_TX SET CMD failed\n");
        SAMPLE_CLOSE_MIPITx_FD(fd);
        return s32Ret;
     }

    return HI_SUCCESS;
}

HI_S32 SAMPLE_VO_INIT_MIPITx_Screen(HI_S32 fd)
{
    HI_S32 s32Ret;
    cmd_info_t cmd_info;

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0xeeff;
    cmd_info.data_type = 0x23;
    cmd_info.cmd = NULL;
    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(1000);

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0x4018;
    cmd_info.data_type = 0x23;
    cmd_info.cmd = NULL;
    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(10000);

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0x18;
    cmd_info.data_type = 0x23;
    cmd_info.cmd = NULL;
    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(1000);

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0xff;
    cmd_info.data_type = 0x23;
    cmd_info.cmd = NULL;
    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(1000);

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0x1fb;
    cmd_info.data_type = 0x23;
    cmd_info.cmd = NULL;
    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(1000);

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0x135;
    cmd_info.data_type = 0x23;
    cmd_info.cmd = NULL;
    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(1000);

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0xff51;
    cmd_info.data_type = 0x23;
    cmd_info.cmd = NULL;
    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(1000);

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0x2c53;
    cmd_info.data_type = 0x23;
    cmd_info.cmd = NULL;
    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(1000);

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0x155;
    cmd_info.data_type = 0x23;
    cmd_info.cmd = NULL;
    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(1000);

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0x24d3;
    cmd_info.data_type = 0x23;
    cmd_info.cmd = NULL;
    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(10000);

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0x10d4;
    cmd_info.data_type = 0x23;
    cmd_info.cmd = NULL;

    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(10000);

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0x11;
    cmd_info.data_type = 0x05;
    cmd_info.cmd = NULL;

    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(200000);

    cmd_info.devno = 0;
    cmd_info.cmd_size = 0x29;
    cmd_info.data_type = 0x05;
    cmd_info.cmd = NULL;
    s32Ret = SAMPLE_USER_INIT_MIPITx(fd,&cmd_info) ;
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    usleep(200000);

    return HI_SUCCESS;
}

HI_S32 SAMPLE_VO_ENABLE_MIPITx(HI_S32 fd)
{
    HI_S32 s32Ret;
    s32Ret = ioctl(fd, HI_MIPI_TX_ENABLE);
    if (s32Ret != HI_SUCCESS) {
        printf("MIPI_TX enable failed\n");
        return s32Ret;
    }

    return s32Ret;
}

HI_S32 SAMPLE_VO_CONFIG_MIPI(HI_VOID)
{
    HI_S32 fd;
    HI_S32 s32Ret;
    /* SET MIPI BAKCLIGHT */

    /* CONFIG MIPI PINUMX */

    /* Reset MIPI */

    /* OPEN MIPI FD */
    fd = SAMPLE_OPEN_MIPITx_FD();
    if (fd < 0) {
        return HI_FAILURE;
    }

    /* SET MIPI Tx Dev ATTR */
    s32Ret = SAMPLE_SET_MIPITx_Dev_ATTR(fd);
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }
    /* CONFIG MIPI Tx INITIALIZATION SEQUENCE */
    s32Ret = SAMPLE_VO_INIT_MIPITx_Screen(fd);
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    /* ENABLE MIPI Tx DEV */
    s32Ret = SAMPLE_VO_ENABLE_MIPITx(fd);
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    return HI_SUCCESS;
}

HI_S32 SAMPLE_VO_RGBLCD_6BIT(HI_VOID)
{
    HI_U32 i = 0;
    HI_S32 VoDev = 0;
    HI_S32 VoLayer = 0;
    HI_S32 VoChnNum = 1;
    VO_PUB_ATTR_S stPubAttr;
    VO_VIDEO_LAYER_ATTR_S stLayerAttr;
    VO_USER_INTFSYNC_INFO_S stUserInfo;
    HI_U32 u32Framerate;
    SIZE_S stDevSize;

    SAMPLE_VOU_ThreadCtrl_Info stThreadInfo;

    VO_CHN_ATTR_S astChnAttr[VO_MAX_CHN_NUM];

    HI_CHAR filename[256] = YUV_320_240;
    SIZE_S stFrameSize    = {320, 240};

    SAMPLE_CHECK_RET(SAMPLE_VOU_SYS_Init(),"SAMPLE_VOU_SYS_Init");

    /* SET VO PUB ATTR OF USER TYPE */
    SAMPLE_VO_GetUserPubBaseAttr(&stPubAttr);

    stPubAttr.enIntfType = VO_INTF_LCD_6BIT;

    /* SET VO DEV SYNC INFO */
    stPubAttr.stSyncInfo.u16Hact = 240;
    stPubAttr.stSyncInfo.u16Hbb = 30;
    stPubAttr.stSyncInfo.u16Hfb = 10;
    stPubAttr.stSyncInfo.u16Hpw = 10;
    stPubAttr.stSyncInfo.u16Vact = 320;
    stPubAttr.stSyncInfo.u16Vbb = 10;
    stPubAttr.stSyncInfo.u16Vfb = 4;
    stPubAttr.stSyncInfo.u16Vpw = 2;

    SAMPLE_CHECK_RET(HI_MPI_VO_SetPubAttr(VoDev, &stPubAttr),"HI_MPI_VO_SetPubAttr");

    /* SET VO FRAME RATE */
    u32Framerate = 50;

    SAMPLE_CHECK_RET(HI_MPI_VO_SetDevFrameRate(VoDev, u32Framerate),"HI_MPI_VO_SetDevFrameRate");

    /* SET VO SYNC INFO OF USER INTF */
    stUserInfo.bClkReverse = HI_TRUE;
    stUserInfo.u32DevDiv = 3;
    stUserInfo.u32PreDiv = 1;
    stUserInfo.stUserIntfSyncAttr.enClkSource = VO_CLK_SOURCE_LCDMCLK;
    stUserInfo.stUserIntfSyncAttr.u32LcdMClkDiv = 0x182ed6;

    SAMPLE_CHECK_RET(HI_MPI_VO_SetUserIntfSyncInfo(VoDev, &stUserInfo),"HI_MPI_VO_SetUserIntfSyncInfo");

    /* ENABLE VO DEV */
    SAMPLE_CHECK_RET(HI_MPI_VO_Enable(VoDev),"HI_MPI_VO_Enable");

    /* SET VO DISPLAY BUFFER LENGTH */
    SAMPLE_CHECK_RET(HI_MPI_VO_SetDisplayBufLen(VoDev, 3),"HI_MPI_VO_SetDisplayBufLen");

    /* SET VO LAYER ATTR */
    stDevSize.u32Width= 240;
    stDevSize.u32Height = 320;

    SAMPLE_VO_GetUserLayerAttr(&stLayerAttr,&stDevSize);
    stLayerAttr.u32DispFrmRt = 50;

    SAMPLE_CHECK_RET(HI_MPI_VO_SetVideoLayerAttr(VoLayer, &stLayerAttr),"HI_MPI_VO_SetVideoLayerAttr");

    /*ENABLE VO LAYER */
    SAMPLE_CHECK_RET(HI_MPI_VO_EnableVideoLayer(VoLayer),"HI_MPI_VO_EnableVideoLayer");

    /* SET AND ENABLE VO CHN */
    SAMPLE_VO_GetUserChnAttr(astChnAttr,&stDevSize,VoChnNum);

    for (i = 0; i < VoChnNum; i++) {
        SAMPLE_CHECK_RET(HI_MPI_VO_SetChnAttr(VoLayer, i, &astChnAttr[i]),"HI_MPI_VO_SetChnAttr");

        SAMPLE_CHECK_RET(HI_MPI_VO_EnableChn(VoLayer, i),"HI_MPI_VO_EnableChn");
    }

    /* START USER THREAD */
    SAMPLE_VO_StartUserThd(&stThreadInfo, VoLayer, VoChnNum,filename,&stFrameSize);

    PAUSE();

    /* STOP USER THREAD */
    SAMPLE_VO_StopUserThd(&stThreadInfo);

    /* DISABLE VO CHN */
    for (i=0; i< VoChnNum; i++) {
        SAMPLE_CHECK_RET(HI_MPI_VO_DisableChn(VoLayer, i),"HI_MPI_VO_DisableChn");
    }

    /* DISABLE VO LAYER */
    SAMPLE_CHECK_RET(HI_MPI_VO_DisableVideoLayer(VoLayer),"HI_MPI_VO_DisableVideoLayer");

    /* DISABLE VO DEV */
    SAMPLE_CHECK_RET(HI_MPI_VO_Disable(VoDev),"HI_MPI_VO_Disable");

    SAMPLE_VOU_SYS_Exit();

    return HI_SUCCESS;
}

HI_S32 SAMPLE_VO_MIPILCD_1920_1080(HI_VOID)
{
    HI_U32 i = 0;
    HI_S32 VoDev = 0;
    HI_S32 VoLayer = 0;
    HI_S32 VoChnNum = 1;
    VO_PUB_ATTR_S stPubAttr;
    VO_VIDEO_LAYER_ATTR_S stLayerAttr;
    VO_USER_INTFSYNC_INFO_S stUserInfo;
    HI_U32 u32Framerate;
    SIZE_S stDevSize;

    SAMPLE_VOU_ThreadCtrl_Info stThreadInfo;

    VO_CHN_ATTR_S astChnAttr[VO_MAX_CHN_NUM];

    HI_CHAR filename[256] = YUV_1920_1080;
    SIZE_S stFrameSize    = {1080, 1920};

    SAMPLE_CHECK_RET(SAMPLE_VOU_SYS_Init(),"SAMPLE_VOU_SYS_Init");

    /* USER CONFIG MIPI DEV */
    SAMPLE_CHECK_RET(SAMPLE_VO_CONFIG_MIPI(),"SAMPLE_VO_CONFIG_MIPI");

    /* SET VO PUB ATTR OF USER TYPE */
    SAMPLE_VO_GetUserPubBaseAttr(&stPubAttr);

    stPubAttr.enIntfType = VO_INTF_MIPI;

    /* USER SET VO DEV SYNC INFO */
    stPubAttr.stSyncInfo.u16Hact = 1080;
    stPubAttr.stSyncInfo.u16Hbb = 28;
    stPubAttr.stSyncInfo.u16Hfb = 130;
    stPubAttr.stSyncInfo.u16Hpw = 8;
    stPubAttr.stSyncInfo.u16Vact = 1920;
    stPubAttr.stSyncInfo.u16Vbb = 36;
    stPubAttr.stSyncInfo.u16Vfb = 16;
    stPubAttr.stSyncInfo.u16Vpw = 10;

    SAMPLE_CHECK_RET(HI_MPI_VO_SetPubAttr(VoDev, &stPubAttr), "HI_MPI_VO_SetPubAttr");

    /* USER SET VO FRAME RATE */
    u32Framerate = 60;

    SAMPLE_CHECK_RET(HI_MPI_VO_SetDevFrameRate(VoDev, u32Framerate), "HI_MPI_VO_SetDevFrameRate");

    /* USER SET VO SYNC INFO OF USER INTF */
    stUserInfo.bClkReverse = HI_TRUE;
    stUserInfo.u32DevDiv = 1;
    stUserInfo.u32PreDiv = 1;
    stUserInfo.stUserIntfSyncAttr.enClkSource = VO_CLK_SOURCE_PLL;
    stUserInfo.stUserIntfSyncAttr.stUserSyncPll.u32Fbdiv = 73;
    stUserInfo.stUserIntfSyncAttr.stUserSyncPll.u32Frac = 0x3D75E2;
    stUserInfo.stUserIntfSyncAttr.stUserSyncPll.u32Refdiv = 2;
    stUserInfo.stUserIntfSyncAttr.stUserSyncPll.u32Postdiv1 = 3;
    stUserInfo.stUserIntfSyncAttr.stUserSyncPll.u32Postdiv2 = 2;

    SAMPLE_CHECK_RET(HI_MPI_VO_SetUserIntfSyncInfo(VoDev, &stUserInfo), "HI_MPI_VO_SetUserIntfSyncInfo");

    /* ENABLE VO DEV */
    SAMPLE_CHECK_RET(HI_MPI_VO_Enable(VoDev), "HI_MPI_VO_Enable");

    /* SET VO DISPLAY BUFFER LENGTH */
    SAMPLE_CHECK_RET(HI_MPI_VO_SetDisplayBufLen(VoDev, 3), "HI_MPI_VO_SetDisplayBufLen");

    /*SET VO LAYER ATTR*/
    stDevSize.u32Width = stPubAttr.stSyncInfo.u16Hact;
    stDevSize.u32Height = stPubAttr.stSyncInfo.u16Vact;

    SAMPLE_VO_GetUserLayerAttr(&stLayerAttr, &stDevSize);
    stLayerAttr.u32DispFrmRt = 60;

    SAMPLE_CHECK_RET(HI_MPI_VO_SetVideoLayerAttr(VoLayer, &stLayerAttr), "HI_MPI_VO_SetVideoLayerAttr");

    /* ENABLE VO LAYER */
    SAMPLE_CHECK_RET(HI_MPI_VO_EnableVideoLayer(VoLayer), "HI_MPI_VO_EnableVideoLayer");

    /* SET AND ENABLE VO CHN */
    SAMPLE_VO_GetUserChnAttr(astChnAttr, &stDevSize, VoChnNum);

    for (i = 0; i < VoChnNum; i++) {
        SAMPLE_CHECK_RET(HI_MPI_VO_SetChnAttr(VoLayer, i, &astChnAttr[i]), "HI_MPI_VO_SetChnAttr");

        SAMPLE_CHECK_RET(HI_MPI_VO_EnableChn(VoLayer, i), "HI_MPI_VO_EnableChn");
    }

    /* START USER THREAD */
    SAMPLE_VO_StartUserThd(&stThreadInfo, VoLayer, VoChnNum,filename,&stFrameSize);

    PAUSE();

    /* STOP USER THREAD */
    SAMPLE_VO_StopUserThd(&stThreadInfo);

    /*DISABLE VO CHN*/
    for (i = 0; i < VoChnNum; i++) {
        SAMPLE_CHECK_RET(HI_MPI_VO_DisableChn(VoLayer, i), "HI_MPI_VO_DisableChn");
    }

    /* DISABLE VO LAYER */
    SAMPLE_CHECK_RET(HI_MPI_VO_DisableVideoLayer(VoLayer), "HI_MPI_VO_DisableVideoLayer");

    /* DISABLE VO DEV */
    SAMPLE_CHECK_RET(HI_MPI_VO_Disable(VoDev), "HI_MPI_VO_Disable");

    SAMPLE_VOU_SYS_Exit();

    return HI_SUCCESS;
}

/* need to be added */
HI_S32 SAMPLE_VO_MIPILCD_320_1280(HI_VOID)
{
    return HI_SUCCESS;
}

/* need to be added */
HI_S32 SAMPLE_VO_MIPILCD_380_1920(HI_VOID)
{
    return HI_SUCCESS;
}

/* need to be added */
HI_S32 SAMPLE_VO_MIPILCD_480_854(HI_VOID)
{
    return HI_SUCCESS;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
