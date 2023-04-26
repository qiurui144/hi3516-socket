

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
#include <sys/prctl.h>
#include <sys/ioctl.h>

#include "hi_comm_isp.h"
#include "hi_isp_debug.h"
#include "sample_comm.h"
#include "mpi_isp.h"

#define MAX(a, b) (((a) < (b)) ?  (b) : (a))
#define ABS(x)          ( (x) >= 0 ? (x) : (-(x)) )


PAYLOAD_TYPE_E g_enVencType   = PT_H265;
SAMPLE_RC_E    g_enRcMode     = SAMPLE_RC_CBR;
PIC_SIZE_E     g_enPicSize    = PIC_1080P;



pthread_t ThreadId;


SAMPLE_VO_CONFIG_S g_stVoConfig = {0};

/******************************************************************************
* function : show usage
******************************************************************************/
void SAMPLE_CalcFlicker_Usage(char *sPrgNm)
{
    printf("Usage : %s <index>\n", sPrgNm);
    printf("index:\n");

    printf("\t 0)Auto Calculate Flicker and AntiFlicker.\n");
    printf("\t 1)Only Calculate Flicker Type For 1 Times.\n");
    printf("\t 2)Only Calculate Flicker Type For 3 Times.\n");

    printf("\t calc flicker normal -mode under wall mount.\n");
    printf("\t If you have any questions, please look at readme.txt!\n");
    return;
}

/******************************************************************************
* function : to process abnormal case
******************************************************************************/
void SAMPLE_CalcFlicker_HandleSig(HI_S32 signo)
{
    signal(SIGINT, SIG_IGN);
    signal(SIGTERM, SIG_IGN);

    if (SIGINT == signo || SIGTERM == signo) {
        SAMPLE_COMM_VO_HdmiStop();
        SAMPLE_COMM_All_ISP_Stop();
        SAMPLE_COMM_SYS_Exit();
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
    }
    exit(-1);
}


HI_S32 SAMPLE_CalcFlicker_StopViVoVenc(SAMPLE_VI_CONFIG_S *pstViConfig, VI_PIPE ViPipe, VI_CHN ViExtChn, VO_CHN VoChn, VENC_CHN VencChn)
{
    HI_S32   s32Ret     = HI_SUCCESS;
    VO_LAYER VoLayer = g_stVoConfig.VoDev;

    s32Ret = HI_MPI_VI_DisableChn(ViPipe, ViExtChn);
    if (HI_SUCCESS != s32Ret) {
        SAMPLE_PRT("HI_MPI_VI_DisableChn extchn:%d failed with %#x\n", ViExtChn, s32Ret);
        return HI_FAILURE;
    }

    SAMPLE_COMM_VI_UnBind_VENC(ViPipe, ViExtChn, VencChn);
    SAMPLE_COMM_VI_UnBind_VO(ViPipe, ViExtChn, VoLayer, VoChn);

    SAMPLE_COMM_VO_StopVO(&g_stVoConfig);

    SAMPLE_COMM_VI_StopVi(pstViConfig);

    return HI_SUCCESS;
}

HI_S32 SAMPLE_CalcFlicker_StartViVo(SAMPLE_VI_CONFIG_S *pstViConfig, SAMPLE_VO_CONFIG_S *pstVoConfig)
{
    HI_S32  s32Ret;

    s32Ret = SAMPLE_COMM_VI_StartVi(pstViConfig);

    if (HI_SUCCESS != s32Ret) {
        SAMPLE_PRT("start vi failed!\n");
        return s32Ret;
    }

    s32Ret = SAMPLE_COMM_VO_StartVO(pstVoConfig);

    if (HI_SUCCESS != s32Ret) {
        SAMPLE_PRT("SAMPLE_VIO start VO failed with %#x!\n", s32Ret);
        goto EXIT;
    }

    return s32Ret;

EXIT:
    SAMPLE_COMM_VI_StopVi(pstViConfig);

    return s32Ret;
}

HI_S32 SAMPLE_CalcFlicker_StopViVo(SAMPLE_VI_CONFIG_S *pstViConfig, SAMPLE_VO_CONFIG_S *pstVoConfig)
{
    SAMPLE_COMM_VO_StopVO(pstVoConfig);

    SAMPLE_COMM_VI_StopVi(pstViConfig);

    return HI_SUCCESS;
}

HI_S32 SAMPLE_CalcFlickerType(VI_PIPE ViPipe, VPSS_GRP VpssGrp, VPSS_GRP_PIPE VpssPipe, ISP_CALCFLICKER_INPUT_S *pstInputParam, ISP_CALCFLICKER_OUTPUT_S *pstOutputParam)
{
    HI_U16 i = 0;
    HI_U16 u16FrameNum = 3;


    HI_U16 u16TimeOut = 50;
    HI_S32 s32Times = 10;

    HI_S32 s32Ret = HI_SUCCESS;

    HI_U32 u32TimeRefLast = 0;

    VIDEO_FRAME_INFO_S stFrame[3];


    /* get frame buffer*/
    for (i = 0; i < u16FrameNum; i++) {
        if (0 == i) {
            while ((HI_MPI_VPSS_GetGrpFrame(VpssGrp, VpssPipe, &(stFrame[i])) != HI_SUCCESS)) {
                s32Times--;

                if (0 >= s32Times) {
                    //ISP_ERR_TRACE(HI_DBG_ERR, "get frame error for 10 times,now exit !!!\n");
                    ISP_ERR_TRACE("get frame error for 10 times,now exit !!!\n");
                    return HI_FAILURE;
                }
                sleep(2);
            }

            if (VIDEO_FORMAT_LINEAR != stFrame[i].stVFrame.enVideoFormat) {
                //ISP_ERR_TRACE(HI_DBG_ERR, "only support linear frame dump!\n");
                ISP_ERR_TRACE("only support linear frame dump!!!\n");
                HI_MPI_VPSS_ReleaseGrpFrame(VpssGrp, VpssPipe, &(stFrame[i]));
                return HI_FAILURE;
            }
        }

        if (0 != i) {
            while (u16TimeOut--) {
                usleep(5 * 1000);

                while ((HI_MPI_VPSS_GetGrpFrame(VpssGrp, VpssPipe, &(stFrame[i])) != HI_SUCCESS)) {
                    printf("VPSS Get Frame Fail Times: %d\n", s32Times);
                    s32Times--;

                    if (0 >= s32Times) {
                        printf("get frame error for 10 times,now exit !!!\n");
                        return HI_FAILURE;
                    }
                    sleep(2);
                }

                if (u32TimeRefLast == stFrame[i].stVFrame.u32TimeRef) {
                    s32Ret = HI_MPI_VPSS_ReleaseGrpFrame(VpssGrp, VpssPipe, &(stFrame[i]));
                } else {
                    break;
                }
            }
        }

        u32TimeRefLast = stFrame[i].stVFrame.u32TimeRef;
    }


    s32Ret = HI_MPI_ISP_CalcFlickerType(ViPipe, pstInputParam, pstOutputParam, stFrame, u16FrameNum);

    for (i = 0; i < u16FrameNum; i++) {
        s32Ret = HI_MPI_VPSS_ReleaseGrpFrame(VpssGrp, VpssPipe, &(stFrame[i]));
    }

    for (i = 0; i < u16FrameNum; i++) {
        s32Ret = HI_MPI_VPSS_ReleaseGrpFrame(VpssGrp, VpssPipe, &(stFrame[i]));
    }

    return s32Ret;
}


HI_S32 SAMPLE_AutoCalcFlickerAndAntiFlicker(void)
{
    //HI_U8 u8FlickerType;
    HI_U8 u8DetectTimeOut = 50;

    HI_U8 u8DetResult50Hz = 0;
    HI_U8 u8DetResult60Hz = 0;
    HI_U8 u8DetResultHaveFlicker = 0;
    HI_U8 u8DetResultNoFlicker = 0;

    HI_U8 u8Test50Hz = HI_FALSE;
    HI_U8 u8Test60Hz = HI_FALSE;

    VPSS_GRP VpssGrp = 0;
    VPSS_GRP_PIPE VpssPipe = 0;

    VI_PIPE ViPipe = 0;
    HI_S32 s32Ret;

    ISP_EXPOSURE_ATTR_S stExposureAttr;
    ISP_EXPOSURE_ATTR_S stExposureAttrTmp;
    ISP_EXP_INFO_S stIspExpInfo;
    ISP_PUB_ATTR_S stPubAttr;
    ISP_EXP_INFO_S pstExpInfo;

    s32Ret = HI_MPI_ISP_GetExposureAttr(ViPipe, &stExposureAttr);
    s32Ret = HI_MPI_ISP_GetExposureAttr(ViPipe, &stExposureAttrTmp);

    ISP_CALCFLICKER_INPUT_S stInputParam;
    ISP_CALCFLICKER_OUTPUT_S stOutputParam;



    HI_MPI_ISP_QueryExposureInfo(ViPipe, &pstExpInfo);

    stInputParam.u32LinesPerSecond = pstExpInfo.u32LinesPer500ms * 2;


    while (u8DetectTimeOut--) {

        s32Ret = SAMPLE_CalcFlickerType(ViPipe, VpssGrp, VpssPipe, &stInputParam, &stOutputParam);

        s32Ret = HI_MPI_ISP_QueryExposureInfo(ViPipe, &stIspExpInfo);
        s32Ret = HI_MPI_ISP_GetPubAttr(ViPipe, &stPubAttr);

        if (FLICKER_TYPE_50HZ == stOutputParam.enFlickerType) {
            printf("##### FLICKER_TYPE_50HZ %d #####\n", u8DetectTimeOut);

            u8DetResultNoFlicker = 0;
            u8DetResult60Hz = 0;
            u8DetResultHaveFlicker = 0;

            u8DetResult50Hz++;

            if (u8DetResult50Hz >= 3) {
                if ((stIspExpInfo.u32ExpTime < 5500) && (stIspExpInfo.u32ISO < 220)) {
                    stPubAttr.f32FrameRate = 25.0;
                    s32Ret = HI_MPI_ISP_SetPubAttr(ViPipe, &stPubAttr);
                    printf("----- Set Pub Frame Rate to 25 -----\n");
                } else {
                    stExposureAttr.stAuto.stAntiflicker.bEnable = HI_TRUE;
                    stExposureAttr.stAuto.stAntiflicker.u8Frequency = 50;

                    s32Ret = HI_MPI_ISP_SetExposureAttr(ViPipe, &stExposureAttr);

                }
                usleep(1000 * 500);

                //break;
                if (u8DetResult50Hz > 5) {
                    s32Ret = HI_MPI_ISP_SetExposureAttr(ViPipe, &stExposureAttrTmp);
                    break;
                }

            }
        } else if (FLICKER_TYPE_60HZ == stOutputParam.enFlickerType) {
            printf("##### FLICKER_TYPE_60HZ %d #####\n", u8DetectTimeOut);

            u8DetResultNoFlicker = 0;
            u8DetResult50Hz = 0;
            u8DetResultHaveFlicker = 0;

            u8DetResult60Hz++;

            if (u8DetResult60Hz >= 3) {
                if ((stIspExpInfo.u32ExpTime < 4400) && (stIspExpInfo.u32ISO < 220)) {
                    stPubAttr.f32FrameRate = 30.0;
                    s32Ret = HI_MPI_ISP_SetPubAttr(ViPipe, &stPubAttr);
                    printf("----- Set Pub Frame Rate to 30 -----\n");
                } else {
                    stExposureAttr.stAuto.stAntiflicker.bEnable = HI_TRUE;
                    stExposureAttr.stAuto.stAntiflicker.u8Frequency = 60;

                    s32Ret = HI_MPI_ISP_SetExposureAttr(ViPipe, &stExposureAttr);
                }
                usleep(1000 * 500);
                //break;
                if (u8DetResult60Hz > 5) {
                    s32Ret = HI_MPI_ISP_SetExposureAttr(ViPipe, &stExposureAttrTmp);
                    break;
                }
            }
        } else if (FLICKER_TYPE_UNKNOW == stOutputParam.enFlickerType) {
            printf("##### FLICKER_TYPE_UNKNOW %d #####\n", u8DetectTimeOut);

            //u8DetResult50Hz = 0;
            //u8DetResult60Hz = 0;
            u8DetResultNoFlicker = 0;

            u8DetResultHaveFlicker++;

            if (u8DetResultHaveFlicker >= 4) {
                if (HI_FALSE == u8Test50Hz) {
                    printf("---- Try to Use 50Hz ----\n");
                    stExposureAttr.stAuto.stAntiflicker.bEnable = HI_TRUE;
                    stExposureAttr.stAuto.stAntiflicker.u8Frequency = 50;
                    s32Ret = HI_MPI_ISP_SetExposureAttr(ViPipe, &stExposureAttr);

                    u8Test50Hz = HI_TRUE;
                    u8DetResultHaveFlicker = 0;

                    usleep(1000 * 500);
                } else if (HI_FALSE == u8Test60Hz) {
                    printf("---- Try to Use 60Hz ----\n");
                    stExposureAttr.stAuto.stAntiflicker.bEnable = HI_TRUE;
                    stExposureAttr.stAuto.stAntiflicker.u8Frequency = 60;
                    s32Ret = HI_MPI_ISP_SetExposureAttr(ViPipe, &stExposureAttr);

                    u8Test60Hz = HI_TRUE;
                    u8DetResultHaveFlicker = 0;

                    usleep(1000 * 500);
                } else {
                    s32Ret = HI_MPI_ISP_SetExposureAttr(ViPipe, &stExposureAttrTmp);

                    printf("Have Try Use 50Hz & 60Hz\n");
                    break;
                }

                if (u8DetResultHaveFlicker > 6) {
                    s32Ret = HI_MPI_ISP_SetExposureAttr(ViPipe, &stExposureAttrTmp);
                    break;
                }

            }
        } else if (FLICKER_TYPE_NONE == stOutputParam.enFlickerType) {
            printf("##### FLICKER_TYPE_NONE %d #####\n", u8DetectTimeOut);

            u8DetResult50Hz = 0;
            u8DetResult60Hz = 0;
            u8DetResultHaveFlicker = 0;

            u8DetResultNoFlicker++;
            if (u8DetResultNoFlicker >= 3) {
                printf("No Flicker Detect more than 5 time\n");
                break;
            }
        }
    }

    return s32Ret;
}


HI_S32 SAMPLE_CalcFlicker_Normal(HI_VOID)
{
    HI_S32 s32Ret;

    s32Ret = SAMPLE_AutoCalcFlickerAndAntiFlicker();

    return s32Ret;
}

HI_S32 SAMPLE_CalcFlicker_3Times(HI_VOID)
{
    HI_S32 s32Ret;

    VPSS_GRP VpssGrp = 0;
    VPSS_GRP_PIPE VpssPipe = 0;
    VI_PIPE ViPipe = 0;

    ISP_EXP_INFO_S pstExpInfo;
    HI_U16 i = 0;
    ISP_CALCFLICKER_INPUT_S stInputParam;
    ISP_CALCFLICKER_OUTPUT_S stOutputParam;

    HI_MPI_ISP_QueryExposureInfo(ViPipe, &pstExpInfo);

    stInputParam.u32LinesPerSecond = pstExpInfo.u32LinesPer500ms * 2;


    for (i = 0; i < 3; i++) {
        s32Ret = SAMPLE_CalcFlickerType(ViPipe, VpssGrp, VpssPipe, &stInputParam, &stOutputParam);

        if (FLICKER_TYPE_50HZ == stOutputParam.enFlickerType) {
            printf("Time: %d, ##### FLICKER_TYPE_50HZ #####\n", i + 1);
        } else if (FLICKER_TYPE_60HZ == stOutputParam.enFlickerType) {
            printf("Time: %d, ##### FLICKER_TYPE_60HZ #####\n", i + 1);
        } else if (FLICKER_TYPE_NONE == stOutputParam.enFlickerType) {
            printf("Time: %d, ##### FLICKER_TYPE_NONE #####\n", i + 1);
        } else if (FLICKER_TYPE_UNKNOW == stOutputParam.enFlickerType) {
            printf("Time: %d, ##### FLICKER_TYPE_UNKNOW #####\n", i + 1);
        }
    }

    return s32Ret;
}

HI_S32 SAMPLE_CalcFlicker_1Times(HI_VOID)
{
    HI_S32 s32Ret;

    VPSS_GRP VpssGrp = 0;
    VPSS_GRP_PIPE VpssPipe = 0;
    VI_PIPE ViPipe = 0;

    ISP_EXP_INFO_S pstExpInfo;
    ISP_CALCFLICKER_INPUT_S stInputParam;
    ISP_CALCFLICKER_OUTPUT_S stOutputParam;


    HI_MPI_ISP_QueryExposureInfo(ViPipe, &pstExpInfo);

    stInputParam.u32LinesPerSecond = pstExpInfo.u32LinesPer500ms * 2;

    s32Ret = SAMPLE_CalcFlickerType(ViPipe, VpssGrp, VpssPipe, &stInputParam, &stOutputParam);

    if (FLICKER_TYPE_50HZ == stOutputParam.enFlickerType) {
        printf("\n##### FLICKER_TYPE_50HZ #####\n");
    } else if (FLICKER_TYPE_60HZ == stOutputParam.enFlickerType) {
        printf("\n##### FLICKER_TYPE_60HZ #####\n");
    } else if (FLICKER_TYPE_NONE == stOutputParam.enFlickerType) {
        printf("\n##### FLICKER_TYPE_NONE #####\n");
    } else if (FLICKER_TYPE_UNKNOW == stOutputParam.enFlickerType) {
        printf("\n##### FLICKER_TYPE_UNKNOW #####\n");
    }

    return s32Ret;
}

HI_S32 SAMPLE_CalcFlicker_Test(HI_VOID)
{
    return HI_SUCCESS;
}


/******************************************************************************
* function : vi/vpss: offline/online fisheye mode VI-VO. Embeded isp, phychn channel preview.
******************************************************************************/


/******************************************************************************
* function    : main()
* Description : video fisheye preview sample
******************************************************************************/
#ifdef __HuaweiLite__
int app_main(int argc, char *argv[])
#else
int main(int argc, char *argv[])
#endif
{
    HI_S32             s32Ret        = HI_FAILURE;

    if (argc < 2 || argc > 2) {
        SAMPLE_CalcFlicker_Usage(argv[0]);
        return HI_FAILURE;
    }

    if (!strncmp(argv[1], "-h", 2)) {
        SAMPLE_CalcFlicker_Usage(argv[0]);
        return HI_SUCCESS;
    }

#ifndef __HuaweiLite__
    signal(SIGINT, SAMPLE_CalcFlicker_HandleSig);
    signal(SIGTERM, SAMPLE_CalcFlicker_HandleSig);
#endif

    g_enVencType = PT_H265;
    g_stVoConfig.enVoIntfType = VO_INTF_HDMI;

    SAMPLE_COMM_VO_GetDefConfig(&g_stVoConfig);

    switch (*argv[1]) {
            /* VI/VPSS - VO. Embeded isp, phychn channel preview. */
        case '0':
            s32Ret = SAMPLE_CalcFlicker_Normal();
            break;

        case '1':
            s32Ret = SAMPLE_CalcFlicker_1Times();
            break;

        case '2':
            s32Ret = SAMPLE_CalcFlicker_3Times();
            break;

        default:
            SAMPLE_PRT("the index is invaild!\n");
            SAMPLE_CalcFlicker_Usage(argv[0]);
            return HI_FAILURE;
    }

    printf("Finish\n");

    return s32Ret;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

