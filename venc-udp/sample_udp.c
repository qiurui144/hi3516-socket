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
#include <sys/ioctl.h>

#include "sample_comm.h"

/******************************************************************************
* funciton : save stream by YJSNPI (114
******************************************************************************/
HI_S32 SAMPLE_COMM_YJSNPIVENC_SaveStreamToUdp(int sock, struct sockaddr* addr, VENC_STREAM_S* pstStream)
{
    HI_S32 i;

    for (i = 0; i < pstStream->u32PackCount; i++) {
		if (sendto(sock, pstStream->pstPack[i].pu8Addr + pstStream->pstPack[i].u32Offset,
					pstStream->pstPack[i].u32Len - pstStream->pstPack[i].u32Offset, 
					0, addr, sizeof(struct sockaddr_in)) == -1) {
			printf("ERROR: Could not send UDP data!");
			return HI_FAILURE;
		}
    }

    return HI_SUCCESS;
}

// note: bitrate is set here
HI_S32 SAMPLE_COMM_YJSNPIVENC_Creat(VENC_CHN VencChn, HI_S32 chn, YJSNPI_VENC_CONFIG_S *pconf, HI_BOOL bRcnRefShareBuf)
{
    HI_S32 s32Ret;
    SIZE_S stPicSize;
    VENC_CHN_ATTR_S        	stVencChnAttr;
    SAMPLE_VI_CONFIG_S     	stViConfig;
    HI_U32                 	u32FrameRate;
    HI_U32                 	u32StatTime;
    HI_U32                 	u32Gop = 30;
	VENC_GOP_ATTR_S 		stGopAttr;
	
    s32Ret = SAMPLE_COMM_SYS_GetPicSize(pconf->res[chn], &stPicSize);
    if (HI_SUCCESS != s32Ret) {
        SAMPLE_PRT("Get picture size failed!\n");
        return HI_FAILURE;
    }

    SAMPLE_COMM_VI_GetSensorInfo(&stViConfig);
    if(SAMPLE_SNS_TYPE_BUTT == stViConfig.astViInfo[0].stSnsInfo.enSnsType) {
        SAMPLE_PRT("Not set SENSOR%d_TYPE !\n",0);
        return HI_FALSE;
    }
    s32Ret = SAMPLE_COMM_VI_GetFrameRateBySensor(stViConfig.astViInfo[0].stSnsInfo.enSnsType, &u32FrameRate);
    if (HI_SUCCESS != s32Ret) {
        SAMPLE_PRT("SAMPLE_COMM_VI_GetFrameRateBySensor failed!\n");
        return s32Ret;
    }

    /******************************************
     step 1:  Create Venc Channel
    ******************************************/
    stVencChnAttr.stVencAttr.enType          = pconf->enc[chn];
    stVencChnAttr.stVencAttr.u32MaxPicWidth  = stPicSize.u32Width;
    stVencChnAttr.stVencAttr.u32MaxPicHeight = stPicSize.u32Height;
    stVencChnAttr.stVencAttr.u32PicWidth     = stPicSize.u32Width;/*the picture width*/
    stVencChnAttr.stVencAttr.u32PicHeight    = stPicSize.u32Height;/*the picture height*/
    stVencChnAttr.stVencAttr.u32BufSize      = ALIGN_UP(stPicSize.u32Width * stPicSize.u32Height * 3/4, 64);/*stream buffer size*/
    stVencChnAttr.stVencAttr.u32Profile      = pconf->profile[chn];
    stVencChnAttr.stVencAttr.bByFrame        = HI_TRUE;/*get stream mode is slice mode or frame mode?*/
	
	s32Ret = SAMPLE_COMM_VENC_GetGopAttr(pconf->gop[chn], &stGopAttr);
    if (HI_SUCCESS != s32Ret) {
        SAMPLE_PRT("SAMPLE_COMM_VENC_GetGopAttr Get GopAttr ch%d for %#x!\n", chn, s32Ret);
		return HI_FAILURE;
    }
	
	u32StatTime = (VENC_GOPMODE_SMARTP == stGopAttr.enGopMode)? stGopAttr.stSmartP.u32BgInterval/u32Gop: 1;
	printf("set bitrate: %d\n", pconf->kbps[chn]);
	// Set framerate, encode mode, bitrate .. etc.
	if (pconf->enc[chn] == PT_H265) {
		switch (pconf->rc[chn]) {
		case SAMPLE_RC_CBR: {
			VENC_H265_CBR_S stH265Cbr;
			stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
			stH265Cbr.u32Gop            = u32Gop;
			stH265Cbr.u32StatTime       = u32StatTime; /* stream rate statics time(s) */
			stH265Cbr.u32SrcFrameRate   = u32FrameRate; /* input (vi) frame rate */
			stH265Cbr.fr32DstFrameRate  = u32FrameRate; /* target frame rate */
			stH265Cbr.u32BitRate = pconf->kbps[chn];
			memcpy(&stVencChnAttr.stRcAttr.stH265Cbr, &stH265Cbr, sizeof(VENC_H265_CBR_S));
		}
		break;
		case SAMPLE_RC_VBR: {
			VENC_H265_VBR_S    stH265Vbr;
			stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
			stH265Vbr.u32Gop           = u32Gop;
			stH265Vbr.u32StatTime      = u32StatTime;
			stH265Vbr.u32SrcFrameRate  = u32FrameRate;
			stH265Vbr.fr32DstFrameRate = u32FrameRate;
			stH265Vbr.u32MaxBitRate = pconf->kbps[chn];
			memcpy(&stVencChnAttr.stRcAttr.stH265Vbr, &stH265Vbr, sizeof(VENC_H265_VBR_S));
		}
		break;
		case SAMPLE_RC_AVBR: {
			VENC_H265_AVBR_S  stH265AVbr;
			stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265AVBR;
			stH265AVbr.u32Gop         = u32Gop;
			stH265AVbr.u32StatTime    = u32StatTime;
			stH265AVbr.u32SrcFrameRate  = u32FrameRate;
			stH265AVbr.fr32DstFrameRate = u32FrameRate;
			stH265AVbr.u32MaxBitRate = pconf->kbps[chn];
			memcpy(&stVencChnAttr.stRcAttr.stH265AVbr, &stH265AVbr, sizeof(VENC_H265_AVBR_S));
		}
		break;
		case SAMPLE_RC_QVBR: {
			VENC_H265_QVBR_S  stH265QVbr;
			stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265QVBR;
			stH265QVbr.u32Gop         = u32Gop;
			stH265QVbr.u32StatTime    = u32StatTime;
			stH265QVbr.u32SrcFrameRate  = u32FrameRate;
			stH265QVbr.fr32DstFrameRate = u32FrameRate;
			stH265QVbr.u32TargetBitRate = pconf->kbps[chn];
			memcpy(&stVencChnAttr.stRcAttr.stH265QVbr, &stH265QVbr, sizeof(VENC_H265_QVBR_S));
		}
		break;
		case SAMPLE_RC_CVBR: {
			VENC_H265_CVBR_S    stH265CVbr;
			stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CVBR;
			stH265CVbr.u32Gop         = u32Gop;
			stH265CVbr.u32StatTime    = u32StatTime;
			stH265CVbr.u32SrcFrameRate  = u32FrameRate;
			stH265CVbr.fr32DstFrameRate = u32FrameRate;
			stH265CVbr.u32LongTermStatTime  = 1;
			stH265CVbr.u32ShortTermStatTime = u32StatTime;
			stH265CVbr.u32MaxBitRate         = pconf->kbps[chn];
			stH265CVbr.u32LongTermMaxBitrate = pconf->kbps[chn] * 0.8;	// enough?
			stH265CVbr.u32LongTermMinBitrate = 128;						// should it be fixed?
			memcpy(&stVencChnAttr.stRcAttr.stH265CVbr, &stH265CVbr, sizeof(VENC_H265_CVBR_S));
		}
		break;
		case SAMPLE_RC_QPMAP: {
			VENC_H265_QPMAP_S    stH265QpMap;
			stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265QPMAP;
			stH265QpMap.u32Gop           = u32Gop;
			stH265QpMap.u32StatTime      = u32StatTime;
			stH265QpMap.u32SrcFrameRate  = u32FrameRate;
			stH265QpMap.fr32DstFrameRate = u32FrameRate;
			stH265QpMap.enQpMapMode      = VENC_RC_QPMAP_MODE_MEANQP;
			memcpy(&stVencChnAttr.stRcAttr.stH265QpMap, &stH265QpMap, sizeof(VENC_H265_QPMAP_S));
		}
		break;
		default:
			SAMPLE_PRT("%s, %d, enRcMode(%d) not support\n", __FUNCTION__, __LINE__, pconf->kbps[chn]);
			return HI_FAILURE;
		}	// switch (pconf->rc[chn])
		stVencChnAttr.stVencAttr.stAttrH265e.bRcnRefShareBuf = bRcnRefShareBuf;
	} 
	else if (pconf->enc[chn] == PT_H264) {
		switch (pconf->rc[chn]) {
		case SAMPLE_RC_CBR: {
			VENC_H264_CBR_S    stH264Cbr;
			stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
			stH264Cbr.u32Gop                = u32Gop; /*the interval of IFrame*/
			stH264Cbr.u32StatTime           = u32StatTime; /* stream rate statics time(s) */
			stH264Cbr.u32SrcFrameRate       = u32FrameRate; /* input (vi) frame rate */
			stH264Cbr.fr32DstFrameRate      = u32FrameRate; /* target frame rate */
			stH264Cbr.u32BitRate = pconf->kbps[chn];
			memcpy(&stVencChnAttr.stRcAttr.stH264Cbr, &stH264Cbr, sizeof(VENC_H264_CBR_S));
		}
		break;
		case SAMPLE_RC_VBR: {
			VENC_H264_VBR_S    stH264Vbr;
			stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
			stH264Vbr.u32Gop           = u32Gop;
			stH264Vbr.u32StatTime      = u32StatTime;
			stH264Vbr.u32SrcFrameRate  = u32FrameRate;
			stH264Vbr.fr32DstFrameRate = u32FrameRate;
			stH264Vbr.u32MaxBitRate    = pconf->kbps[chn];
			memcpy(&stVencChnAttr.stRcAttr.stH264Vbr, &stH264Vbr, sizeof(VENC_H264_VBR_S));
		}
		break;
		case SAMPLE_RC_AVBR: {
			VENC_H264_VBR_S    stH264AVbr;
			stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264AVBR;
			stH264AVbr.u32Gop           = u32Gop;
			stH264AVbr.u32StatTime      = u32StatTime;
			stH264AVbr.u32SrcFrameRate  = u32FrameRate;
			stH264AVbr.fr32DstFrameRate = u32FrameRate;
			stH264AVbr.u32MaxBitRate = pconf->kbps[chn];
			memcpy(&stVencChnAttr.stRcAttr.stH264AVbr, &stH264AVbr, sizeof(VENC_H264_AVBR_S));
		}
		break;
		case SAMPLE_RC_QVBR: {
			VENC_H264_QVBR_S    stH264QVbr;
			stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264QVBR;
			stH264QVbr.u32Gop           = u32Gop;
			stH264QVbr.u32StatTime      = u32StatTime;
			stH264QVbr.u32SrcFrameRate  = u32FrameRate;
			stH264QVbr.fr32DstFrameRate = u32FrameRate;
			stH264QVbr.u32TargetBitRate = pconf->kbps[chn];
			memcpy(&stVencChnAttr.stRcAttr.stH264QVbr, &stH264QVbr, sizeof(VENC_H264_QVBR_S));
		}
		break;
		case SAMPLE_RC_CVBR: {
			VENC_H264_CVBR_S    stH264CVbr;
			stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CVBR;
			stH264CVbr.u32Gop         = u32Gop;
			stH264CVbr.u32StatTime    = u32StatTime;
			stH264CVbr.u32SrcFrameRate  = u32FrameRate;
			stH264CVbr.fr32DstFrameRate = u32FrameRate;
			stH264CVbr.u32LongTermStatTime  = 1;
			stH264CVbr.u32ShortTermStatTime = u32StatTime;
			stH264CVbr.u32MaxBitRate         = pconf->kbps[chn];
			stH264CVbr.u32LongTermMaxBitrate = pconf->kbps[chn] *0.8;
			stH264CVbr.u32LongTermMinBitrate = 128;
			memcpy(&stVencChnAttr.stRcAttr.stH264CVbr, &stH264CVbr, sizeof(VENC_H264_CVBR_S));
		}
		break;
		case SAMPLE_RC_QPMAP: {
			VENC_H264_QPMAP_S    stH264QpMap;
			stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264QPMAP;
			stH264QpMap.u32Gop           = u32Gop;
			stH264QpMap.u32StatTime      = u32StatTime;
			stH264QpMap.u32SrcFrameRate  = u32FrameRate;
			stH264QpMap.fr32DstFrameRate = u32FrameRate;
			memcpy(&stVencChnAttr.stRcAttr.stH264QpMap, &stH264QpMap, sizeof(VENC_H264_QPMAP_S));
		}
		break;
		default:
			SAMPLE_PRT("%s,%d,enRcMode(%d) not support\n", __FUNCTION__, __LINE__, pconf->rc[chn]);
            return HI_FAILURE;
		break;
		}
		stVencChnAttr.stVencAttr.stAttrH264e.bRcnRefShareBuf = bRcnRefShareBuf;
	} 
	else {
		SAMPLE_PRT("cann't support this enType (%d) in this version!\n", pconf->enc[chn]);
		return HI_ERR_VENC_NOT_SUPPORT;
	}
	
	memcpy(&stVencChnAttr.stGopAttr, &stGopAttr, sizeof(VENC_GOP_ATTR_S));
	
    s32Ret = HI_MPI_VENC_CreateChn(VencChn, &stVencChnAttr);
    if (HI_SUCCESS != s32Ret) {
        SAMPLE_PRT("HI_MPI_VENC_CreateChn [%d] faild with %#x! ===\n", VencChn, s32Ret);
        return s32Ret;
    }
    s32Ret = SAMPLE_COMM_VENC_CloseReEncode(VencChn);
    if (HI_SUCCESS != s32Ret) {
        HI_MPI_VENC_DestroyChn(VencChn);
        return s32Ret;
    }
    return HI_SUCCESS;
}

/******************************************************************************
* funciton : Start venc stream mode (YJSNPI)
* note      : rate control parameter need adjust, according your case.
******************************************************************************/
HI_S32 SAMPLE_COMM_YJSNPIVENC_Start(VENC_CHN VencChn, HI_S32 chn, YJSNPI_VENC_CONFIG_S *pconf, HI_BOOL bRcnRefShareBuf)
{
    HI_S32 s32Ret;
    VENC_RECV_PIC_PARAM_S  stRecvParam;

    /******************************************
     step 1:  Creat Encode Chnl
    ******************************************/
    s32Ret = SAMPLE_COMM_YJSNPIVENC_Creat(VencChn, chn, pconf, bRcnRefShareBuf);
    if (HI_SUCCESS != s32Ret) {
        SAMPLE_PRT("SAMPLE_COMM_YJSNPIVENC_Creat faild with%#x! \n", s32Ret);
        return HI_FAILURE;
    }
    /******************************************
     step 2:  Start Recv Venc Pictures
    ******************************************/
    stRecvParam.s32RecvPicNum = -1;
    s32Ret = HI_MPI_VENC_StartRecvFrame(VencChn, &stRecvParam);
    if (HI_SUCCESS != s32Ret) {
        SAMPLE_PRT("HI_MPI_VENC_StartRecvPic faild with%#x! \n", s32Ret);
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

/******************************************************************************
* funciton : get stream from each channels and save them
******************************************************************************/
HI_VOID* SAMPLE_COMM_YJSNPIVENC_GetVencStreamProc(HI_VOID* p)
{
    HI_S32 i;
    HI_S32 s32ChnTotal;
    VENC_CHN_ATTR_S stVencChnAttr;
    SAMPLE_VENC_GETSTREAM_PARA_S* pstPara;
    HI_S32 maxfd = 0;
    struct timeval TimeoutVal;
    fd_set read_fds;
    HI_U32 u32PictureCnt[VENC_MAX_CHN_NUM]={0};
    HI_S32 VencFd[VENC_MAX_CHN_NUM];
    HI_CHAR aszFileName[VENC_MAX_CHN_NUM][64];
    FILE* pFile[VENC_MAX_CHN_NUM];
    char szFilePostfix[10];
    VENC_CHN_STATUS_S stStat;
    VENC_STREAM_S stStream;
    HI_S32 s32Ret;
    VENC_CHN VencChn;
    PAYLOAD_TYPE_E enPayLoadType[VENC_MAX_CHN_NUM];
    VENC_STREAM_BUF_INFO_S stStreamBufInfo[VENC_MAX_CHN_NUM];

    prctl(PR_SET_NAME, "GetVencStream", 0,0,0);

    pstPara = (SAMPLE_VENC_GETSTREAM_PARA_S*)p;
    s32ChnTotal = pstPara->s32Cnt;
    /******************************************
     step 1:  check & prepare save-file & venc-fd
    ******************************************/
    if (s32ChnTotal >= VENC_MAX_CHN_NUM)
    {
        SAMPLE_PRT("input count invaild\n");
        return NULL;
    }
    for (i = 0; i < s32ChnTotal; i++)
    {
        /* decide the stream file name, and open file to save stream */
        VencChn = pstPara->VeChn[i];
        s32Ret = HI_MPI_VENC_GetChnAttr(VencChn, &stVencChnAttr);
        if (s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("HI_MPI_VENC_GetChnAttr chn[%d] failed with %#x!\n", \
                       VencChn, s32Ret);
            return NULL;
        }
        enPayLoadType[i] = stVencChnAttr.stVencAttr.enType;

        s32Ret = SAMPLE_COMM_VENC_GetFilePostfix(enPayLoadType[i], szFilePostfix);
        if (s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("SAMPLE_COMM_VENC_GetFilePostfix [%d] failed with %#x!\n", \
                       stVencChnAttr.stVencAttr.enType, s32Ret);
            return NULL;
        }
        if(PT_JPEG != enPayLoadType[i])
        {
            snprintf(aszFileName[i],32, "stream_chn%d%s", i, szFilePostfix);

            pFile[i] = fopen(aszFileName[i], "wb");
            if (!pFile[i])
            {
                SAMPLE_PRT("open file[%s] failed!\n",
                           aszFileName[i]);
                return NULL;
            }
        }
        /* Set Venc Fd. */
        VencFd[i] = HI_MPI_VENC_GetFd(VencChn);
        if (VencFd[i] < 0)
        {
            SAMPLE_PRT("HI_MPI_VENC_GetFd failed with %#x!\n",
                       VencFd[i]);
            return NULL;
        }
        if (maxfd <= VencFd[i])
        {
            maxfd = VencFd[i];
        }

        s32Ret = HI_MPI_VENC_GetStreamBufInfo (VencChn, &stStreamBufInfo[i]);
        if (HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("HI_MPI_VENC_GetStreamBufInfo failed with %#x!\n", s32Ret);
            return (void *)HI_FAILURE;
        }
    }

    /******************************************
     step 2:  Start to get streams of each channel.
    ******************************************/
    while (HI_TRUE == pstPara->bThreadStart)
    {
        FD_ZERO(&read_fds);
        for (i = 0; i < s32ChnTotal; i++)
        {
            FD_SET(VencFd[i], &read_fds);
        }

        TimeoutVal.tv_sec  = 2;
        TimeoutVal.tv_usec = 0;
        s32Ret = select(maxfd + 1, &read_fds, NULL, NULL, &TimeoutVal);
        if (s32Ret < 0)
        {
            SAMPLE_PRT("select failed!\n");
            break;
        }
        else if (s32Ret == 0)
        {
            SAMPLE_PRT("get venc stream time out, exit thread\n");
            continue;
        }
        else
        {
            for (i = 0; i < s32ChnTotal; i++)
            {
                if (FD_ISSET(VencFd[i], &read_fds))
                {
                    /*******************************************************
                     step 2.1 : query how many packs in one-frame stream.
                    *******************************************************/
                    memset(&stStream, 0, sizeof(stStream));

                    VencChn = pstPara->VeChn[i];

                    s32Ret = HI_MPI_VENC_QueryStatus(VencChn, &stStat);
                    if (HI_SUCCESS != s32Ret)
                    {
                        SAMPLE_PRT("HI_MPI_VENC_QueryStatus chn[%d] failed with %#x!\n", VencChn, s32Ret);
                        break;
                    }

                    /*******************************************************
                    step 2.2 :suggest to check both u32CurPacks and u32LeftStreamFrames at the same time,for example:
                     if(0 == stStat.u32CurPacks || 0 == stStat.u32LeftStreamFrames)
                     {
                        SAMPLE_PRT("NOTE: Current  frame is NULL!\n");
                        continue;
                     }
                    *******************************************************/
                    if(0 == stStat.u32CurPacks)
                    {
                          SAMPLE_PRT("NOTE: Current  frame is NULL!\n");
                          continue;
                    }
                    /*******************************************************
                     step 2.3 : malloc corresponding number of pack nodes.
                    *******************************************************/
                    stStream.pstPack = (VENC_PACK_S*)malloc(sizeof(VENC_PACK_S) * stStat.u32CurPacks);
                    if (NULL == stStream.pstPack)
                    {
                        SAMPLE_PRT("malloc stream pack failed!\n");
                        break;
                    }

                    /*******************************************************
                     step 2.4 : call mpi to get one-frame stream
                    *******************************************************/
                    stStream.u32PackCount = stStat.u32CurPacks;
                    s32Ret = HI_MPI_VENC_GetStream(VencChn, &stStream, HI_TRUE);
                    if (HI_SUCCESS != s32Ret)
                    {
                        free(stStream.pstPack);
                        stStream.pstPack = NULL;
                        SAMPLE_PRT("HI_MPI_VENC_GetStream failed with %#x!\n", \
                                   s32Ret);
                        break;
                    }

                    /*******************************************************
                     step 2.5 : save frame to file
                    *******************************************************/
                    if(PT_JPEG == enPayLoadType[i])
                    {
                        strcpy(szFilePostfix, ".jpg");
                        snprintf(aszFileName[i],32, "stream_chn%d_%d%s", VencChn, u32PictureCnt[i],szFilePostfix);
                        pFile[i] = fopen(aszFileName[i], "wb");
                        if (!pFile[i])
                        {
                            SAMPLE_PRT("open file err!\n");
                            return NULL;
                        }
                    }

#ifndef __HuaweiLite__
                    s32Ret = SAMPLE_COMM_VENC_SaveStream(pFile[i], &stStream);
#else
                    s32Ret = SAMPLE_COMM_VENC_SaveStream_PhyAddr(pFile[i], &stStreamBufInfo[i], &stStream);
#endif
                    if (HI_SUCCESS != s32Ret)
                    {
                        free(stStream.pstPack);
                        stStream.pstPack = NULL;
                        SAMPLE_PRT("save stream failed!\n");
                        break;
                    }
                    /*******************************************************
                     step 2.6 : release stream
                     *******************************************************/
                    s32Ret = HI_MPI_VENC_ReleaseStream(VencChn, &stStream);
                    if (HI_SUCCESS != s32Ret)
                    {
                        SAMPLE_PRT("HI_MPI_VENC_ReleaseStream failed!\n");
                        free(stStream.pstPack);
                        stStream.pstPack = NULL;
                        break;
                    }

                    /*******************************************************
                     step 2.7 : free pack nodes
                    *******************************************************/
                    free(stStream.pstPack);
                    stStream.pstPack = NULL;
                    u32PictureCnt[i]++;
                    if(PT_JPEG == enPayLoadType[i])
                    {
                        fclose(pFile[i]);
                    }
                }
            }
        }
    }
    /*******************************************************
    * step 3 : close save-file
    *******************************************************/
    for (i = 0; i < s32ChnTotal; i++)
    {
        if(PT_JPEG != enPayLoadType[i])
        {
            fclose(pFile[i]);
        }
    }
    return NULL;
}

