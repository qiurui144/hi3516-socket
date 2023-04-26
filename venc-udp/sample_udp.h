#ifndef __SAMPLE_VIO_H__
#define __SAMPLE_VIO_H__

#include "hi_common.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

/*******************************************************
    structure define
*******************************************************/
typedef struct hiYJSNPI_VENC_CONFIG_S
{
    PIC_SIZE_E			res[2]; 	// resolution 
	PAYLOAD_TYPE_E  	enc[2];		// enc. type
	SAMPLE_RC_E     	rc[2];		// rate control
	HI_U32			kbps[2];	// bitrate
	VENC_GOP_MODE_E 	gop[2];		// gop mode
	HI_BOOL			dis;		// dis en
	char  			save[128];	// ch0 save file name
	HI_S16			lport;		// udp listen port
	HI_S16			dport;		// udp dest. port
	char 			daddr[16];	// udp dest. addr (char)
	HI_U32          	profile[2];	// encode profile
	HI_BOOL			awb_strength_custom;
	HI_U8			awb_strength_r;
	HI_U8			awb_strength_b;
	HI_U16			awb_speed;
    
} YJSNPI_VENC_CONFIG_S;

typedef struct hiSAMPLE_VENC_GETSTREAM_PARA_S
{
    HI_BOOL bThreadStart;
    VENC_CHN VeChn[VENC_MAX_CHN_NUM];
    HI_S32  s32Cnt;
    YJSNPI_VENC_CONFIG_S conf;	// add by yjsnpi
} SAMPLE_YJSNPIVENC_GETSTREAM_PARA_S;


HI_S32 SAMPLE_COMM_YJSNPIVENC_StartGetStream(VENC_CHN VeChn[], HI_S32 s32Cnt, YJSNPI_VENC_CONFIG_S *pconf);
HI_VOID* SAMPLE_COMM_YJSNPIVENC_GetVencStreamProc(HI_VOID* p);
HI_S32 SAMPLE_COMM_YJSNPIVENC_Start(VENC_CHN VencChn, HI_S32 chn, YJSNPI_VENC_CONFIG_S *pconf, HI_BOOL bRcnRefShareBuf);
HI_S32 SAMPLE_COMM_YJSNPIVENC_Creat(VENC_CHN VencChn, HI_S32 chn, YJSNPI_VENC_CONFIG_S *pconf, HI_BOOL bRcnRefShareBuf);
HI_S32 SAMPLE_COMM_YJSNPIVENC_SaveStream(FILE* pFd, VENC_STREAM_S* pstStream);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif /* End of #ifndef __SAMPLE_VIO_H__*/
