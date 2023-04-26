#ifndef __SAMPLE_VO_H__
#define __SAMPLE_VO_H__

#include "hi_common.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#ifndef SAMPLE_PRT
#define SAMPLE_PRT(fmt...)   \
    do {\
        printf("[%s]-%d: ", __FUNCTION__, __LINE__);\
        printf(fmt);\
    }while(0)
#endif

#ifndef PAUSE
#define PAUSE()  do {\
        printf("---------------press Enter key to exit!---------------\n");\
        getchar();\
    } while (0)
#endif


void SAMPLE_VIO_HandleSig(HI_S32 signo);
HI_VOID SAMPLE_VOU_SYS_Exit(void);

HI_S32 SAMPLE_VO_RGBLCD_6BIT(HI_VOID);
HI_S32 SAMPLE_VO_MIPILCD_1920_1080(HI_VOID);
HI_S32 SAMPLE_VO_MIPILCD_320_1280(HI_VOID);
HI_S32 SAMPLE_VO_MIPILCD_380_1920(HI_VOID);
HI_S32 SAMPLE_VO_MIPILCD_480_854(HI_VOID);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif /* End of #ifndef __SAMPLE_VO_H__*/
