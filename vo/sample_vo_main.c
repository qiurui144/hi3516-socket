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

#include "hi_common.h"
#include "sample_vo.h"
#include "mpi_sys.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */


/******************************************************************************
* function : show usage
******************************************************************************/
void SAMPLE_VO_Usage(char *sPrgNm)
{
    printf("Usage : %s <index>\n", sPrgNm);
    printf("index:\n");
    printf("\t 0)USERPIC -> VO ->6BITLCD.\n");
    printf("\t 1)USERPIC -> VO ->MIPILCD_1920_1080.\n");
    printf("\t 2)USERPIC -> VO ->MIPILCD_320_1280.\n");
    printf("\t 3)USERPIC -> VO ->MIPILCD_380_1920.\n");
    printf("\t 4)USERPIC -> VO ->MIPILCD_480_854.\n");

    printf("\t If you have any questions, please look at readme.txt!\n");
    return;
}

/******************************************************************************
* function    : main()
* Description : main
******************************************************************************/
#ifdef __HuaweiLite__
int app_main(int argc, char *argv[])
#else
int main(int argc, char *argv[])
#endif
{
    HI_S32 s32Ret = HI_FAILURE;
    HI_S32 s32Index;

    if (argc < 2 || argc > 2) {
        SAMPLE_VO_Usage(argv[0]);
        return HI_FAILURE;
    }

    if (!strncmp(argv[1], "-h", 2)) {
        SAMPLE_VO_Usage(argv[0]);
        return HI_SUCCESS;
    }

#ifdef __HuaweiLite__
#else
    signal(SIGINT, SAMPLE_VIO_HandleSig);
    signal(SIGTERM, SAMPLE_VIO_HandleSig);
#endif

    s32Index = atoi(argv[1]);
    switch (s32Index) {
        case 0:
            s32Ret = SAMPLE_VO_RGBLCD_6BIT();
            break;
        case 1:
            s32Ret = SAMPLE_VO_MIPILCD_1920_1080();
            break;
        case 2:
            s32Ret = SAMPLE_VO_MIPILCD_320_1280();
            break;
        case 3:
            s32Ret = SAMPLE_VO_MIPILCD_380_1920();
            break;
        case 4:
            s32Ret = SAMPLE_VO_MIPILCD_480_854();
            break;
        default:
            SAMPLE_PRT("the index %d is invaild!\n",s32Index);
            SAMPLE_VO_Usage(argv[0]);
            return HI_FAILURE;
    }

    if (s32Ret == HI_SUCCESS) {
        SAMPLE_PRT("sample_vo exit success!\n");
    } else {
        SAMPLE_PRT("sample_vo exit abnormally!\n");
    }

    return s32Ret;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
