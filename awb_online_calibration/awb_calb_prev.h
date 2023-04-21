#ifndef _HI_AWB_CALI_PREV_H_
#define _HI_AWB_CALI_PREV_H_

typedef struct HiAWB_CALI_PREV_S
{
    SAMPLE_VI_CONFIG_S stViConfig;
    SAMPLE_VO_CONFIG_S stVoConfig;
    VO_DEV             VoDev   ;
    VO_CHN             VoChn   ;
    VI_PIPE            ViPipe  ;
    VI_CHN             ViChn   ;
}AWB_CALI_PREV_S;

#endif
