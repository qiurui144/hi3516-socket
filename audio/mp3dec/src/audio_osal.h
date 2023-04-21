#ifndef __AUDIO_OSAL_H__
#define __AUDIO_OSAL_H__

#define AUDIO_OSAL_SECUREC

#ifdef AUDIO_OSAL_SECUREC
#include "securec.h"
#else
#include <string.h>
#include <stdio.h>
#endif

#include <pthread.h>


#ifdef AUDIO_OSAL_SECUREC
#define audio_memcpy(dest, destMax, src, count)                 memcpy_s(dest, destMax, src, count)
#define audio_memmove(dest, destMax, src, count)                memmove_s(dest, destMax, src, count)
#define audio_memset(dest, destMax, c, count)                   memset_s(dest, destMax, c, count)
#define audio_strcpy(strDest, destMax, strSrc)                  strcpy_s(strDest, destMax, strSrc)
#define audio_sprintf(strDest, destMax, format, ...)            sprintf_s(strDest, destMax, format, ##__VA_ARGS__)
#define audio_snprintf(strDest, destMax, count, format, ...)    snprintf_s(strDest, destMax, count, format, ##__VA_ARGS__)
#else
#define audio_memcpy(dest, destMax, src, count)                 memcpy(dest, src, count)
#define audio_memmove(dest, destMax, src, count)                memmove(dest, src, count)
#define audio_memset(dest, destMax, c, count)                   memset(dest, c, count)
#define audio_strcpy(strDest, destMax, strSrc)                  strcpy(strDest, strSrc)
#define audio_sprintf(strDest, destMax, format, ...)            sprintf(strDest, format, ##__VA_ARGS__)
#define audio_snprintf(strDest, destMax, count, format, ...)    snprintf(strDest, count, format, ##__VA_ARGS__)
#endif

#define AUDIO_MUTEX_LOCK(mutex)                                 (void)pthread_mutex_lock(&mutex)
#define AUDIO_MUTEX_UNLOCK(mutex)                               (void)pthread_mutex_unlock(&mutex)

#endif
