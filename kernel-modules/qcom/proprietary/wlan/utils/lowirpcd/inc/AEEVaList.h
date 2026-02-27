/*
 * Copyright (c) 2004-2015 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc
 */
#ifndef AEEVALIST_H
#define AEEVALIST_H

#if !defined(__clang__) && (defined(__ARMCC_VERSION) || (defined(__GNUC__) && defined(__arm__)))



#if (defined(__ARMCC_VERSION) && __ARMCC_VERSION >= 200000 && !defined(__APCS_ADSABI)) || \
    (defined(__GNUC__) && defined(__arm__) && defined(__ARM_EABI__))

# define __AEEVA_ATPCS 0

#else

# define __AEEVA_ATPCS 1

#endif

typedef void* AEEVaList;

#define __AEEVA_ARGALIGN(t)   (((char*)(&((struct{char c;t x;}*)1)->x))-((char*)1))
#define __AEEVA_ARGSIZE(t)    ((sizeof(t)+sizeof(int)-1) & ~(sizeof(int)-1))

static __inline void __cpy(char*d, const char*s, int len)
{
   while (len-- > 0) *d++ = *s++;
}

static __inline AEEVaList __AEEVa_Arg(AEEVaList args, void* pv, int nVSize,
                                      int nArgSize, int nArgAlign)
{
   int   nArgs = (int)args & ~1;
   char* pcArgs = (char*)args;
   int   bATPCS = (int)args & 1;
   int   nArgsOffset = 0;
   int   nVOffset = 0;

   if (!bATPCS) { /* caller was compiled with AAPCS */

      if (nArgAlign > (int)sizeof(int)) {
         nArgAlign--; /* make a mask */
         pcArgs += ((nArgs + nArgAlign) & (int)~(unsigned)nArgAlign) - nArgs;
         /* move pv to next alignment */
      }
   }

#if defined(AEE_BIGENDIAN)
   if (nArgSize < (int)sizeof(int)) {
      nArgsOffset = (int)sizeof(int) - nArgSize;
   }
   nVOffset = nVSize - nArgSize;
#else
   (void)nVSize;
#endif /* AEE_BIGENDIAN */

   __cpy((char*)pv + nVOffset, (pcArgs - bATPCS) + nArgsOffset, nArgSize);

   /* round up */
   nArgSize = (nArgSize+(int)sizeof(int)-1) & ~((int)sizeof(int)-1);

   return pcArgs + nArgSize; /* increment va */
}

#define AEEVA_START(va,v)     ((va) = (char*)&(v) + __AEEVA_ARGSIZE(v) + __AEEVA_ATPCS)
#define AEEVA_ARG(va,v,t)     ((void)((va) = __AEEVa_Arg(va,&v,sizeof(v),sizeof(t),__AEEVA_ARGALIGN(t))))
#define AEEVA_END(va)         ((va) = (AEEVaList)0)
#define AEEVA_COPY(dest, src) ((void)((dest) = (src)))

#else /* !defined(__clang__) && (defined(__ARMCC_VERSION) || (defined(__GNUC__) && defined(__arm__))) */

#include <stdarg.h>

typedef va_list AEEVaList;

#define AEEVA_START(va,v)     (va_start((va), (v)))
#define AEEVA_ARG(va,v,t)     ((v) = va_arg((va),t))
#define AEEVA_END(va)         (va_end((va)))
#define AEEVA_COPY(dest, src) (va_copy((dest),(src)))

#endif/* !defined(__clang__) && (defined(__ARMCC_VERSION) || (defined(__GNUC__) && defined(__arm__))) */

#endif /* #ifndef AEEVALIST_H */

